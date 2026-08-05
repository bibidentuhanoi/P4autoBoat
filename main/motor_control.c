#include "motor_control.h"
#include "drivers/esc_driver.h"
#include "drivers/winch_driver.h"
#include "drivers/steer_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "sensor_fusion.h"
#include "pipeline.h"
#include "transports/ws_transport.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

static const char *TAG = "MOTOR_CTL";

#define WATCHDOG_INTERVAL_US (100 * 1000)
#define STATUS_DIVIDER       10

/* New Kconfig symbols are absent until sdkconfig is regenerated. Keep safe
 * fallback defaults so this branch builds and dry-run works immediately. */
#ifndef CONFIG_HEADING_ASSIST_DRY_RUN
#define CONFIG_HEADING_ASSIST_DRY_RUN 1
#endif
#ifndef CONFIG_HEADING_ASSIST_BAND_DEG
#define CONFIG_HEADING_ASSIST_BAND_DEG "4.0"
#endif
#ifndef CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS
#define CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS 500
#endif
#ifndef CONFIG_HEADING_ASSIST_RUDDER_DEADBAND
#define CONFIG_HEADING_ASSIST_RUDDER_DEADBAND "0.08"
#endif
#ifndef CONFIG_HEADING_ASSIST_MAX_RUDDER
#define CONFIG_HEADING_ASSIST_MAX_RUDDER "0.25"
#endif
#ifndef CONFIG_HEADING_ASSIST_MAX_DIFF
#define CONFIG_HEADING_ASSIST_MAX_DIFF "0.06"
#endif

#define ASSIST_KP_MIN       0.012f
#define ASSIST_KP_BASE      0.022f
#define ASSIST_KP_MAX       0.050f
#define ASSIST_KD           0.030f
#define ASSIST_TRIM_RATE    0.002f   /* rudder units/sec while persistently outside band */
#define ASSIST_TRIM_MAX     0.120f
#define ASSIST_DIFF_GAIN    0.150f

/* ── Manual-driving control & safety model (settle it here, don't let it drift) ─
 *  Two independent power domains:
 *    • Thrusters (ESC pins 31/33): live ONLY in ARMED. Arming needs a GPS lock
 *      unless the dashboard sends force=true (bench override — OFF by default,
 *      a deliberate tick, so nothing arms unattended without a fix).
 *    • Servo rail (pin 36 → winch + both rudders): auto-powers on the first
 *      NON-ZERO winch/steer command, so manual driving has zero friction.
 *  Rail power rules:
 *    • Auto-power is suppressed after an explicit PWR-OFF (s_rail_cut) until an
 *      explicit PWR-ON — the operator's kill actually holds.
 *    • A zero/stop command never powers the rail (spring-back-to-0 must not
 *      re-energise it).
 *    • ARM powers the rail and clears the cut; DISARM cuts rail power.
 *    • WS-loss failsafe (100 ms) zeroes throttle + winch, centres rudders, and
 *      DE-ENERGISES the rail — loss of the control link returns to a safe,
 *      unpowered state, not a hot rail holding torque forever.
 * ───────────────────────────────────────────────────────────────────────────── */

static bool s_was_nonzero = false;
static esp_timer_handle_t s_watchdog = NULL;

/* Set when motor/servo state changes so the next 100ms watchdog tick publishes
 * status immediately (vs the ~1s cadence). Only the timer publishes it: doing so
 * from an incoming-command handler would deadlock — that path holds the shared
 * pipeline envelope mutex that publish also takes. */
static volatile bool s_status_dirty = false;

/* True after an explicit PWR-OFF: suppresses auto-power until an explicit PWR-ON
 * (or ARM). Without it the rail re-energised on the very next command — even a
 * spring-back-to-zero winch stop — so the operator's kill never held. */
static volatile bool s_rail_cut = false;

/* Last manual inputs, used only by the heading-assist dry-run logger for now.
 * The actual actuator path remains unchanged until dry-run is explicitly
 * promoted to active control. */
static float s_manual_left = 0.0f;
static float s_manual_right = 0.0f;
static float s_manual_rudder = 0.0f;

/* Heading assist dry-run state. */
static bool s_assist_cfg_loaded = false;
static float s_assist_band_deg = 4.0f;
static float s_assist_rudder_deadband = 0.08f;
static float s_assist_max_rudder = 0.25f;
static float s_assist_max_diff = 0.06f;
static bool s_assist_hold = false;
static bool s_assist_have_last_heading = false;
static float s_assist_target = 0.0f;
static float s_assist_last_heading = 0.0f;
static float s_assist_last_error = 0.0f;
static float s_assist_kp = ASSIST_KP_BASE;
static float s_assist_trim = 0.0f;
static int64_t s_assist_center_since_us = 0;
static int64_t s_assist_last_us = 0;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static float wrap180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float parse_cfg_float(const char *s, float fallback, float lo, float hi)
{
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || !isfinite(v)) {
        return fallback;
    }
    return clampf(v, lo, hi);
}

static void heading_assist_load_cfg_once(void)
{
    if (s_assist_cfg_loaded) return;
    s_assist_cfg_loaded = true;
    s_assist_band_deg = parse_cfg_float(CONFIG_HEADING_ASSIST_BAND_DEG, 4.0f, 0.5f, 30.0f);
    s_assist_rudder_deadband = parse_cfg_float(CONFIG_HEADING_ASSIST_RUDDER_DEADBAND, 0.08f, 0.01f, 0.50f);
    s_assist_max_rudder = parse_cfg_float(CONFIG_HEADING_ASSIST_MAX_RUDDER, 0.25f, 0.01f, 1.0f);
    s_assist_max_diff = parse_cfg_float(CONFIG_HEADING_ASSIST_MAX_DIFF, 0.06f, 0.0f, 0.5f);
}

static void heading_assist_reset(void)
{
    s_assist_hold = false;
    s_assist_center_since_us = 0;
    s_assist_have_last_heading = false;
    s_assist_last_us = 0;
    s_assist_last_error = 0.0f;
    s_assist_kp = ASSIST_KP_BASE;
    s_assist_trim = 0.0f;
}

static void heading_assist_dry_run_tick(void)
{
#if CONFIG_HEADING_ASSIST_DRY_RUN
    heading_assist_load_cfg_once();

    static uint32_t log_div = 0;
    const int64_t now = esp_timer_get_time();
    const bool log_now = (++log_div % STATUS_DIVIDER) == 0;

    int ws_clients = ws_transport_client_count();
    if (ws_clients == 0) {
        heading_assist_reset();
        return;
    }

    if (!imu_icm_ok()) {
        heading_assist_reset();
        if (log_now) {
            ESP_LOGI(TAG, "CTRL_DRY,disabled,reason=imu_bad");
        }
        return;
    }

    FusionResult fusion = {0};
    fusion_get_result(&fusion);

    float dt = 0.0f;
    float yaw_rate = 0.0f;
    if (s_assist_last_us != 0) {
        dt = (float)(now - s_assist_last_us) / 1000000.0f;
        if (dt > 0.001f && dt < 1.0f && s_assist_have_last_heading) {
            yaw_rate = wrap180(fusion.heading - s_assist_last_heading) / dt;
        }
    }
    s_assist_last_us = now;
    s_assist_last_heading = fusion.heading;
    s_assist_have_last_heading = true;

    const bool mag_ok = imu_mag_ok();
    const bool centered = fabsf(s_manual_rudder) <= s_assist_rudder_deadband;

    if (!mag_ok) {
        s_assist_hold = false;
    } else if (!centered) {
        s_assist_hold = false;
        s_assist_center_since_us = 0;
    } else {
        if (s_assist_center_since_us == 0) {
            s_assist_center_since_us = now;
        }
        if (!s_assist_hold &&
            (now - s_assist_center_since_us) >=
                (int64_t)CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS * 1000LL) {
            s_assist_target = fusion.heading;
            s_assist_last_error = 0.0f;
            s_assist_hold = true;
            ESP_LOGI(TAG, "CTRL_DRY,capture,target=%.1f,band=+/-%.1f",
                     s_assist_target, s_assist_band_deg);
        }
    }

    float error = s_assist_hold ? wrap180(s_assist_target - fusion.heading) : 0.0f;
    float p_error = 0.0f;
    if (fabsf(error) > s_assist_band_deg) {
        p_error = copysignf(fabsf(error) - s_assist_band_deg, error);
    }

    if (s_assist_hold && mag_ok && dt > 0.001f && dt < 1.0f) {
        if (p_error != 0.0f) {
            s_assist_trim += ASSIST_TRIM_RATE * dt * copysignf(1.0f, p_error);
            s_assist_trim = clampf(s_assist_trim, -ASSIST_TRIM_MAX, ASSIST_TRIM_MAX);

            if (fabsf(error) > fabsf(s_assist_last_error) + 0.2f &&
                (error * s_assist_last_error) >= 0.0f) {
                s_assist_kp = clampf(s_assist_kp + 0.002f * dt, ASSIST_KP_MIN, ASSIST_KP_MAX);
            } else if ((error * s_assist_last_error) < 0.0f) {
                s_assist_kp = clampf(s_assist_kp - 0.006f, ASSIST_KP_MIN, ASSIST_KP_MAX);
                s_assist_trim *= 0.98f;
            }
        }
        s_assist_last_error = error;
    }

    const float correction = s_assist_hold
        ? (s_assist_kp * p_error) - (ASSIST_KD * yaw_rate) + s_assist_trim
        : 0.0f;
    const float rudder_assist = clampf(correction, -s_assist_max_rudder, s_assist_max_rudder);
    const float diff_assist = clampf(correction * ASSIST_DIFF_GAIN, -s_assist_max_diff, s_assist_max_diff);
    const float left_would = clampf(s_manual_left + diff_assist, 0.0f, 1.0f);
    const float right_would = clampf(s_manual_right - diff_assist, 0.0f, 1.0f);
    const float rudder_would = clampf(s_manual_rudder + rudder_assist, -1.0f, 1.0f);

    gps_fix_t gps = {0};
    gps_driver_get_fix(&gps);

    if (log_now) {
        ESP_LOGI(TAG,
                 "CTRL_DRY,%s,tgt=%.1f,hdg=%.1f,err=%.1f,yaw=%.1f,band=%.1f,kp=%.3f,trim=%.3f,user_rud=%.2f,rud_would=%.2f,diff=%.3f,L=%.2f,R=%.2f,mag=%d,gps_v=%.2f",
                 s_assist_hold ? "hold" : (centered ? "capture_wait" : "override"),
                 s_assist_target, fusion.heading, error, yaw_rate, s_assist_band_deg,
                 s_assist_kp, s_assist_trim, s_manual_rudder, rudder_would,
                 diff_assist, left_would, right_would, mag_ok ? 1 : 0, gps.speed_mps);
    }
#endif
}

static void motor_command_handler(const boat_MotorCommand *cmd)
{
    float left, right;

    if (cmd->left != 0.0f || cmd->right != 0.0f) {
        left  = cmd->left;
        right = cmd->right;
    } else {
        float throttle = cmd->throttle;
        float rudder   = cmd->rudder;
        left  = throttle + rudder;
        right = throttle - rudder;
        float max_abs = fmaxf(fabsf(left), fabsf(right));
        if (max_abs > 1.0f) {
            left  /= max_abs;
            right /= max_abs;
        }
    }

    s_was_nonzero = (left != 0.0f || right != 0.0f);
    s_manual_left = clampf(left, 0.0f, 1.0f);
    s_manual_right = clampf(right, 0.0f, 1.0f);
    esc_driver_set_throttle(left, right);
}

/* Winch + rudders share the pin-36 servo rail. A non-zero command auto-powers
 * it (zero-friction manual driving) — unless the operator explicitly cut it. */
static void ensure_servo_rail(void)
{
    if (s_rail_cut) return;                 /* respect an explicit PWR-OFF */
    if (!winch_driver_get_power()) {
        steer_driver_reassert();            /* guarantee the pad matches before power arrives */
        winch_driver_set_power(true);
        s_status_dirty = true;              /* reflect PWR-on to the dashboard next tick */
    }
}

static void winch_command_handler(const boat_WinchCommand *cmd)
{
    if (cmd->speed != 0.0f) ensure_servo_rail();   /* a stop never powers the rail */
    winch_driver_set_speed(cmd->speed);
}

static void steer_command_handler(const boat_SteerCommand *cmd)
{
    /* One unified rudder. Collapse the legacy left/right wire fields into the
     * single physical steering axis. */
    float steer = 0.0f;
    if (cmd->left != 0.0f && cmd->right != 0.0f) {
        steer = 0.5f * (cmd->left + cmd->right);
    } else {
        steer = (cmd->left != 0.0f) ? cmd->left : cmd->right;
    }
    s_manual_rudder = clampf(steer, -1.0f, 1.0f);
    if (steer != 0.0f) ensure_servo_rail();
    steer_driver_set(steer);
}

/* Calibration only — see steer_driver_set_raw_us(). Always powers the rail:
 * there's no point sending a raw pulse to find a mechanical stop if the
 * servo can't move to prove it. */
static void steer_raw_command_handler(const boat_SteerRawCommand *cmd)
{
    ensure_servo_rail();
    steer_driver_set_raw_us(cmd->pulse_us);
}

/* Explicit servo-rail switch. PWR-OFF latches s_rail_cut so auto-power stays off
 * until PWR-ON — the operator's kill holds. Off also zeroes the commanded winch
 * and homes steer back to STEER_HOME (full-right) — the operator's confirmed
 * hand-positioned reference — so the state firmware reports matches where the
 * rudder should physically be re-homed to before the next power-up. */
static void servo_power_command_handler(bool on)
{
    s_rail_cut = !on;
    if (winch_driver_set_power(on) == ESP_OK && !on) {
        winch_driver_set_speed(0.0f);   /* rail off ⇒ make commanded state match */
        if (steer_driver_get() != 1.0f) {
            steer_driver_set(1.0f);
            s_manual_rudder = 1.0f;
        }
        heading_assist_reset();
    }
    s_status_dirty = true;
}

static void arm_task_fn(void *arg)
{
    intptr_t v = (intptr_t)arg;
    bool do_arm = (v & 1) != 0;
    bool force  = (v & 2) != 0;
    if (do_arm) {
        motor_control_arm(force);
    } else {
        motor_control_disarm();
    }
    vTaskDelete(NULL);
}

static void arm_command_handler(bool arm, bool force)
{
    ESP_LOGI(TAG, "%s command received%s", arm ? "Arm" : "Disarm",
             (arm && force) ? " (GPS override)" : "");
    if (arm && esc_driver_get_state() != ESC_STATE_DISARMED) return;
    if (!arm && esc_driver_get_state() == ESC_STATE_DISARMED) return;
    intptr_t v = (arm ? 1 : 0) | (force ? 2 : 0);
    /* 2048 was too tight: esc_driver_arm() (MCPWM setup + 3s arm delay + logging)
     * followed by steer_driver_reassert() (mutex + MCPWM + a float-formatted log
     * line) overflowed it — confirmed via a Stack protection fault with the
     * reported SP sitting just below the task's own stack bounds. 4096 gives
     * real headroom instead of refitting to the exact previous call depth. */
    xTaskCreate(arm_task_fn, "esc_arm", 4096, (void *)v, 5, NULL);
}

static void publish_status(void)
{
    boat_MotorStatus ms = boat_MotorStatus_init_zero;
    ms.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&ms.left_throttle, &ms.right_throttle);
    ms.winch_speed = winch_driver_get_speed();
    ms.servo_power = winch_driver_get_power();
    pipeline_publish_motor_status(&ms);
}

static void watchdog_cb(void *arg)
{
    (void)arg;
    static int tick = 0;

    /* Failsafe on WS loss: covers ARMED, and the bench case where the servo rail
     * is powered while disarmed. Return to a safe, DE-ENERGISED state. */
    if ((esc_driver_get_state() == ESC_STATE_ARMED || winch_driver_get_power()) &&
        ws_transport_client_count() == 0) {
        bool acted = false;
        if (s_was_nonzero) {
            esc_driver_set_throttle(0.0f, 0.0f);
            s_was_nonzero = false;
            s_manual_left = 0.0f;
            s_manual_right = 0.0f;
            acted = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            acted = true;
        }
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);   /* WS-loss: center, not STEER_HOME — safer than a hard-over rudder */
            s_manual_rudder = 0.0f;
            acted = true;
        }
        if (winch_driver_get_power()) {
            winch_driver_set_power(false);   /* de-energise the rail — don't hold torque forever */
            heading_assist_reset();
            acted = true;
        }
        if (acted) {
            ESP_LOGW(TAG, "WS lost — throttle 0, winch 0, rudders centred, servo rail cut");
            s_status_dirty = true;
        }
    }

    heading_assist_dry_run_tick();

    ++tick;
    if (s_status_dirty || (tick % STATUS_DIVIDER) == 0) {
        s_status_dirty = false;
        publish_status();
    }
}

esp_err_t motor_control_init_hw(void)
{
    esp_err_t ret = esc_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESC driver init failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t motor_control_init(void)
{
    pipeline_register_motor_handler(motor_command_handler);
    pipeline_register_arm_handler(arm_command_handler);
    pipeline_register_winch_handler(winch_command_handler);
    pipeline_register_steer_handler(steer_command_handler);
    pipeline_register_servo_power_handler(servo_power_command_handler);
    pipeline_register_steer_raw_handler(steer_raw_command_handler);

    const esp_timer_create_args_t timer_args = {
        .callback = watchdog_cb,
        .name     = "motor_wd",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_watchdog), TAG, "timer_create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_watchdog, WATCHDOG_INTERVAL_US), TAG, "timer_start");

    ESP_LOGI(TAG, "Motor control initialized (status ~1 Hz)");
    return ESP_OK;
}

esp_err_t motor_control_arm(bool force)
{
    if (!force && !gps_driver_has_lock()) {
        ESP_LOGW(TAG, "Arm refused — waiting for GPS lock (use override to bypass)");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = esc_driver_arm();
    if (ret == ESP_OK) {
        s_rail_cut = false;             /* arming = go live: re-enable the rail */
        steer_driver_reassert();        /* guarantee the pad matches before power arrives */
        winch_driver_set_power(true);   /* arming always powers the servo rail */
    }
    s_status_dirty = true;
    return ret;
}

esp_err_t motor_control_disarm(void)
{
    s_was_nonzero = false;
    s_manual_left = 0.0f;
    s_manual_right = 0.0f;
    s_manual_rudder = 0.0f;
    heading_assist_reset();
    winch_driver_set_speed(0.0f);       /* stop the winch */
    winch_driver_set_power(false);      /* cut servo power */
    if (steer_driver_get() != 0.0f) {   /* disarm: center, not STEER_HOME — see WS-loss failsafe */
        steer_driver_set(0.0f);
    }
    s_status_dirty = true;
    return esc_driver_disarm();
}
