#include "motor_control.h"
#include "arm_sequence.h"
#include "esc_trim.h"
#include "esc_trim_cal.h"
#include "file_system.h"
#include "drivers/esc_driver.h"
#include "drivers/winch_driver.h"
#include "drivers/steer_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "sensor_fusion.h"
#include "stability_control.h"
#include "pipeline.h"
#include "control_arbiter.h"
#include "runtime_metrics.h"
#include "runtime_task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MOTOR_CTL";

#define CONTROL_PERIOD_MS    10
#define HEADING_DIVIDER      10
#define STATUS_DIVIDER       10
#define ARMING_DURATION_US   3000000LL
#define ARM_REQUEST_QUEUE_LENGTH 4
#define ARM_ACTION_QUEUE_LENGTH 4

/* How long accepted manual-control traffic may go silent before failsafe.
 * Detect and all other decodable protobuf traffic are deliberately excluded. */
#define CONTROL_LINK_TIMEOUT_US CONTROL_DRIVE_TIMEOUT_US

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
#ifndef CONFIG_STABILITY_SAS_ENABLE
#define CONFIG_STABILITY_SAS_ENABLE 0
#endif
#ifndef CONFIG_STABILITY_SAS_RMAX_DPS
#define CONFIG_STABILITY_SAS_RMAX_DPS 45
#endif
#ifndef CONFIG_STABILITY_SAS_KR
#define CONFIG_STABILITY_SAS_KR "0.010"
#endif
#ifndef CONFIG_STABILITY_SAS_YAW_TAU_S
#define CONFIG_STABILITY_SAS_YAW_TAU_S "0.15"
#endif
#ifndef CONFIG_STABILITY_SAS_OUT_CAP
#define CONFIG_STABILITY_SAS_OUT_CAP "0.50"
#endif
#ifndef CONFIG_STABILITY_SAS_MAX_AGE_MS
#define CONFIG_STABILITY_SAS_MAX_AGE_MS 200
#endif
#define STAB_LOG_DIVIDER 5   /* throttle the CTRL_SAS,active line to ~every 5th correction */

/* ESC differential-trim auto-calibration (Milestone 2). Its excessive-yaw abort
 * is its OWN value, deliberately NOT reused from CONFIG_STABILITY_SAS_RMAX_DPS:
 * calibration runs with SAS suppressed and must not inherit a dependency on how
 * the pilot's stick feel was tuned (spec 2026-08-12-esc-trim-autocal-design.md
 * sec 4/7). Seeded from the same number, separate symbol. */
#ifndef CONFIG_STABILITY_TRIMCAL_KI
#define CONFIG_STABILITY_TRIMCAL_KI "0.02"
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_CLAMP
#define CONFIG_STABILITY_TRIMCAL_CLAMP "0.30"
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_ACCEPT_K
#define CONFIG_STABILITY_TRIMCAL_ACCEPT_K "3.0"
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_MIN_SPEED_MPS
#define CONFIG_STABILITY_TRIMCAL_MIN_SPEED_MPS "0.3"
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_MAX_YAW_DPS
#define CONFIG_STABILITY_TRIMCAL_MAX_YAW_DPS 45
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_WINDOW_TICKS
#define CONFIG_STABILITY_TRIMCAL_WINDOW_TICKS 100
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_NOISE_TICKS
#define CONFIG_STABILITY_TRIMCAL_NOISE_TICKS 100
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_SETTLE_MS
#define CONFIG_STABILITY_TRIMCAL_SETTLE_MS 400
#endif
#ifndef CONFIG_STABILITY_TRIMCAL_LEVEL_TIMEOUT_MS
#define CONFIG_STABILITY_TRIMCAL_LEVEL_TIMEOUT_MS 15000
#endif
/* How long calibration may go without a CalibrateCommand keepalive from its
 * dashboard before aborting. Longer than the 400ms driving-link timeout: the
 * dashboard keepalive is ~1 Hz, and a tethered supervised sweep can tolerate a
 * couple of seconds of dashboard silence before treating it as "operator gone". */
#define CAL_LINK_TIMEOUT_US 2000000LL

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
 *    • Control-link-loss failsafe (10 ms control loop) zeroes throttle + winch,
 *      centres rudders, and DE-ENERGISES the rail — loss of the control link
 *      returns to a safe, unpowered state, not a hot rail holding torque
 *      forever. A failsafe rail cut is not latched; only an explicit PWR-OFF
 *      suppresses later non-zero steering/winch auto-power.
 * ───────────────────────────────────────────────────────────────────────────── */

static control_arbiter_t s_arbiter;
static portMUX_TYPE s_arbiter_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_last_control_rx_us = 0;
static bool s_control_failsafe = true;
static TaskHandle_t s_control_task = NULL;
static TaskHandle_t s_arm_sequence_task = NULL;
static QueueHandle_t s_arm_request_queue = NULL;
static QueueHandle_t s_arm_action_queue = NULL;

static boat_MotorStatus s_status_buffers[2];
static uint32_t s_status_generation;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static atomic_uintptr_t s_status_reader_task;

static bool s_arm_power_allowed;

/* ESC differential-trim table -- own NVS record ("esc_trim"), loaded once at
 * boot by main.c (fs_load_esc_trim) and handed in via motor_control_set_esc_trim().
 * Applied on the common throttle in control_apply_decision(), before
 * esc_driver_set_throttle(). Entirely independent of CalibrationData. */
static EscTrimPoint s_esc_trim[ESC_TRIM_MAX_POINTS];
static uint8_t s_esc_trim_count = 0;

/* True while ESC-trim auto-calibration is actively driving the ESCs itself --
 * gates trim application off (control_apply_decision) and suppresses SAS
 * (stability_sas_tick) so neither fights the calibration routine's probing.
 * Owned by the control task: only calibration_tick writes it. */
static bool s_calibrating = false;

/* ESC-trim calibration state. The pure state machine (esc_trim_cal) lives in
 * s_cal; s_cal_cfg is loaded once from Kconfig. The two *_pending flags and
 * s_cal_last_rx_us are the cross-task handoff from the pipeline RX task's
 * calibrate_command_handler -- written only under s_arbiter_lock.
 *
 * s_cal_last_rx_us is calibration's OWN heartbeat, entirely separate from the
 * general s_last_control_rx_us link timer: CalibrateCommand traffic must never
 * refresh the driving-link liveness, and a manual command arriving mid-sweep
 * (which DOES advance s_last_control_rx_us past s_cal_baseline_rx_us) is an
 * abort, not a keep-alive. See design doc sec 7a. */
static etc_t s_cal;
static etc_cfg_t s_cal_cfg;
static bool s_cal_cfg_loaded = false;
static bool s_cal_start_pending = false;
static bool s_cal_stop_pending = false;
static bool s_cal_average = false;
static int64_t s_cal_baseline_rx_us = 0;
static int64_t s_cal_last_rx_us = 0;
static etc_out_t s_cal_out;   /* this tick's step output (ESC commands + done/abort) */

/* CalibrateStatus telemetry: written by the control task, read+published by the
 * core-1 diagnostics task (same producer/consumer split as motor status).
 * Guarded by s_status_lock; generation == 0 means "never updated" -> the
 * diagnostics task publishes nothing when idle. */
static boat_CalibrateStatus s_cal_status;
static uint32_t s_cal_status_generation = 0;

typedef struct {
    arm_request_t request;
    bool force;
    int64_t requested_us;
} arm_request_message_t;

/* Pure command recency -- deliberately does NOT treat an open WS connection
 * as sufficient on its own. tests/test_runtime_architecture.py's
 * test_pipeline_handlers_do_not_write_actuators asserts throttle/winch/steer
 * all zero out after CONTROL_LINK_TIMEOUT_US of silence with the WS-client
 * stub fixed at "connected" the whole time -- i.e. a stale browser tab
 * holding an open socket must still fail safe. (2026-08-10: an earlier
 * revision of this fix OR'd in ws_transport_client_count() > 0 to solve the
 * ARMING-duration problem below; that broke this exact test, correctly --
 * "still connected" and "still receiving commands" are different safety
 * claims, and only the second one should keep throttle live.) */
static inline bool control_link_alive(void)
{
    int64_t last;
    portENTER_CRITICAL(&s_arbiter_lock);
    last = s_last_control_rx_us;
    portEXIT_CRITICAL(&s_arbiter_lock);
    return last != 0 && esp_timer_get_time() - last < CONTROL_LINK_TIMEOUT_US;
}

/* True after an explicit PWR-OFF: suppresses auto-power until an explicit PWR-ON
 * (or ARM). Without it the rail re-energised on the very next command — even a
 * spring-back-to-zero winch stop — so the operator's kill never held. */
static bool s_rail_cut = false;

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
#if CONFIG_STABILITY_SAS_ENABLE
static stab_state_t s_stab_state;
static stab_cfg_t s_stab_cfg;
static bool s_stab_cfg_loaded = false;
static uint32_t s_stab_last_seq = 0;
static int64_t s_stab_last_tick_us = 0;
static uint32_t s_stab_log_div = 0;
#endif

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

    if (!control_link_alive()) {
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

static bool motor_status_equal(const boat_MotorStatus *a, const boat_MotorStatus *b)
{
    return a->state == b->state &&
           a->left_throttle == b->left_throttle &&
           a->right_throttle == b->right_throttle &&
           a->winch_speed == b->winch_speed &&
           a->servo_power == b->servo_power;
}

static void status_commit_current(bool force)
{
    boat_MotorStatus status = boat_MotorStatus_init_zero;
    status.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&status.left_throttle, &status.right_throttle);
    status.winch_speed = winch_driver_get_speed();
    status.servo_power = winch_driver_get_power();

    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_status_generation;
    if (!force && motor_status_equal(&status, &s_status_buffers[generation & 1U])) {
        portEXIT_CRITICAL(&s_status_lock);
        return;
    }

    uint32_t next = generation + 1U;
    s_status_buffers[next & 1U] = status;
    s_status_generation = next;
    portEXIT_CRITICAL(&s_status_lock);

    TaskHandle_t reader = (TaskHandle_t)atomic_load_explicit(&s_status_reader_task,
                                                             memory_order_acquire);
    if (reader) xTaskNotifyGive(reader);
}

uint32_t motor_control_get_status(boat_MotorStatus *out)
{
    if (!out) return 0;
    atomic_store_explicit(&s_status_reader_task, (uintptr_t)xTaskGetCurrentTaskHandle(),
                          memory_order_release);

    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_status_generation;
    *out = s_status_buffers[generation & 1U];
    portEXIT_CRITICAL(&s_status_lock);
    return generation;
}

/* Publish one CalibrateStatus snapshot for the diagnostics task to pick up.
 * Called from the control task (calibration_tick) only. Bumps the generation
 * so the reader can tell a fresh update from a repeat. yaw_avg/making_way are
 * computed by the caller (it already has the fusion sample and GPS fix). */
static void cal_status_commit(const etc_t *cal, float yaw_avg, bool making_way)
{
    boat_CalibrateStatus st = boat_CalibrateStatus_init_zero;
    st.state = (uint32_t)cal->state;
    st.level_index = cal->level_idx;
    st.level_throttle = (cal->level_idx < ESC_TRIM_MAX_POINTS)
                            ? s_cal_cfg.levels[cal->level_idx] : 0.0f;
    st.trim_diff = cal->trim_diff;
    st.yaw_avg_dps = yaw_avg;
    st.making_way = making_way;
    st.points_done = cal->out_count;

    portENTER_CRITICAL(&s_status_lock);
    s_cal_status = st;
    s_cal_status_generation++;
    portEXIT_CRITICAL(&s_status_lock);
}

uint32_t motor_control_get_calibrate_status(boat_CalibrateStatus *out)
{
    if (!out) return 0;
    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_cal_status_generation;
    *out = s_cal_status;
    portEXIT_CRITICAL(&s_status_lock);
    return generation;
}

static void notify_link_rx_locked(int64_t received_us)
{
    if (received_us > s_last_control_rx_us) s_last_control_rx_us = received_us;

    control_drive_proposal_t *drive = &s_arbiter.drive[CONTROL_SOURCE_MANUAL];
    if (drive->valid && received_us > drive->rx_us) drive->rx_us = received_us;
}

void motor_control_notify_link_rx(int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
}

static void count_submit_result(control_submit_result_t result, bool overwrote)
{
    if (result == CONTROL_ACCEPTED && overwrote) {
        runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_COMMAND_OVERWRITE);
    } else if (result != CONTROL_ACCEPTED) {
        runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_INVALID_COMMAND);
    }
}

/* This module is the manual transport boundary. Source-less discrete arbiter
 * events are reachable only through this static wrapper, never from a future
 * waypoint or ML producer. */
static control_submit_result_t manual_transport_control_arbiter_submit_drive(
    float throttle, float rudder, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.drive[CONTROL_SOURCE_MANUAL].changed;
    control_submit_result_t result = control_arbiter_submit_drive(
        &s_arbiter, CONTROL_SOURCE_MANUAL, throttle, rudder, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_winch(
    float winch, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.winch.changed;
    control_submit_result_t result = control_arbiter_submit_winch(
        &s_arbiter, CONTROL_SOURCE_MANUAL, winch, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_steer(
    float steer, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.steer.changed;
    control_submit_result_t result = control_arbiter_submit_steer(
        &s_arbiter, CONTROL_SOURCE_MANUAL, steer, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_steer_raw(
    uint32_t pulse_us, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.steer.changed;
    control_submit_result_t result = control_arbiter_submit_steer_raw(
        &s_arbiter, CONTROL_SOURCE_MANUAL, pulse_us, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static bool manual_transport_control_arbiter_push_event(control_event_kind_t event,
                                                        int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool accepted = control_arbiter_push_event(&s_arbiter, event);
    if (accepted) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    if (!accepted) runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_INVALID_COMMAND);
    return accepted;
}

static void motor_command_handler(const boat_MotorCommand *cmd)
{
    float left = cmd->left != 0.0f || cmd->right != 0.0f
               ? cmd->left : cmd->throttle + cmd->rudder;
    float right = cmd->left != 0.0f || cmd->right != 0.0f
                ? cmd->right : cmd->throttle - cmd->rudder;
    if (fmaxf(fabsf(left), fabsf(right)) > 1.0f) {
        float scale = fmaxf(fabsf(left), fabsf(right));
        left /= scale;
        right /= scale;
    }

    float throttle = 0.5f * (left + right);
    float rudder = 0.5f * (left - right);
    manual_transport_control_arbiter_submit_drive(throttle, rudder,
                                                   esp_timer_get_time());
}

static void winch_command_handler(const boat_WinchCommand *cmd)
{
    manual_transport_control_arbiter_submit_winch(cmd->speed, esp_timer_get_time());
}

static void steer_command_handler(const boat_SteerCommand *cmd)
{
    float steer = 0.0f;
    if (cmd->left != 0.0f && cmd->right != 0.0f) {
        steer = 0.5f * (cmd->left + cmd->right);
    } else {
        steer = (cmd->left != 0.0f) ? cmd->left : cmd->right;
    }
    manual_transport_control_arbiter_submit_steer(steer, esp_timer_get_time());
}

static void steer_raw_command_handler(const boat_SteerRawCommand *cmd)
{
    manual_transport_control_arbiter_submit_steer_raw(cmd->pulse_us,
                                                       esp_timer_get_time());
}

static void servo_power_command_handler(bool on)
{
    control_event_kind_t event = on ? CONTROL_EVENT_SERVO_POWER_ON
                                    : CONTROL_EVENT_SERVO_POWER_OFF;
    if (manual_transport_control_arbiter_push_event(event, esp_timer_get_time()) &&
        !on && s_control_task) {
        xTaskNotifyGive(s_control_task);
    }
}

static void arm_command_handler(bool arm, bool force)
{
    ESP_LOGI(TAG, "%s command received%s", arm ? "Arm" : "Disarm",
             (arm && force) ? " (GPS override)" : "");
    control_event_kind_t event = arm
        ? (force ? CONTROL_EVENT_FORCE_ARM : CONTROL_EVENT_ARM)
        : CONTROL_EVENT_DISARM;
    if (manual_transport_control_arbiter_push_event(event, esp_timer_get_time()) &&
        !arm && s_control_task) {
        xTaskNotifyGive(s_control_task);
    }
}

/* Runs in the pipeline RX task, NOT the control task. Only sets a pending flag
 * for the control task's calibration_tick to consume -- it must never actuate
 * from here, and it must NOT call notify_link_rx_locked / any manual_transport_*
 * wrapper: CalibrateCommand traffic is deliberately invisible to the driving
 * link heartbeat (design doc sec 7a). s_cal_last_rx_us is calibration's own,
 * separate liveness signal. */
static void calibrate_command_handler(bool start, bool average)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    if (start) {
        s_cal_start_pending = true;
        s_cal_average = average;
    } else {
        s_cal_stop_pending = true;
    }
    s_cal_last_rx_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_arbiter_lock);
}

static bool submit_arm_request(arm_request_t request, bool force, int64_t requested_us)
{
    arm_request_message_t message = {
        .request = request,
        .force = force,
        .requested_us = requested_us,
    };

    BaseType_t queued;
    if (request == ARM_REQUEST_DISARM) {
        xQueueReset(s_arm_request_queue);
        queued = xQueueSendToFront(s_arm_request_queue, &message, 0);
    } else {
        queued = xQueueSend(s_arm_request_queue, &message, 0);
    }
    if (queued != pdTRUE) {
        runtime_metrics_count(RUNTIME_TASK_ARM_SEQUENCE,
                              RUNTIME_EVENT_INVALID_COMMAND);
        ESP_LOGE(TAG, "ArmSeq request queue full");
        return false;
    }
    return true;
}

static void publish_arm_action(arm_action_t action)
{
    if (action == ARM_ACTION_REJECT_NO_GPS) {
        ESP_LOGW(TAG, "Arm refused — waiting for GPS lock (use override to bypass)");
        return;
    }
    if (action == ARM_ACTION_NONE) return;

    BaseType_t queued;
    if (action == ARM_ACTION_DISARM) {
        xQueueReset(s_arm_action_queue);
        queued = xQueueSendToFront(s_arm_action_queue, &action, 0);
    } else {
        queued = xQueueSend(s_arm_action_queue, &action, 0);
    }
    if (queued != pdTRUE) {
        runtime_metrics_count(RUNTIME_TASK_ARM_SEQUENCE,
                              RUNTIME_EVENT_INVALID_COMMAND);
        ESP_LOGE(TAG, "ArmSeq action queue full");
        return;
    }
    if (s_control_task) xTaskNotifyGive(s_control_task);
}

static TickType_t arm_sequence_wait_ticks(const arm_sequence_t *sequence,
                                          int64_t now_us)
{
    if (sequence->state != ARM_SEQUENCE_ARMING) return portMAX_DELAY;
    if (sequence->deadline_us <= now_us) return 0;

    uint64_t remaining_ms = (uint64_t)(sequence->deadline_us - now_us + 999) / 1000;
    TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
    return ticks == 0 ? 1 : ticks;
}

static void task_arm_sequence(void *arg)
{
    (void)arg;
    arm_sequence_t sequence;
    arm_sequence_init(&sequence, ARMING_DURATION_US);

    for (;;) {
        arm_request_message_t message;
        int64_t before_wait_us = esp_timer_get_time();
        TickType_t wait = arm_sequence_wait_ticks(&sequence, before_wait_us);
        if (xQueueReceive(s_arm_request_queue, &message, wait) == pdTRUE) {
            arm_sequence_request(&sequence, message.request, message.force,
                                 message.requested_us);
        }

        int64_t now_us = esp_timer_get_time();
        bool gps_locked = false;
        if (sequence.state == ARM_SEQUENCE_ARM_PENDING) {
            gps_locked = sequence.force || gps_driver_has_lock();
        }
        publish_arm_action(arm_sequence_step(&sequence, now_us, gps_locked));
    }
}

static bool control_apply_arm_action(control_decision_t *decision, bool safe_stop,
                                     bool *changed)
{
    arm_action_t action;
    if (xQueueReceive(s_arm_action_queue, &action, 0) != pdTRUE) return false;

    switch (action) {
    case ARM_ACTION_BEGIN:
        if (safe_stop || !s_arm_power_allowed) {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        } else if (esc_driver_arm_begin() == ESP_OK) {
            *changed = true;
        } else {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        }
        break;

    case ARM_ACTION_COMPLETE:
        if (safe_stop || !s_arm_power_allowed) {
            if (esc_driver_get_state() != ESC_STATE_DISARMED) {
                esc_driver_disarm();
                *changed = true;
            }
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        } else if (esc_driver_arm_complete() == ESP_OK) {
            decision->servo_power_on = true;
            *changed = true;
        } else {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        }
        break;

    case ARM_ACTION_DISARM:
        if (esc_driver_get_state() != ESC_STATE_DISARMED) {
            esc_driver_disarm();
            *changed = true;
        }
        decision->arm = false;
        decision->force_arm = false;
        decision->disarm = true;
        return true;

    case ARM_ACTION_NONE:
    case ARM_ACTION_REJECT_NO_GPS:
    default:
        break;
    }
    return false;
}

static void control_apply_decision(control_decision_t *decision)
{
    bool changed = false;
    bool explicit_off = decision->servo_power_off;
    /* Arm-gating and the failsafe-zeroing branch below both use general link
     * liveness (control_link_alive(), refreshed by ANY accepted arbiter
     * command incl. the arm click itself), not decision->failsafe.
     * decision->failsafe is drive-proposal-specific: drive->valid only
     * becomes true once a throttle/rudder command has actually been
     * submitted, so on a fresh connect (arm clicked before ever touching the
     * throttle) failsafe was permanently true, which alone blocked arming,
     * and ALSO used to gate the zeroing branch below (which unconditionally
     * clears s_arm_power_allowed whenever it fires) -- the two branches were
     * fighting until both were switched to safe_stop (2026-08-10).
     *
     * safe_stop ALSO grants a grace exception while ESC_STATE_ARMING:
     * CONTROL_LINK_TIMEOUT_US (400ms) is shorter than ARMING_DURATION_US
     * (3s), so a user who clicks ARM once and touches nothing else for the
     * rest of the sequence would otherwise have control_link_alive() go
     * false ~400ms in, well before ARM_ACTION_COMPLETE arrives 3s later --
     * confirmed on hardware, ARMING pulsed for the full duration then
     * reverted to DISARMED right at completion. Scoped to ESC_STATE_ARMING
     * specifically (not "connected", not any other state) so it can't mask
     * a genuinely dead link once armed and driving: an earlier revision
     * tried OR-ing in ws_transport_client_count() > 0 instead, which fixed
     * this but broke tests/test_runtime_architecture.py's
     * test_pipeline_handlers_do_not_write_actuators -- that test explicitly
     * holds a WS-connected stub for CONTROL_LINK_TIMEOUT_US of silence and
     * asserts throttle/winch/steer all zero out anyway, i.e. a stale
     * browser tab holding an open socket must still fail safe. "Still
     * connected" and "still receiving commands" are different safety
     * claims; only mid-arming specifically needed the exception. */
    bool safe_stop = explicit_off || decision->disarm ||
                     (!control_link_alive() &&
                      esc_driver_get_state() != ESC_STATE_ARMING);
    bool internal_disarm = control_apply_arm_action(decision, safe_stop, &changed);
    safe_stop = explicit_off || decision->disarm ||
               (!control_link_alive() && esc_driver_get_state() != ESC_STATE_ARMING);

    if (explicit_off) {
        s_rail_cut = true;
        s_arm_power_allowed = false;
        if (winch_driver_get_power()) {
            winch_driver_set_power(false);
            changed = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            changed = true;
        }
        if (steer_driver_get() != 1.0f) {
            steer_driver_set(1.0f);
            changed = true;
        }
        s_manual_rudder = 1.0f;
        heading_assist_reset();
    } else if (safe_stop) {
        s_arm_power_allowed = false;
        float left;
        float right;
        esc_driver_get_throttle(&left, &right);
        if (left != 0.0f || right != 0.0f) {
            esc_driver_set_throttle(0.0f, 0.0f);
            changed = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            changed = true;
        }
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);
            changed = true;
        }
        /* Keep the servo rail powered WHILE calibrating: the driving link is
         * stale by design during a sweep (sec 7a), so this safe_stop fires every
         * tick -- but calibration is actively holding the rudder centred and
         * needs the rail live to do it. Cutting it would let the rudder float
         * off-centre and corrupt the yaw measurement. An explicit PWR-OFF still
         * cuts the rail (the explicit_off branch above, not this one) and aborts
         * calibration via s_rail_cut. */
        if (winch_driver_get_power() && !s_calibrating) {
            winch_driver_set_power(false);
            changed = true;
        }
        s_manual_left = 0.0f;
        s_manual_right = 0.0f;
        s_manual_rudder = 0.0f;
        heading_assist_reset();
        if (!control_link_alive() && changed && !s_calibrating) {
            ESP_LOGW(TAG, "Control link lost — throttle 0, winch 0, rudders centred, servo rail cut");
        }
    }

    if (decision->disarm && !internal_disarm) {
        submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
    }

    if (!safe_stop) {
        if (decision->arm || decision->force_arm) {
            s_arm_power_allowed = true;
            submit_arm_request(ARM_REQUEST_ARM, decision->force_arm,
                               esp_timer_get_time());
        }

        if (decision->servo_power_on) {
            s_rail_cut = false;
            s_arm_power_allowed = true;
            if (!winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
        }

        if (decision->drive_changed) {
            float left;
            float right;
            esc_driver_get_throttle(&left, &right);
            float target_left  = decision->left;
            float target_right = decision->right;
            if (!s_calibrating) {
                esc_trim_mix(decision->throttle, decision->rudder,
                             s_esc_trim, s_esc_trim_count,
                             &target_left, &target_right);
            }
            if (left != target_left || right != target_right) {
                esc_driver_set_throttle(target_left, target_right);
                changed = true;
            }
            s_manual_left = clampf(decision->left, 0.0f, 1.0f);
            s_manual_right = clampf(decision->right, 0.0f, 1.0f);
        }

        if (decision->winch_changed) {
            if (decision->winch != 0.0f && !s_rail_cut && !winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
            if (winch_driver_get_speed() != decision->winch) {
                winch_driver_set_speed(decision->winch);
                changed = true;
            }
        }

        if (decision->steer_changed) {
            bool needs_power = decision->steer_raw || decision->steer != 0.0f;
            if (needs_power && !s_rail_cut && !winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
            if (decision->steer_raw) {
                steer_driver_set_raw_us(decision->steer_raw_us);
                changed = true;
            } else if (steer_driver_get() != decision->steer) {
                steer_driver_set(decision->steer);
                changed = true;
            }
            s_manual_rudder = decision->steer_raw
                ? s_manual_rudder : clampf(decision->steer, -1.0f, 1.0f);
        }
    }

    status_commit_current(changed);
}

static void stability_sas_tick(bool steer_raw, int64_t now_us)
{
#if CONFIG_STABILITY_SAS_ENABLE
    /* Calibration owns the rudder (holds it at 0) and the ESCs while it runs;
     * SAS must not fight it. calibration_tick already ran this cycle. */
    if (s_calibrating) {
        return;
    }
    if (!s_stab_cfg_loaded) {
        s_stab_cfg_loaded = true;
        s_stab_cfg.r_max_dps = (float)CONFIG_STABILITY_SAS_RMAX_DPS;
        s_stab_cfg.k_r = parse_cfg_float(CONFIG_STABILITY_SAS_KR, 0.010f, 0.0f, 1.0f);
        s_stab_cfg.yaw_tau_s = parse_cfg_float(CONFIG_STABILITY_SAS_YAW_TAU_S, 0.15f, 0.01f, 5.0f);
        s_stab_cfg.out_cap = parse_cfg_float(CONFIG_STABILITY_SAS_OUT_CAP, 0.50f, 0.0f, 1.0f);
        stab_reset(&s_stab_state);
        s_stab_last_tick_us = 0;
    }

    /* Calibration escape hatch: never touch the rudder while the operator is
     * raw-pulse probing -- not even to centre it, that would fight the exact
     * thing they're doing. */
    if (steer_raw) {
        stab_reset(&s_stab_state);
        s_stab_last_seq = 0;
        s_stab_last_tick_us = 0;
        return;
    }

    /* Not armed / link dead / IMU unhealthy: step back AND actively centre.
     * Resetting internal state alone (the old behaviour) left SAS's last
     * rudder correction physically applied to the servo, frozen, with
     * nothing ever telling it otherwise -- worse than doing nothing.
     * Idempotent against control_apply_decision's own centring for the
     * ARMED/link cases; imu_icm_ok() is the one condition nothing else
     * checks, so this is the only place that catches it. */
    if (esc_driver_get_state() != ESC_STATE_ARMED ||
        !control_link_alive() || !imu_icm_ok()) {
        stab_reset(&s_stab_state);
        s_stab_last_seq = 0;
        s_stab_last_tick_us = 0;
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);
        }
        return;
    }

    FusionResult fusion = {0};
    fusion_get_result(&fusion);

    /* Bounded-age check on the real IMU timestamp -- catches a hung fusion
     * task even while the control link stays alive. sequence alone only
     * tells us "not a fresh sample this tick", not "how long has it been". */
    int64_t age_us = (fusion.captured_us != 0) ? (now_us - (int64_t)fusion.captured_us) : INT64_MAX;
    if (fusion.sequence == 0 || age_us > (int64_t)CONFIG_STABILITY_SAS_MAX_AGE_MS * 1000) {
        /* Throttled: this branch can otherwise log and write every 10ms tick
         * for as long as the fault persists -- up to 100/s inside the
         * highest-priority task in the system. */
        if ((++s_stab_log_div % STAB_LOG_DIVIDER) == 0) {
            if (fusion.sequence == 0) {
                ESP_LOGW(TAG, "CTRL_SAS,no_fusion_yet -> centring");
            } else {
                ESP_LOGW(TAG, "CTRL_SAS,stale,age_ms=%lld,max_ms=%d -> centring",
                         (long long)(age_us / 1000), (int)CONFIG_STABILITY_SAS_MAX_AGE_MS);
            }
        }
        stab_reset(&s_stab_state);
        s_stab_last_seq = 0;
        s_stab_last_tick_us = 0;
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);   /* fusion is gone -- centre, don't hold a stale command */
        }
        return;
    }
    if (fusion.sequence == s_stab_last_seq) {
        return;
    }
    s_stab_last_seq = fusion.sequence;

    float dt_s = (s_stab_last_tick_us == 0) ? 0.0f
               : clampf((float)(now_us - s_stab_last_tick_us) / 1000000.0f, 0.0f, 0.5f);
    s_stab_last_tick_us = now_us;

    float rudder = stab_rudder_update(&s_stab_state, &s_stab_cfg,
                                      dt_s, s_manual_rudder, fusion.yaw_rate);
    steer_driver_set(rudder);

    if ((++s_stab_log_div % STAB_LOG_DIVIDER) == 0) {
        ESP_LOGI(TAG, "CTRL_SAS,active,yaw=%.1f,rudder=%.2f,dt=%.3f,age_ms=%lld",
                 fusion.yaw_rate, rudder, dt_s, (long long)(age_us / 1000));
    }
#else
    (void)steer_raw;
    (void)now_us;
#endif
}

static void load_cal_cfg(void)
{
    s_cal_cfg.ki = parse_cfg_float(CONFIG_STABILITY_TRIMCAL_KI, 0.02f, 0.0f, 1.0f);
    s_cal_cfg.trim_clamp = parse_cfg_float(CONFIG_STABILITY_TRIMCAL_CLAMP, 0.30f, 0.05f, 1.0f);
    s_cal_cfg.accept_k = parse_cfg_float(CONFIG_STABILITY_TRIMCAL_ACCEPT_K, 3.0f, 0.5f, 10.0f);
    s_cal_cfg.excessive_yaw_dps = (float)CONFIG_STABILITY_TRIMCAL_MAX_YAW_DPS;
    s_cal_cfg.min_speed_mps = parse_cfg_float(CONFIG_STABILITY_TRIMCAL_MIN_SPEED_MPS, 0.3f, 0.0f, 10.0f);
    s_cal_cfg.window_ticks = CONFIG_STABILITY_TRIMCAL_WINDOW_TICKS;
    s_cal_cfg.noise_ticks = CONFIG_STABILITY_TRIMCAL_NOISE_TICKS;
    s_cal_cfg.settle_in_us = (int64_t)CONFIG_STABILITY_TRIMCAL_SETTLE_MS * 1000;
    s_cal_cfg.level_timeout_us = (int64_t)CONFIG_STABILITY_TRIMCAL_LEVEL_TIMEOUT_MS * 1000;
    /* Starting level list -- confirm on the water, config not commitment. */
    const float levels[] = {0.2f, 0.4f, 0.6f, 0.8f};
    s_cal_cfg.level_count = (uint8_t)(sizeof(levels) / sizeof(levels[0]));
    for (uint8_t i = 0; i < s_cal_cfg.level_count && i < ESC_TRIM_MAX_POINTS; ++i) {
        s_cal_cfg.levels[i] = levels[i];
    }
}

/* One control-task tick of ESC-trim auto-calibration. Runs AFTER
 * control_apply_decision (so calibration's own ESC writes are the last word of
 * the cycle) and BEFORE stability_sas_tick (which self-suppresses while
 * s_calibrating). Non-blocking: one esc_trim_cal_step per tick, no I/O except
 * the NVS write on a clean finish. Owns nothing until a start command lands and
 * the ARMED + link gates hold. See design doc sec 6/7/7a. */
static void calibration_tick(int64_t now_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool start_req = s_cal_start_pending; s_cal_start_pending = false;
    bool stop_req  = s_cal_stop_pending;  s_cal_stop_pending  = false;
    bool average   = s_cal_average;
    int64_t link_rx_us = s_last_control_rx_us;
    int64_t cal_rx_us  = s_cal_last_rx_us;
    portEXIT_CRITICAL(&s_arbiter_lock);

    if (!s_cal_cfg_loaded) {
        load_cal_cfg();
        s_cal_cfg_loaded = true;
    }

    /* Calibration's OWN link liveness, separate from the driving link. The
     * dashboard sends periodic CalibrateCommand keepalives while a sweep runs
     * (they refresh s_cal_last_rx_us but never the driving-link timer), so a
     * dashboard that crashes or goes out of range aborts the sweep within
     * CAL_LINK_TIMEOUT_US -- supervised, no dead-man hold required. Gating on
     * the DRIVING link here would be wrong: calibration deliberately lets it go
     * stale (sec 7a), so control_link_alive() is false for most of a real sweep. */
    bool cal_link_alive = (now_us - cal_rx_us) < CAL_LINK_TIMEOUT_US;

    if (start_req && !s_calibrating) {
        if (esc_driver_get_state() == ESC_STATE_ARMED && cal_link_alive && !s_rail_cut) {
            s_cal_cfg.average_into_existing = average;
            esc_trim_cal_start(&s_cal, &s_cal_cfg);
            s_cal_baseline_rx_us = link_rx_us;   /* any later manual cmd advances past this = abort */
            s_cal_out = (etc_out_t){0};
            s_calibrating = true;
            ESP_LOGI(TAG, "CAL,start,avg=%d,levels=%u", (int)average, (unsigned)s_cal_cfg.level_count);
        } else {
            ESP_LOGW(TAG, "CAL,start_rejected,armed=%d,rail_cut=%d",
                     (int)(esc_driver_get_state() == ESC_STATE_ARMED), (int)s_rail_cut);
        }
    }

    if (!s_calibrating) {
        return;
    }

    /* Fusion freshness: a hung fusion task leaves imu_icm_ok() true but the
     * yaw sample stale -- treat stale fusion as an IMU fault so the machine
     * aborts rather than integrating a frozen yaw. Same age bound as SAS. */
    FusionResult fusion = {0};
    fusion_get_result(&fusion);
    int64_t age_us = (fusion.captured_us != 0) ? (now_us - (int64_t)fusion.captured_us) : INT64_MAX;
    bool fusion_fresh = (fusion.sequence != 0) &&
                        (age_us <= (int64_t)CONFIG_STABILITY_SAS_MAX_AGE_MS * 1000);
    bool imu_ok = imu_icm_ok() && fusion_fresh;

    gps_fix_t gps = {0};
    (void)gps_driver_get_fix(&gps);
    bool making_way = gps.valid && gps.speed_mps >= s_cal_cfg.min_speed_mps;

    /* Abort (handed to the state machine's manual_override gate) on any of:
     *   - a real manual throttle/winch/steer command accepted since the sweep
     *     began (it advanced s_last_control_rx_us past the baseline),
     *   - an explicit Stop (CalibrateCommand{start:false} -> stop_req),
     *   - an explicit servo-rail PWR-OFF (s_rail_cut) -- a hard operator kill.
     * All three hand authority straight back to the pilot / safe state. */
    bool manual_override = link_rx_us > s_cal_baseline_rx_us;

    s_cal_out = esc_trim_cal_step(&s_cal, &s_cal_cfg, now_us,
                                  fusion.yaw_rate, gps.speed_mps,
                                  esc_driver_get_state() == ESC_STATE_ARMED,
                                  cal_link_alive,
                                  imu_ok,
                                  manual_override || stop_req || s_rail_cut);

    /* Telemetry: throttle to ~5 Hz so the 100 Hz control loop doesn't wake the
     * diagnostics publisher every tick (trim shifts L/R every tick during
     * SETTLE). Always emit the terminal DONE/ABORTED state so the UI sees it. */
    static uint8_t cal_status_div = 0;
    if ((++cal_status_div % 20) == 0 || s_cal_out.done || s_cal_out.aborted) {
        float yaw_avg = fusion.yaw_rate - s_cal.b0;
        cal_status_commit(&s_cal, yaw_avg, making_way);
    }

    if (s_cal_out.active) {
        /* Calibration's own ESC command is the last write of the cycle,
         * overriding control_apply_decision (which, with the driving link now
         * stale, has been zeroing the ESCs each tick). Rudder held centered. */
        esc_driver_set_throttle(s_cal_out.left_cmd, s_cal_out.right_cmd);
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);
        }
    }

    if (s_cal_out.done) {
        EscTrimNvsBlob blob;
        memset(&blob, 0, sizeof(blob));
        blob.magic_word = ESC_TRIM_NVS_MAGIC;
        blob.count = (s_cal.out_count > ESC_TRIM_MAX_POINTS) ? ESC_TRIM_MAX_POINTS : s_cal.out_count;
        for (uint8_t i = 0; i < blob.count; ++i) {
            blob.points[i] = s_cal.out_table[i];
        }
        motor_control_set_esc_trim(blob.points, blob.count);
        fs_save_esc_trim(&blob);
        s_calibrating = false;
        ESP_LOGI(TAG, "CAL,done,points=%u -> saved to NVS", (unsigned)blob.count);
    } else if (s_cal_out.aborted) {
        s_calibrating = false;
        ESP_LOGW(TAG, "CAL,aborted,reason=%s -- table discarded",
                 s_cal_out.reason ? s_cal_out.reason : "?");
    }
    /* On finish/abort we do NOT touch s_last_control_rx_us: it still holds the
     * pilot's last real command time, so control_apply_decision's existing
     * safe_stop path keeps the ESCs at 0 until a genuinely new manual command
     * arrives (or, if that command is <400ms old, resumes it -- live intent,
     * not a stale replay). No new failsafe logic (design doc sec 7a). */
}

static void run_control_cycle(bool scheduled, int64_t scheduled_us)
{
    static uint8_t heading_divider = 0;
    int64_t start_us = esp_timer_get_time();
    control_decision_t decision;

    runtime_metrics_cycle_begin(RUNTIME_TASK_CONTROL, (uint64_t)scheduled_us,
                                (uint64_t)start_us);
    portENTER_CRITICAL(&s_arbiter_lock);
    control_arbiter_decide(&s_arbiter, start_us, &decision);
    bool failsafe = s_last_control_rx_us == 0 ||
                    start_us - s_last_control_rx_us >= CONTROL_LINK_TIMEOUT_US;
    if (failsafe != s_control_failsafe) decision.drive_changed = true;
    decision.failsafe = failsafe;
    s_control_failsafe = failsafe;
    portEXIT_CRITICAL(&s_arbiter_lock);

    control_apply_decision(&decision);
    calibration_tick(start_us);
    stability_sas_tick(decision.steer_raw, start_us);
    status_commit_current(false);

    if (scheduled && ++heading_divider == HEADING_DIVIDER) {
        heading_divider = 0;
        heading_assist_dry_run_tick();
    }
    runtime_metrics_cycle_end(RUNTIME_TASK_CONTROL, (uint64_t)esp_timer_get_time());
}

static void task_control(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    TickType_t next = xTaskGetTickCount();
    int64_t scheduled_us = esp_timer_get_time();

    for (;;) {
        run_control_cycle(true, scheduled_us);
        next += period;
        scheduled_us += CONTROL_PERIOD_MS * 1000LL;

        for (;;) {
            TickType_t now = xTaskGetTickCount();
            if (next <= now) {
                if (next < now) {
                    runtime_metrics_count(RUNTIME_TASK_CONTROL,
                                          RUNTIME_EVENT_DEADLINE_MISS);
                    next = now;
                    scheduled_us = esp_timer_get_time();
                }
                break;
            }
            if (ulTaskNotifyTake(pdTRUE, next - now) == 0) break;
            run_control_cycle(false, esp_timer_get_time());
        }
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

void motor_control_set_esc_trim(const EscTrimPoint *pts, uint8_t count)
{
    s_esc_trim_count = 0;
    if (!pts) return;
    if (count > ESC_TRIM_MAX_POINTS) count = ESC_TRIM_MAX_POINTS;
    for (uint8_t i = 0; i < count; ++i) {
        if (!isfinite(pts[i].throttle_frac) || !isfinite(pts[i].trim_diff) ||
            pts[i].throttle_frac < 0.0f || pts[i].throttle_frac > 1.0f) {
            s_esc_trim_count = 0;
            return;
        }
        s_esc_trim[s_esc_trim_count++] = pts[i];
    }
}

void motor_control_disarm(void)
{
    s_arm_power_allowed = false;
    (void)esc_driver_disarm();
    (void)winch_driver_set_speed(0.0f);
    (void)winch_driver_set_power(false);
    (void)steer_driver_set(0.0f);
}

esp_err_t motor_control_init(void)
{
    control_arbiter_init(&s_arbiter);
    s_last_control_rx_us = 0;
    s_control_failsafe = true;
    s_rail_cut = false;
    s_status_generation = 0;
    atomic_store(&s_status_reader_task, (uintptr_t)NULL);
    s_arm_power_allowed = false;
    s_arm_request_queue = xQueueCreate(ARM_REQUEST_QUEUE_LENGTH,
                                       sizeof(arm_request_message_t));
    s_arm_action_queue = xQueueCreate(ARM_ACTION_QUEUE_LENGTH,
                                      sizeof(arm_action_t));
    if (!s_arm_request_queue || !s_arm_action_queue) return ESP_ERR_NO_MEM;
    status_commit_current(true);

    pipeline_register_motor_handler(motor_command_handler);
    pipeline_register_arm_handler(arm_command_handler);
    pipeline_register_winch_handler(winch_command_handler);
    pipeline_register_steer_handler(steer_command_handler);
    pipeline_register_servo_power_handler(servo_power_command_handler);
    pipeline_register_steer_raw_handler(steer_raw_command_handler);
    pipeline_register_calibrate_handler(calibrate_command_handler);

    esp_err_t task_error = runtime_task_create(RUNTIME_TASK_ARM_SEQUENCE,
                                               task_arm_sequence, NULL,
                                               &s_arm_sequence_task);
    if (task_error != ESP_OK) {
        motor_control_disarm();
        (void)winch_driver_set_power(false);
        ESP_LOGE(TAG, "critical task failed to start: ArmSeq");
        return task_error;
    }
    task_error = runtime_task_create(RUNTIME_TASK_CONTROL, task_control,
                                     NULL, &s_control_task);
    if (task_error != ESP_OK) {
        motor_control_disarm();
        (void)winch_driver_set_power(false);
        ESP_LOGE(TAG, "critical task failed to start: Control");
        return task_error;
    }

    ESP_LOGI(TAG, "Motor control initialized (ControlTask 10 ms, persistent ArmSeq)");
    return ESP_OK;
}
