#include "motor_control.h"
#include "arm_sequence.h"
#include "esc_trim.h"
#include "esc_trim_cal.h"
#include "bench_run.h"
#include "trim_learn.h"
#include "trim_assist.h"
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
/* When the fast control values last went out; 0 = never. Guarded by
 * s_status_lock, same as the buffers. */
static int64_t s_status_continuous_us = 0;
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
/* Bench throttle-mismatch run. s_bench_active mirrors s_calibrating: it tells
 * the rest of the control cycle that something else owns the ESCs this tick. */
static bool s_bench_active = false;
/* On-the-fly trim learner. s_trim_moved lets the drive path know c changed even
 * when no new pilot command arrived -- the ESC write is otherwise gated on
 * drive_changed, so a silently-adapting c would never reach the motors. */
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
static trim_learn_t s_trim_learn;
static trim_learn_cfg_t s_trim_learn_cfg;
static uint64_t s_trim_last_capture_us = 0;
#endif
static bool s_trim_moved = false;
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
/* The fast P correction, added on top of the learned c and never stored.
 * s_p_assist_on is a RUNTIME switch (default off) so the A and B arms of the
 * experiment run the same firmware -- a rebuild between arms would let a
 * compiler or config difference masquerade as a result. */
static trim_assist_t s_trim_assist;
static trim_assist_cfg_t s_trim_assist_cfg;
static bool s_p_assist_on = false;
static float s_p_correction = 0.0f;
/* Set by the RX task, consumed by the control task. The control task is the
 * only writer of s_p_assist_on and of everything downstream of it. */
static bool s_p_assist_req = false;
static bool s_p_assist_req_pending = false;
/* Set by the control task on a P transition, printed by the core-1 logger. */
static bool s_p_log_pending = false;
#endif
/* Read by the drive path in BOTH builds; always false when P is compiled out. */
static bool s_p_moved = false;
#if CONFIG_STABILITY_SAS_ENABLE
/* Assisted Steering: the rudder yaw-rate loop. RUNTIME switch, OFF at every
 * boot -- the default mode is Raw Manual and nothing persists this. Kept
 * strictly separate from s_p_assist_on, which is the MOTOR assist. */
static bool s_assist_rudder_on = false;
static bool s_assist_rudder_req = false;
static bool s_assist_rudder_req_pending = false;
/* Last values the loop commanded, for telemetry. COMMANDED, not measured --
 * this servo has no position feedback. */
static float s_assist_rudder_cmd = 0.0f;
static float s_assist_target_dps = 0.0f;
static float s_assist_yaw_filt = 0.0f;
static bool s_assist_saturated = false;
#endif
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
/* Why the learner is not moving, or NULL when it is. Written on the control
 * task, read by the core-1 logger: a torn read costs one wrong log line and
 * nothing else, which is far cheaper than a lock on the 10 ms path. */
static const char *s_trim_why = "starting up";
static float s_trim_thr = 0.0f;
#endif
static bench_t s_bench;
/* Run profile. Zero-initialising this would make every run finish instantly,
 * so it is spelled out here. 0.5 s still + 3.0 s driving + 1.0 s coasting. */
/* Run length is set by the POOL and by COMPARABILITY, never by the learner.
 *
 * 3 s, the same as every run recorded before 2026-08-29, so new files can be
 * held against the ~200 already on file. That comparison is the whole value of
 * the BASE test and must not be traded away lightly.
 *
 * A 10 s BASE run was tried so the learner could converge inside one run. In a
 * 1.2 m pool that backfires: the hull reaches the wall at a median of 4 s and
 * bounces for the rest, so 33% of all recorded run time was post-contact,
 * against 19% for the old 3 s runs. The trim was not less stable -- the
 * recording just contained twice as much wall.
 *
 * The learner does not need a long run anyway: c carries over between runs, so
 * three short runs integrate exactly like one long one, and short runs are
 * what the fixed-trim results were validated under.
 *
 *   3 s -> 12% post-contact   4 s -> 16%   5 s -> 19%   10 s -> 33%
 *
 * And the long run did not even help. The learner keeps MOVING at a steady
 * ~50% of its maximum rate throughout, but once the hull is bouncing the
 * disturbance is symmetric, so it steps up as often as down and cancels:
 *
 *   first 3 s: 0.0034 of c per second  |  next 7 s: 0.0010 per second
 *
 * Three quarters of the progress happens in the first three seconds.
 *
 * A LEFT/RIGHT run is deliberately turning throughout and only has to show
 * which way, so it stays short. */
#define BENCH_RUN_US_SPLIT 3000000
#define BENCH_RUN_US_BASE  3000000

static bench_cfg_t s_bench_cfg = {
    .baseline_us = 500000,
    .run_us      = BENCH_RUN_US_SPLIT,      /* set per kind in bench_tick */
    .coast_us    = 1000000,
    .max_yaw_dps = 200.0f,   /* safety only -- a held boat never gets near this */
};
static bool s_bench_start_pending = false;
/* > 0 asks the learner to restart from that c. Consumed by the single run it
 * arrived with -- never latched, so an ordinary run that follows cannot
 * inherit it. */
static float s_bench_req_reset_c = 0.0f;
static uint32_t s_bench_req_kind = 0;
static float s_bench_req_base = 0.0f, s_bench_req_delta = 0.0f;
static boat_BenchStatus s_bench_status;
static uint32_t s_bench_status_generation = 0;
static uint32_t s_bench_file_index = 0;
/* Set by the control task when a run finishes; the SD write itself is done
 * by the core-1 diagnostics task via motor_control_bench_flush(). */
static volatile bool s_bench_save_pending = false;

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
static bool s_cal_ready = true;   /* start-latch: a start fires only on a clean edge (see calibration_tick) */
/* True when the boat's current arm was a FORCE arm ("bench / no GPS"). Set on
 * every arm from decision->force_arm, so while armed it always reflects that
 * arm (a normal re-arm resets it). Calibration uses it to decide whether to
 * GPS-gate: force-armed = operator said "no GPS" = skip the making-way gate;
 * normal-armed = GPS present = keep it. */
static bool s_force_armed = false;
static float s_cal_min_speed_cfg = 0.3f;   /* GPS making-way threshold (Kconfig); per-run value derives from this + force-arm */
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

/* DISCRETE state: arm, throttle, winch, rail, and the two assist mode flags.
 * These change rarely and matter immediately, so any difference publishes at
 * once -- an operator must see an arm or mode change without waiting. */
static bool motor_status_discrete_equal(const boat_MotorStatus *a,
                                        const boat_MotorStatus *b)
{
    return a->state == b->state &&
           a->left_throttle == b->left_throttle &&
           a->right_throttle == b->right_throttle &&
           a->winch_speed == b->winch_speed &&
           a->servo_power == b->servo_power &&
           a->assist_rudder == b->assist_rudder &&
           a->assist_motor_p == b->assist_motor_p;
}

/* CONTINUOUS control values, which move on essentially every 100 Hz cycle once
 * the assisted loop is live -- the loop re-computes on every fresh fusion
 * sample, so ~50 Hz, and status_commit_current runs at the 100 Hz control
 * rate. Treating a change in these as "publish now" put MotorStatus on the air
 * at the fusion rate over a link carrying ~20 Hz of telemetry;
 * it congested, MotorStatus was what got dropped, and the rudder-test
 * staleness gate then killed both assisted runs on 2026-09-02 while the boat
 * was in fact driving perfectly well.
 *
 * They still need better than the 1 Hz periodic -- the assisted CSV records
 * them per telemetry frame -- so they publish on their own bounded schedule
 * instead. */
static bool motor_status_continuous_equal(const boat_MotorStatus *a,
                                          const boat_MotorStatus *b)
{
    return a->rudder_cmd == b->rudder_cmd &&
           a->rudder_pulse_us == b->rudder_pulse_us &&
           a->rudder_saturated == b->rudder_saturated &&
           a->yaw_target_dps == b->yaw_target_dps &&
           a->yaw_filt_dps == b->yaw_filt_dps;
}

/* 10 Hz. Comfortably finer than the ~20 Hz telemetry the CSV samples against,
 * and a tenth of the flood. */
#define MOTOR_STATUS_CONTINUOUS_MIN_INTERVAL_US 100000

static void status_commit_current(bool force)
{
    boat_MotorStatus status = boat_MotorStatus_init_zero;
    status.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&status.left_throttle, &status.right_throttle);
    status.winch_speed = winch_driver_get_speed();
    status.servo_power = winch_driver_get_power();
    /* COMMANDED rudder, never measured -- this servo has no position feedback.
     * Read from the driver so it is what actually went out, whichever mode
     * put it there. */
    status.rudder_cmd = steer_driver_get();
    status.rudder_pulse_us = steer_driver_get_pulse_us();
#if CONFIG_STABILITY_SAS_ENABLE
    status.assist_rudder = s_assist_rudder_on;
    status.rudder_saturated = s_assist_saturated;
    status.yaw_target_dps = s_assist_target_dps;
    status.yaw_filt_dps = s_assist_yaw_filt;
#endif
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    status.assist_motor_p = s_p_assist_on;
#endif

    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_status_generation;
    const boat_MotorStatus *prev = &s_status_buffers[generation & 1U];
    if (!force) {
        const bool discrete_same = motor_status_discrete_equal(&status, prev);
        const bool continuous_same = motor_status_continuous_equal(&status, prev);
        if (discrete_same && continuous_same) {
            portEXIT_CRITICAL(&s_status_lock);
            return;                       /* nothing moved at all */
        }
        if (discrete_same) {
            /* Only the fast control values moved. Rate-limit those. */
            if (s_status_continuous_us != 0 &&
                (esp_timer_get_time() - s_status_continuous_us)
                    < MOTOR_STATUS_CONTINUOUS_MIN_INTERVAL_US) {
                portEXIT_CRITICAL(&s_status_lock);
                return;
            }
        }
        /* A discrete change is never withheld -- an operator must see an arm
         * or mode change at once. Either way we are about to publish, so the
         * budget is measured from the last thing that actually went on the
         * air. Stamping it only on the continuous path (the first version of
         * this) left a stale mark behind every discrete publish, so the next
         * continuous change slipped out immediately and the comment claiming
         * otherwise was simply false. */
        s_status_continuous_us = esp_timer_get_time();
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

static void bench_status_commit(void)
{
    boat_BenchStatus st = boat_BenchStatus_init_zero;
    st.state      = (uint32_t)s_bench.state;
    st.kind       = (uint32_t)s_bench.kind;
    st.base       = s_bench.base;
    st.samples    = s_bench.count;
    st.file_index = s_bench_file_index;
    st.elapsed_s  = s_bench.elapsed_s;
    st.learn_c    = motor_control_trimlearn_c();
    st.p_on       = motor_control_p_assist_on();
    portENTER_CRITICAL(&s_status_lock);
    s_bench_status = st;
    s_bench_status_generation++;
    portEXIT_CRITICAL(&s_status_lock);
}

uint32_t motor_control_get_bench_status(boat_BenchStatus *out)
{
    if (!out) return 0;
    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_bench_status_generation;
    *out = s_bench_status;
    portEXIT_CRITICAL(&s_status_lock);
    return generation;
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
/* Runs in the pipeline RX task. Sets a pending flag only -- the control task
 * owns every actuator write, same rule as calibrate_command_handler. */
/* RX task. Records a REQUEST and nothing else.
 *
 * Every byte of P state -- the filter, the correction, the enable flag -- is
 * owned by the control task. Touching it from here would race the 100 Hz loop
 * mid-mix and could leave the ESCs holding a correction the gate has already
 * revoked. */
static void assist_command_handler(bool p_on, bool rudder_assist)
{
    /* MUTUALLY EXCLUSIVE for this experiment. The motor P assist perturbs
     * differential thrust and the rudder loop perturbs the rudder; running
     * both would leave any result unattributable to either. Resolved HERE, on
     * the request, so the boat can never hold both -- and resolved in favour
     * of OFF: a command asking for both is a mistake, and the safe reading of
     * a mistake is neither. */
    if (p_on && rudder_assist) {
        p_on = false;
        rudder_assist = false;
    }
    portENTER_CRITICAL(&s_arbiter_lock);
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    s_p_assist_req = p_on;
    s_p_assist_req_pending = true;
#endif
#if CONFIG_STABILITY_SAS_ENABLE
    s_assist_rudder_req = rudder_assist;
    s_assist_rudder_req_pending = true;
#endif
    portEXIT_CRITICAL(&s_arbiter_lock);
    (void)p_on; (void)rudder_assist;
}

static void bench_command_handler(uint32_t kind, float base, float delta,
                                  float reset_c)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    s_bench_start_pending = true;
    s_bench_req_kind = kind;
    s_bench_req_base = base;
    s_bench_req_delta = delta;
    s_bench_req_reset_c = reset_c;
    portEXIT_CRITICAL(&s_arbiter_lock);
}

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

/* One learner step per FUSION SAMPLE (50 Hz), not per control tick (100 Hz):
 * acting every tick would integrate each gyro sample twice. */
static void trim_learn_tick(const control_decision_t *decision)
{
    s_trim_moved = false;
    s_p_moved = false;
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    /* Apply a pending enable/disable HERE, on the control task, before any
     * gate is evaluated -- so an OFF takes effect on this very tick and the
     * clean value is remixed in the same cycle. */
    portENTER_CRITICAL(&s_arbiter_lock);
    bool req_pending = s_p_assist_req_pending;
    bool req = s_p_assist_req;
    s_p_assist_req_pending = false;
    portEXIT_CRITICAL(&s_arbiter_lock);
    if (req_pending && req != s_p_assist_on) {
        s_p_assist_on = req;
        if (!req) {                     /* OFF returns to the exact pre-P path */
            trim_assist_reset(&s_trim_assist);
            if (s_p_correction != 0.0f) s_p_moved = true;
            s_p_correction = 0.0f;
        }
        /* NOT logged here: one ESP_LOG line is a ~9 ms synchronous UART write
         * and this is the 10 ms control task. Hand the transition to the
         * core-1 logger instead. */
        s_p_log_pending = true;
    }
    /* A BASE run commands both motors equal, so it IS a straight-line demand
     * and any yaw is exactly the error the learner exists to remove: it stays
     * LIVE, and bench_tick reads its c every tick, so the correction reaches
     * the jets during the run and the file records it happening.
     *
     * A LEFT/RIGHT run is the opposite -- the boat is deliberately turning,
     * and learning from a commanded turn would poison c with the very
     * perturbation the test applies. Calibration owns the motors outright. */
    const bool bench_learning = s_bench_active &&
                                s_bench.kind == BENCH_KIND_BASE &&
                                s_bench.state == BENCH_RUN;
    if (s_calibrating || (s_bench_active && !bench_learning)) {
        s_trim_why = "bench/cal owns the motors";
        /* Drop P here too. This return is BEFORE the P block, so without it a
         * LEFT/RIGHT run or a calibration would keep applying whatever
         * correction was live when it started -- fighting the very
         * perturbation the test is applying. */
        if (s_p_correction != 0.0f) s_p_moved = true;   /* never overwrite:
                                 * a pending OFF this same tick already set it */
        trim_assist_reset(&s_trim_assist);
        s_p_correction = 0.0f;
        /* Forget when the last sample was. Otherwise the first sample after a
         * 5 s bench run carries dt = 5 s, and the step is proportional to dt.
         * trim_learn_update bounds this too; dropping it here is what makes
         * the gap explicit rather than merely survivable. */
        s_trim_last_capture_us = 0;
        return;
    }

    FusionResult f = {0};
    fusion_get_result(&f);
    int64_t age_us = (f.captured_us != 0)
                   ? (esp_timer_get_time() - (int64_t)f.captured_us) : INT64_MAX;
    bool healthy = imu_icm_ok() && f.sequence != 0 &&
                   age_us <= (int64_t)CONFIG_STABILITY_SAS_MAX_AGE_MS * 1000;

    float dt_s = 0.0f;
    if (s_trim_last_capture_us != 0 && f.captured_us > s_trim_last_capture_us) {
        dt_s = (float)(f.captured_us - s_trim_last_capture_us) / 1000000.0f;
    }
    /* Any rudder demand means the boat is MEANT to be turning, and this boat
     * can be turned three different ways. All three must count, because the
     * learner cannot tell a commanded turn from a motor imbalance -- it would
     * happily "correct" the pilot's own steering into c and keep it there.
     *
     *   decision->rudder    differential thrust, folded into the drive command
     *   decision->steer     the physical rudder servos, a separate SteerCommand
     *   decision->steer_raw a raw microsecond pulse (bench/servo probing)
     *
     * steer is its OWN arbiter channel, not derived from rudder: a boat
     * turning purely on its rudder servos has decision->rudder == 0, so the
     * old rudder-only test saw a straight line and learned from a turn.
     *
     * steer_raw is a bare flag, deliberately: control_arbiter_submit_steer_raw
     * forces value to 0.0f and only carries the pulse, so there is no
     * normalized position to threshold. Any raw pulse at all means someone is
     * driving the servo by hand. Same call SAS makes (stability_sas_tick bails
     * outright on steer_raw rather than fighting the operator).
     *
     * Neither channel goes stale -- update_continuous() has no timeout, so the
     * last SteerCommand holds until the next one. That is what makes recovery
     * work: a SteerCommand of 0.0 both zeroes steer and clears the raw flag,
     * so returning the rudder to neutral resumes learning by itself. */
    bool steering = fabsf(decision->rudder) > 0.02f ||
                    fabsf(decision->steer) > 0.02f ||
                    decision->steer_raw;

    /* A held-up throttle slider on a DISARMED boat is not a measurement: the
     * jets are dead, the boat is not moving, and every degree the gyro reads is
     * noise or someone carrying it. Integrating that is a random walk on c.
     * Reporting throttle 0 freezes the integrator while still letting the yaw
     * filter track, so it is already settled when the motors do come on. */
    bool driving = (esc_driver_get_state() == ESC_STATE_ARMED);
    float thr = driving ? decision->throttle : 0.0f;
    if (bench_learning) {
        /* The bench is driving, not the pilot -- whose throttle the tool zeroes
         * at the button press. Reading that zero would freeze the learner on
         * the low-throttle gate through the entire run. */
        thr = driving ? s_bench.base : 0.0f;
        steering = false;                   /* BASE commands both jets equal */
    }

    /* RAW gyro, deliberately. Subtracting the run's motors-off baseline was
     * tried and made it measurably WORSE (dataout/autotrim2, 39 runs). The
     * baseline does not predict what the gyro does once the motors run:
     * corr(raw, baseline) = -0.16 over one session and +0.05 over the next,
     * i.e. nothing. Subtracting an uncorrelated offset removes no bias and
     * adds a second noise source -- and this one is a step held for a whole
     * run rather than white noise, the worst possible input to an integrator:
     *
     *     sd(raw) 0.40  ->  sd(raw - baseline) 0.92
     *     c wandered 0.129..0.206  ->  0.114..0.277, repeatedly hitting the
     *     per-run maximum step of 0.050, twice reaching the clamp.
     *
     * What DOES track c is the raw yaw: slope +11 to +20 deg/s per unit c,
     * r = +0.53..+0.83 across five level-fits. The plant is real and raw is
     * the signal. Do not reintroduce the subtraction.
     *
     * bench_baseline_yaw() is still logged as a diagnostic -- knowing it is
     * uncorrelated is worth seeing. */
    const float yaw = f.yaw_rate;

    uint32_t before = s_trim_learn.last_seq;
    bool fresh_sample = (f.sequence != before);
    s_trim_moved = trim_learn_update(&s_trim_learn, &s_trim_learn_cfg,
                                     f.sequence, dt_s, yaw,
                                     thr, steering, healthy);
    if (s_trim_learn.last_seq != before) s_trim_last_capture_us = f.captured_us;

    /* --- the fast P correction, on the SAME sample the learner just used ---
     *
     * Every gate the learner has, plus the switch and the impact threshold.
     * A false gate RESETS rather than decays: the correction must not survive
     * a pause, so that OFF and "gated off" are the same state, and so the A
     * arm of the experiment is bit-identical to the pre-P firmware. */
    const float p_prev = s_p_correction;
    bool p_gate = s_p_assist_on && healthy && driving && !steering &&
                  (thr >= s_trim_learn_cfg.min_throttle) &&
                  (fabsf(yaw) <= TRIM_LEARN_REJECT_DPS);
    if (fresh_sample) {
        s_p_correction = trim_assist_update(&s_trim_assist, &s_trim_assist_cfg,
                                            dt_s, yaw, p_gate);
    } else if (!p_gate) {
        trim_assist_reset(&s_trim_assist);
        s_p_correction = 0.0f;
    }
    /* A changed correction must reach the ESCs even with no new pilot command,
     * or P would be visible in the log and absent from the motors. */
    if (s_p_correction != p_prev) s_p_moved = true;

    s_trim_why = !healthy   ? "gyro stale"
               : steering   ? "steering"
               : !driving   ? "disarmed"
               : s_trim_learn.faulted ? "FAULTED at the clamp"
               : (thr < s_trim_learn_cfg.min_throttle) ? "throttle too low"
               : NULL;                      /* NULL == actually learning */
    s_trim_thr = thr;
#else
    (void)decision;
#endif
}

#if CONFIG_STABILITY_TRIMLEARN_ENABLE
/* The value the mixer actually uses: learned c plus the temporary P
 * correction, inside the learner's own bounds. The learned c is untouched --
 * only this sum reaches the mixer, and only the I learner may move c itself.
 * Both call sites are inside the same #if, so this is too. */
static float effective_trim_c(float learned_c)
{
    return trim_assist_effective_c(learned_c, s_p_correction,
                                   s_trim_learn_cfg.c_min,
                                   s_trim_learn_cfg.c_max);
}
#endif

static void control_apply_decision(control_decision_t *decision)
{
    trim_learn_tick(decision);
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
        if (winch_driver_get_power() && !s_calibrating && !s_bench_active) {
            winch_driver_set_power(false);
            changed = true;
        }
        s_manual_left = 0.0f;
        s_manual_right = 0.0f;
        s_manual_rudder = 0.0f;
        heading_assist_reset();
        if (!control_link_alive() && changed && !s_calibrating && !s_bench_active) {
            ESP_LOGW(TAG, "Control link lost — throttle 0, winch 0, rudders centred, servo rail cut");
        }
    }

    if (decision->disarm && !internal_disarm) {
        submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
    }

    if (!safe_stop) {
        if (decision->arm || decision->force_arm) {
            s_arm_power_allowed = true;
            s_force_armed = decision->force_arm;   /* bench/no-GPS arm vs normal arm */
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

        if (decision->drive_changed || s_trim_moved || s_p_moved) {
            float left;
            float right;
            esc_driver_get_throttle(&left, &right);
            float target_left  = decision->left;
            float target_right = decision->right;
            if (!s_calibrating && !s_bench_active) {
                const EscTrimPoint *pts = s_esc_trim;
                uint8_t count = s_esc_trim_count;
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
                /* The learned trim scales with throttle, so hand the mixer a
                 * single point holding this cycle's value. Mixer units are
                 * twice the split (left -= t/2, right += t/2). */
                EscTrimPoint learned = {
                    .throttle_frac = 1.0f,
                    .trim_diff = 2.0f * effective_trim_c(s_trim_learn.c)
                                     * clampf(decision->throttle, 0.0f, 1.0f),
                };
                pts = &learned;
                count = 1;
#endif
                esc_trim_mix(decision->throttle, decision->rudder,
                             pts, count, &target_left, &target_right);
            }
            if (left != target_left || right != target_right) {
                esc_driver_set_throttle(target_left, target_right);
                changed = true;
            }
            if (decision->drive_changed) {   /* only a real command sets these */
                s_manual_left = clampf(decision->left, 0.0f, 1.0f);
                s_manual_right = clampf(decision->right, 0.0f, 1.0f);
            }
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
    if (s_calibrating || s_bench_active) {
        return;
    }
    if (!s_stab_cfg_loaded) {
        s_stab_cfg_loaded = true;
        s_stab_cfg.r_max_dps = (float)CONFIG_STABILITY_SAS_RMAX_DPS;
        s_stab_cfg.k_p = parse_cfg_float(CONFIG_STABILITY_SAS_KR, 0.05f, 0.0f, 1.0f);
        s_stab_cfg.ff_left = parse_cfg_float(CONFIG_STABILITY_SAS_FF_LEFT, 0.18f, 0.0f, 2.0f);
        s_stab_cfg.ff_right = parse_cfg_float(CONFIG_STABILITY_SAS_FF_RIGHT, 0.31f, 0.0f, 2.0f);
        s_stab_cfg.yaw_tau_s = parse_cfg_float(CONFIG_STABILITY_SAS_YAW_TAU_S, 0.25f, 0.01f, 5.0f);
        s_stab_cfg.out_cap = parse_cfg_float(CONFIG_STABILITY_SAS_OUT_CAP, 0.80f, 0.0f, 1.0f);
        s_stab_cfg.slew_per_s = parse_cfg_float(CONFIG_STABILITY_SAS_SLEW_PER_S, 2.0f, 0.0f, 20.0f);
        stab_reset(&s_stab_state);
        s_stab_last_tick_us = 0;
        ESP_LOGI(TAG, "SAS built in (Assisted Steering default OFF): r_max=%.1f "
                      "kp=%.3f ff_l=%.3f ff_r=%.3f tau=%.2f cap=%.2f slew=%.1f",
                 (double)s_stab_cfg.r_max_dps, (double)s_stab_cfg.k_p,
                 (double)s_stab_cfg.ff_left, (double)s_stab_cfg.ff_right,
                 (double)s_stab_cfg.yaw_tau_s, (double)s_stab_cfg.out_cap,
                 (double)s_stab_cfg.slew_per_s);
    }

    /* Apply a pending runtime enable/disable, on THIS task, before any gate is
     * evaluated -- so an OFF takes effect on this very tick. */
    portENTER_CRITICAL(&s_arbiter_lock);
    const bool a_pending = s_assist_rudder_req_pending;
    const bool a_req = s_assist_rudder_req;
    s_assist_rudder_req_pending = false;
    portEXIT_CRITICAL(&s_arbiter_lock);
    if (a_pending && a_req != s_assist_rudder_on) {
        s_assist_rudder_on = a_req;
        stab_reset(&s_stab_state);
        s_stab_last_seq = 0;
        s_stab_last_tick_us = 0;
        s_assist_rudder_cmd = 0.0f;
        s_assist_target_dps = 0.0f;
        s_assist_yaw_filt = 0.0f;
        s_assist_saturated = false;
        /* Leaving Assisted returns the rudder to exactly where Raw Manual says
         * it should be, rather than wherever the loop had walked it. */
        if (!s_assist_rudder_on) {
            steer_driver_set(s_manual_rudder);
        }
    }

    /* Raw Manual is the default mode and the boot mode. With the loop off,
     * control_apply_decision owns the rudder exactly as it always has. */
    if (!s_assist_rudder_on) {
        return;
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

    stab_debug_t dbg;
    float rudder = stab_rudder_update(&s_stab_state, &s_stab_cfg,
                                      dt_s, s_manual_rudder, fusion.yaw_rate,
                                      &dbg);
    steer_driver_set(rudder);
    s_assist_rudder_cmd = rudder;
    s_assist_target_dps = dbg.target_dps;
    s_assist_yaw_filt = dbg.yaw_filt;
    s_assist_saturated = dbg.saturated;

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
    s_cal_min_speed_cfg = parse_cfg_float(CONFIG_STABILITY_TRIMCAL_MIN_SPEED_MPS, 0.3f, 0.0f, 10.0f);
    s_cal_cfg.min_speed_mps = s_cal_min_speed_cfg;   /* per-run value is set at start from force-arm */
    s_cal_cfg.window_ticks = CONFIG_STABILITY_TRIMCAL_WINDOW_TICKS;
    s_cal_cfg.noise_ticks = CONFIG_STABILITY_TRIMCAL_NOISE_TICKS;
    s_cal_cfg.settle_in_us = (int64_t)CONFIG_STABILITY_TRIMCAL_SETTLE_MS * 1000;
    s_cal_cfg.level_timeout_us = (int64_t)CONFIG_STABILITY_TRIMCAL_LEVEL_TIMEOUT_MS * 1000;
    /* Starting level list -- confirm on the water, config not commitment. */
    const float levels[] = {0.05f, 0.10f, 0.20f, 0.30f};
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
/* Write the buffered run to the card. 8.3 filenames only
 * (CONFIG_FATFS_LFN_NONE=y), so the name is "T<pct>_<K>_<NN>.CSV" -- exactly 8
 * characters before the dot. The WHOLE run is written (baseline, drive and
 * coast) so which part is useful can be decided later, off the file. */
static void bench_write_csv(void)
{
    const char kc = (s_bench.kind == BENCH_KIND_LEFT) ? 'L'
                  : (s_bench.kind == BENCH_KIND_RIGHT) ? 'R' : 'B';
    unsigned pct = (unsigned)(s_bench.base * 100.0f + 0.5f);
    if (pct > 99u) pct = 99u;

    char name[16];
    unsigned idx = 0;
    for (unsigned i = 1; i <= 99u; ++i) {
        uint8_t probe = 0;
        size_t got = 0;
        snprintf(name, sizeof(name), "T%02u_%c_%02u.CSV", pct, kc, i);
        if (fs_sdcard_read(name, &probe, 1, &got) == ESP_ERR_NOT_FOUND) {
            idx = i;                        /* first free slot: never overwrite */
            break;
        }
    }
    if (idx == 0) {
        ESP_LOGE(TAG, "BENCH,save_failed,no free filename");
        s_bench.state = BENCH_FAILED;
        return;
    }
    s_bench_file_index = idx;

    /* Each flush is a whole fopen/fwrite/fflush/fclose on FAT, and it takes
     * the SD lock that the C6 radio shares (one mutex for both slots, in
     * ESP-IDF's own driver). So the count of flushes -- not the byte total --
     * is what decides how long the link goes quiet after a run. A 10 s BASE
     * run is ~48 KB; at 1 KB a flush that is ~48 of them, roughly a second of
     * silence. 4 KB brings it to ~12, fewer than the old short runs took. */
    static char chunk[8192];
    int n = snprintf(chunk, sizeof(chunk), "t_s,phase,yaw_dps,left,right,c,p_on,p_yaw,c_learn,p_corr,split,at_cap\n");
    if (n <= 0 || fs_sdcard_write(name, chunk, (size_t)n) != ESP_OK) {
        ESP_LOGE(TAG, "BENCH,save_failed,%s", name);
        s_bench.state = BENCH_FAILED;
        return;
    }

    const float base_s = (float)s_bench_cfg.baseline_us / 1000000.0f;
    const float run_s  = base_s + (float)s_bench_cfg.run_us / 1000000.0f;
    n = 0;
    for (uint16_t i = 0; i < s_bench.count; ++i) {
        if ((size_t)n > sizeof(chunk) - 96u) {      /* flush before it can truncate */
            (void)fs_sdcard_append(name, chunk, (size_t)n);
            n = 0;
        }
        const bench_sample_t *smp = &s_bench.samples[i];
        const char *ph = (smp->t_s < base_s) ? "baseline"
                       : ((smp->t_s < run_s) ? "run" : "coast");
        int w = snprintf(chunk + n, sizeof(chunk) - (size_t)n,
                         "%.3f,%s,%.3f,%.3f,%.3f,%.4f,"
                         "%u,%.3f,%.4f,%.4f,%.4f,%u\n",
                         (double)smp->t_s, ph, (double)smp->yaw_rate_dps,
                         (double)smp->left, (double)smp->right, (double)smp->c,
                         (unsigned)((smp->p_flags & BENCH_P_ON) ? 1u : 0u),
                         (double)smp->p_yaw,
                         (double)smp->c_learn,
                         (double)smp->p_corr,
                         (double)(0.5f * (smp->right - smp->left)),
                         (unsigned)((smp->p_flags & BENCH_P_AT_CAP) ? 1u : 0u));
        if (w < 0 || (size_t)w >= sizeof(chunk) - (size_t)n) break;
        n += w;
    }
    if (n > 0) (void)fs_sdcard_append(name, chunk, (size_t)n);

    ESP_LOGI(TAG, "BENCH,saved,%s,samples=%u%s", name, (unsigned)s_bench.count,
             s_bench.overflow ? ",OVERFLOW" : "");
}

/* One bench tick. Mirrors calibration_tick: the control task owns the ESCs,
 * nothing blocks, and the run is driven by the clock -- a telemetry gap can
 * never cut the recording short. */
/* Runs on the core-1 diagnostics task, never on the control loop. Safe because
 * the control task does not touch the sample buffer once a run has finished,
 * and a new run is refused while a save is still pending. */
/* What the BOAT thinks, not what the tool asked for. The A/B is void if the
 * two ever disagree, so the operator must be able to see the boat's own
 * answer. */
bool motor_control_p_assist_on(void)
{
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    return s_p_assist_on;
#else
    return false;
#endif
}

float motor_control_trimlearn_c(void)
{
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    return s_trim_learn.c;
#else
    return 0.0f;
#endif
}

void motor_control_trimlearn_log(void)
{
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    /* Core-1 diagnostics only. One ESP_LOG line is ~100 bytes, and the UART
     * writes it synchronously -- ~9 ms at 115200, which alone would blow the
     * control task's 10 ms deadline. */
    static int64_t last_us = 0;
    static float last_c = -1.0f;
    static const char *last_why = "";
    int64_t now = esp_timer_get_time();
    bool moved = fabsf(s_trim_learn.c - last_c) >= 0.005f;
    bool changed_why = (s_trim_why != last_why);
    if (!moved && !changed_why && now - last_us < 5000000) return;
    last_us = now;
    last_c = s_trim_learn.c;
    last_why = s_trim_why;

    if (s_p_log_pending) {
        s_p_log_pending = false;
        ESP_LOGW(TAG, "P-ASSIST %s (kp=%.3f tau=%.2f cap=%.3f)",
                 s_p_assist_on ? "ON" : "OFF",
                 (double)s_trim_assist_cfg.kp, (double)s_trim_assist_cfg.tau_s,
                 (double)s_trim_assist_cfg.cap);
    }
    if (s_trim_why) {
        ESP_LOGI(TAG, "TRIMLEARN,hold,c=%.3f,yaw=%+.2f,thr=%.2f,why=%s",
                 (double)s_trim_learn.c, (double)s_trim_learn.yaw_filt,
                 (double)s_trim_thr, s_trim_why);
    } else {
        ESP_LOGI(TAG, "TRIMLEARN,learn,c=%.3f,yaw=%+.2f,thr=%.2f,split=%.1f%%"
                      ",zero=%+.2f",
                 (double)s_trim_learn.c, (double)s_trim_learn.yaw_filt,
                 (double)s_trim_thr,
                 (double)(trim_learn_split(&s_trim_learn, s_trim_thr) * 100.0f),
                 (double)bench_baseline_yaw(&s_bench));
    }
#endif
}

void motor_control_bench_flush(void)
{
    if (!s_bench_save_pending) return;
    bench_write_csv();                      /* sets SAVED or FAILED */
    s_bench_save_pending = false;
    bench_status_commit();                  /* only now is the file real */
}

static void bench_tick(int64_t now_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool start_req = s_bench_start_pending;
    s_bench_start_pending = false;
    uint32_t kind = s_bench_req_kind;
    float base = s_bench_req_base, delta = s_bench_req_delta;
    float reset_c = s_bench_req_reset_c;
    s_bench_req_reset_c = 0.0f;             /* one shot, consumed here */
    portEXIT_CRITICAL(&s_arbiter_lock);

    if (start_req && !s_bench_active && !s_calibrating) {
        if (esc_driver_get_state() == ESC_STATE_ARMED && !s_rail_cut &&
            fs_sdcard_ready() && !s_bench_save_pending) {
            if (kind > (uint32_t)BENCH_KIND_RIGHT) kind = (uint32_t)BENCH_KIND_BASE;
            bench_init(&s_bench);
            s_bench_cfg.run_us = (kind == (uint32_t)BENCH_KIND_BASE)
                               ? BENCH_RUN_US_BASE : BENCH_RUN_US_SPLIT;
            if (bench_start(&s_bench, (bench_kind_t)kind, base, delta, now_us)) {
                s_bench_active = true;
                s_bench_file_index = 0;
                /* Inside the accepted branch on purpose: a rejected start must
                 * leave the learner exactly as it was, or a refused button
                 * press would silently discard everything it had learned. */
                if (reset_c > 0.0f) {
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
                    if (trim_learn_reset(&s_trim_learn, &s_trim_learn_cfg,
                                         reset_c)) {
                        s_trim_last_capture_us = 0;
                        trim_assist_reset(&s_trim_assist);
                        s_p_correction = 0.0f;
                        ESP_LOGW(TAG, "BENCH,trimlearn_reset,c=%.3f",
                                 (double)reset_c);
                    } else {
                        ESP_LOGW(TAG, "BENCH,trimlearn_reset_REFUSED,c=%.3f,"
                                      "bounds=%.2f..%.2f", (double)reset_c,
                                 (double)s_trim_learn_cfg.c_min,
                                 (double)s_trim_learn_cfg.c_max);
                    }
#else
                    ESP_LOGW(TAG, "BENCH,trimlearn_reset_IGNORED,"
                                  "learner compiled out");
#endif
                }
                ESP_LOGI(TAG, "BENCH,start,kind=%u,base=%.2f,delta=%.2f",
                         (unsigned)kind, (double)base, (double)delta);
            }
        } else {
            ESP_LOGW(TAG, "BENCH,start_rejected,armed=%d,rail_cut=%d,sd=%d,saving=%d",
                     (int)(esc_driver_get_state() == ESC_STATE_ARMED),
                     (int)s_rail_cut, (int)fs_sdcard_ready(),
                     (int)s_bench_save_pending);
        }
    }

    if (!s_bench_active) return;

    FusionResult fusion = {0};
    fusion_get_result(&fusion);
    /* A run must measure the trim the boat is ACTUALLY running, or "press BASE
     * and see if it goes straight" answers a question about some other trim.
     * With the learner on that is its current c, read fresh EVERY TICK -- the
     * learner stays live through a BASE run (trim_learn_tick only bails for
     * LEFT/RIGHT runs and calibration), so the correction reaches the jets
     * during the run and the per-sample c column records it moving. */
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    const float bench_trim = 2.0f * effective_trim_c(s_trim_learn.c) * s_bench.base;
#else
    const float bench_trim = esc_trim_lookup(s_esc_trim, s_esc_trim_count,
                                             s_bench.base);
#endif
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    const bench_assist_t bench_pa = {
        .yaw_filt   = s_trim_assist.yaw_filt,
        .correction = s_p_correction,
        .learned_c  = s_trim_learn.c,
        .on         = s_p_assist_on,
        .at_cap     = s_trim_assist.at_cap,
    };
    const bench_assist_t *pa = &bench_pa;
#else
    const bench_assist_t *pa = NULL;
#endif
    bench_out_t o = bench_step(&s_bench, &s_bench_cfg, now_us, fusion.yaw_rate,
                               esc_driver_get_state() == ESC_STATE_ARMED,
                               bench_trim, pa);

    if (o.active) {
        /* Last ESC write of the cycle, overriding control_apply_decision --
         * the driving link is deliberately idle during a run. Rudder centred.
         *
         * The stored trim is applied HERE TOO, exactly as esc_trim_mix would.
         * Without it a bench run drives the raw, uncorrected motors, so a BASE
         * run after setting a trim would read unchanged and look like the fix
         * failed. With it the bench becomes a closed loop: measure, apply,
         * re-measure, and a corrected boat reads zero. */
        esc_driver_set_throttle(o.left_cmd, o.right_cmd);
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);
        }
    }

    if (o.finished) {
        esc_driver_set_throttle(0.0f, 0.0f);
        /* The SD write is FAR too slow for this 10 ms critical loop (tens of
         * fopen/fclose on FAT, and the card shares a driver mutex with the C6
         * radio). Hand it to the core-1 diagnostics task instead. */
        s_bench_save_pending = true;
        s_bench_active = false;
    } else if (o.aborted) {
        esc_driver_set_throttle(0.0f, 0.0f);
        s_bench_active = false;
        ESP_LOGW(TAG, "BENCH,aborted,reason=%s -- nothing saved",
                 o.reason ? o.reason : "?");
    }

    /* ~5 Hz, plus always on a terminal state, same as the calibration status. */
    static uint8_t bench_status_div = 0;
    if ((++bench_status_div % 20) == 0 || o.aborted) {
        bench_status_commit();
    }
}

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

    /* A stop (CalibrateCommand{start:false}) re-arms the start latch. The
     * dashboards send start:true as a ~1 Hz keepalive while calibrating, so a
     * bare "start:true when idle" cannot mean "start": it's usually a stale
     * keepalive still arriving after a sweep already finished (the ESP-NOW
     * dashboard can't see the DONE status to stop it; even the WiFi one has a
     * ~1 s window). Without this latch those keepalives would endlessly
     * re-trigger calibration. A start only fires on a clean edge: idle AND
     * re-armed by a prior stop (or boot). */
    if (stop_req) {
        s_cal_ready = true;
    }
    if (start_req && !s_calibrating && s_cal_ready) {
        if (esc_driver_get_state() == ESC_STATE_ARMED && cal_link_alive && !s_rail_cut) {
            s_cal_cfg.average_into_existing = average;
            /* Force-armed ("bench / no GPS") -> drop the making-way gate so a
             * level records on yaw settling alone (operator watches the boat).
             * Normal-armed (GPS present) -> keep the gate at its Kconfig value. */
            s_cal_cfg.min_speed_mps = s_force_armed ? 0.0f : s_cal_min_speed_cfg;
            esc_trim_cal_start(&s_cal, &s_cal_cfg);
            s_cal_baseline_rx_us = link_rx_us;   /* any later manual cmd advances past this = abort */
            s_cal_out = (etc_out_t){0};
            s_calibrating = true;
            s_cal_ready = false;                 /* latched until an explicit stop re-arms */
            ESP_LOGI(TAG, "CAL,start,avg=%d,levels=%u,gps_gate=%d",
                     (int)average, (unsigned)s_cal_cfg.level_count, (int)!s_force_armed);
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
    /* force-armed skips the gate (min_speed is 0 for this run), so report making
     * way; otherwise report the real GPS-based state. */
    bool making_way = s_force_armed || (gps.valid && gps.speed_mps >= s_cal_cfg.min_speed_mps);

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
    bench_tick(start_us);
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
    pipeline_register_bench_handler(bench_command_handler);
    pipeline_register_assist_handler(assist_command_handler);
#if CONFIG_STABILITY_TRIMLEARN_ENABLE
    s_trim_learn_cfg = (trim_learn_cfg_t){
        /* Same number the static table uses -- the learner starts where the
         * bench left off and only ever corrects the residual. */
        .c_init = parse_cfg_float(CONFIG_ESC_TRIM_C, 0.20f, 0.0f, 0.5f),
        .c_min = 0.10f, .c_max = 0.35f,
        .deadband_dps = parse_cfg_float(CONFIG_STABILITY_TRIMLEARN_DEADBAND_DPS, 0.5f, 0.05f, 10.0f),
        .step_per_s = parse_cfg_float(CONFIG_STABILITY_TRIMLEARN_STEP_PER_S, 0.005f, 0.0001f, 0.1f),
        .yaw_tau_s = 2.0f,
        .min_throttle = parse_cfg_float(CONFIG_STABILITY_TRIMLEARN_MIN_THROTTLE, 0.15f, 0.0f, 1.0f),
        .reject_dps = TRIM_LEARN_REJECT_DPS,
    };
    s_trim_assist_cfg = (trim_assist_cfg_t){
        .kp    = parse_cfg_float(CONFIG_STABILITY_PASSIST_KP, 0.020f, 0.0f, 0.20f),
        .tau_s = parse_cfg_float(CONFIG_STABILITY_PASSIST_TAU_S, 0.50f, 0.02f, 5.0f),
        .cap   = parse_cfg_float(CONFIG_STABILITY_PASSIST_CAP, 0.030f, 0.0f, 0.20f),
    };
    trim_assist_reset(&s_trim_assist);
    ESP_LOGI(TAG, "P-assist built in (default OFF): kp=%.3f tau=%.2fs cap=%.3f",
             (double)s_trim_assist_cfg.kp, (double)s_trim_assist_cfg.tau_s,
             (double)s_trim_assist_cfg.cap);
    trim_learn_init(&s_trim_learn, &s_trim_learn_cfg);
    ESP_LOGI(TAG, "TrimLearn ON: c=%.3f deadband=%.2f deg/s step=%.4f/s "
                  "min_thr=%.2f reject=%.0f deg/s",
             s_trim_learn_cfg.c_init, s_trim_learn_cfg.deadband_dps,
             s_trim_learn_cfg.step_per_s, s_trim_learn_cfg.min_throttle,
             s_trim_learn_cfg.reject_dps);
#endif

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
