#include "autonomy.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "auto_drive.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "file_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mission.h"
#include "motor_control.h"
#include "pipeline.h"
#include "runtime_metrics.h"
#include "runtime_schedule.h"
#include "runtime_task.h"
#include "sensor_fusion.h"

static const char *TAG = "MISSION";

/* Built only with the heading hold it flies through, and the Kconfig switch
 * (a bool set to n is ABSENT from sdkconfig.h: absence means off). */
#if defined(CONFIG_MISSION_ENABLE) && CONFIG_STABILITY_TRIMLEARN_ENABLE
#define AUTONOMY_BUILT 1
#else
#define AUTONOMY_BUILT 0
#endif

#if AUTONOMY_BUILT

#define AUTONOMY_PERIOD_MS      50          /* 20 Hz, as simulated */
#define STATUS_ACTIVE_US        200000      /* 5 Hz while a mission runs */
#define STATUS_IDLE_US          1000000     /* 1 Hz otherwise */
/* The on-boat record, 20 Hz in PSRAM.  20 min (~3 MB) rather than the spec's
 * 30: a 10 m mission takes ~70 s and the tether stops it at ~20 m.  A run
 * that outlasts the buffer keeps its first 20 min and says so. */
#define RECORD_HZ               20
#define RECORD_MAX_SAMPLES      (20u * 60u * RECORD_HZ)
#define RECORD_FALLBACK_SAMPLES (5u * 60u * RECORD_HZ)
#define RECORD_CHUNK            8192

/* MissionStatus.record_state */
typedef enum {
    REC_NONE = 0,
    REC_RECORDING = 1,
    REC_SAVING = 2,
    REC_SAVED = 3,
    REC_FAILED = 4,
    REC_UNAVAILABLE = 5,
} rec_state_t;

/* One 50 ms step of the run, as the boat saw it. */
typedef struct {
    float t_s;
    uint8_t state;
    uint8_t reason;
    uint8_t sats;
    uint8_t flags;            /* REC_F_* */
    double lat;
    double lon;
    float pdop;
    float sacc;
    float fix_age_s;
    float speed;
    float course;
    float heading;
    float yaw_dps;
    float wanted_course;
    float wanted_heading;
    float dist_target;
    float bearing_target;
    float cross_track;
    float progress;
    float beta;
    float turned;
    float throttle;
    float left;
    float right;
    float hold_target;
    float p_term;
    float i_term;
} msn_sample_t;

#define REC_F_LINK      0x01u
#define REC_F_GPS_NEW   0x02u
#define REC_F_HOLD      0x04u
#define REC_F_APPROACH  0x08u
#define REC_F_BETA      0x10u

/* ---- RX task -> mission task ------------------------------------------- */
static portMUX_TYPE s_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static boat_MissionCommand s_cmd;
static bool s_cmd_start = false;
static bool s_cmd_stop = false;

/* ---- mission task only --------------------------------------------------- */
static mission_t s_m;
static mission_cfg_t s_cfg;
static int64_t s_t0_us = 0;
static int64_t s_last_fix_us = 0;
static int64_t s_last_status_us = 0;
static uint32_t s_rec_run = 0;               /* the run the buffer holds */
static mission_state_t s_prev_state = MISSION_IDLE;
static uint32_t s_prev_run = 0;

/* ---- the record: filled by the mission task, written by diagnostics ------ */
static msn_sample_t *s_rec = NULL;
static uint32_t s_rec_cap = 0;
static uint32_t s_rec_n = 0;
static bool s_rec_overflow = false;
static char *s_rec_chunk = NULL;
static mission_t s_rec_final;                /* the run as it ended, for the header */
/* The mission task hands a finished record over by setting this; diagnostics
 * clears it once the file is written.  A new START is refused meanwhile, so
 * the buffer is never written by both. */
static atomic_bool s_save_pending;
static _Atomic uint32_t s_rec_state;         /* rec_state_t */
static _Atomic uint32_t s_rec_file_index;
static uint32_t s_rec_next_index = 1;        /* diagnostics only */

/* ---- mission task -> diagnostics: the status double buffer --------------- */
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static boat_MissionStatus s_status;
static uint32_t s_status_generation = 0;
static atomic_uintptr_t s_status_reader;
static uint32_t s_published_generation = 0;  /* diagnostics only */

static void mission_command_handler(const boat_MissionCommand *cmd)
{
    if (!cmd) return;
    /* STOP reaches the jets through the control task at once -- not 50 ms
     * later through this task -- and wins over a START in the same window. */
    if (cmd->stop) motor_control_request_auto_stop();
    portENTER_CRITICAL(&s_cmd_lock);
    if (cmd->stop) {
        s_cmd_stop = true;
        s_cmd_start = false;
    } else if (cmd->start) {
        s_cmd = *cmd;
        s_cmd_start = true;
    }
    portEXIT_CRITICAL(&s_cmd_lock);
}

static mission_settings_t settings_from(const boat_MissionCommand *c)
{
    mission_settings_t s = {
        .stage = c->stage,
        .out_distance_m = c->out_distance_m,
        .home_radius_m = c->home_radius_m,
        .throttle = c->throttle,
        .approach_throttle = c->approach_throttle,
        .turn_right = c->turn_right,
        .dry_run = c->dry_run,
    };
    return s;
}

static bool is_active(mission_state_t s)
{
    return s == MISSION_HOME || s == MISSION_OUTBOUND || s == MISSION_TURN || s == MISSION_RETURN;
}

static bool is_terminal(mission_state_t s)
{
    return s == MISSION_DONE || s == MISSION_ABORTED || s == MISSION_REFUSED;
}

static float finite_or(float v, float fallback)
{
    return isfinite(v) ? v : fallback;
}

/* The mission's view of the boat, gathered fresh every step. */
static void gather_inputs(mission_input_t *in, int64_t now_us)
{
    gps_fix_t fix;
    memset(&fix, 0, sizeof(fix));
    (void)gps_driver_get_fix(&fix);
    gps_runtime_status_t gps_rt;
    memset(&gps_rt, 0, sizeof(gps_rt));
    (void)gps_driver_get_runtime_status(&gps_rt);

    in->gps_new = fix.last_update_us > 0 && fix.last_update_us != s_last_fix_us;
    s_last_fix_us = fix.last_update_us;
    in->gps_ubx = gps_rt.protocol_authority == GPS_PROTOCOL_UBX;
    /* fixOk (gnssFixOK) off means the receiver itself does not trust it. */
    in->fix_type = fix.valid ? fix.fix_quality : 0u;
    in->sats = fix.satellites;
    in->pdop = fix.hdop;                     /* UBX: NAV-PVT pDOP (gps_driver.c) */
    in->fix_age_s = fix.last_update_us > 0 ? (float)(now_us - fix.last_update_us) / 1e6f : 99.0f;
    in->lat_deg = fix.latitude;
    in->lon_deg = fix.longitude;
    in->speed_mps = fix.speed_mps;
    in->course_deg = fix.course_deg;
    in->speed_acc_mps = fix.speed_acc_mps;

    FusionResult f;
    memset(&f, 0, sizeof(f));
    fusion_get_result(&f);
    in->heading_valid = f.heading_valid;
    in->heading_deg = f.heading;
    in->yaw_rate_dps = f.yaw_rate;
    in->fusion_age_s = (f.sequence != 0 && f.captured_us != 0)
                     ? (float)(now_us - (int64_t)f.captured_us) / 1e6f : 99.0f;
    in->imu_ok = imu_icm_ok();
    /* > 0 only with a compass calibration AND live magnetometer data. */
    in->field_ratio = fusion_get_field_ratio();
    in->compass_calibrated = in->field_ratio > 0.0f;

    in->armed = motor_control_armed();
    in->busy = motor_control_jets_busy() || atomic_load(&s_save_pending);
    in->heading_hold_built = motor_control_mission_built();
    in->link_alive = motor_control_link_alive();
    uint32_t abort_run = 0;
    uint8_t abort_reason = 0;
    motor_control_get_auto_abort(&abort_run, &abort_reason);
    in->control_abort_run = abort_run;
    in->control_abort_reason = abort_reason;
}

static void record_sample(const mission_input_t *in, const mission_output_t *out,
                          const motor_drive_snapshot_t *drive)
{
    if (!s_rec || atomic_load(&s_rec_state) != REC_RECORDING) return;
    if (s_rec_n >= s_rec_cap) {
        s_rec_overflow = true;
        return;
    }
    msn_sample_t *r = &s_rec[s_rec_n++];
    r->t_s = in->t_s - s_m.t_start_s;
    r->state = (uint8_t)s_m.state;
    r->reason = s_m.reason;
    r->sats = in->sats;
    r->flags = (uint8_t)((in->link_alive ? REC_F_LINK : 0u) | (in->gps_new ? REC_F_GPS_NEW : 0u) |
                         (drive->hold_active ? REC_F_HOLD : 0u) | (s_m.approach ? REC_F_APPROACH : 0u) |
                         (s_m.beta_valid ? REC_F_BETA : 0u));
    r->lat = in->lat_deg;
    r->lon = in->lon_deg;
    r->pdop = in->pdop;
    r->sacc = in->speed_acc_mps;
    r->fix_age_s = in->fix_age_s;
    r->speed = in->speed_mps;
    r->course = in->course_deg;
    r->heading = in->heading_deg;
    r->yaw_dps = in->yaw_rate_dps;
    r->wanted_course = s_m.wanted_course_deg;
    r->wanted_heading = s_m.wanted_heading_deg;
    r->dist_target = s_m.dist_target_m;
    r->bearing_target = s_m.bearing_target_deg;
    r->cross_track = s_m.state == MISSION_RETURN ? s_m.line.cross_m : 0.0f;
    r->progress = s_m.state == MISSION_RETURN ? s_m.line.progress : 0.0f;
    r->beta = s_m.beta_deg;
    r->turned = s_m.turned_deg;
    r->throttle = out->throttle;
    r->left = drive->left;
    r->right = drive->right;
    r->hold_target = drive->hold_target_deg;
    r->p_term = drive->p_term;
    r->i_term = drive->i_term;
}

static void commit_status(const mission_input_t *in, const motor_drive_snapshot_t *drive)
{
    boat_MissionStatus st = boat_MissionStatus_init_zero;
    st.run_id = s_m.run_id;
    st.request_id = s_m.request_id;
    st.state = (uint32_t)s_m.state;
    st.reason = s_m.reason;
    st.stage = s_m.settings.stage;
    st.out_distance_m = s_m.settings.out_distance_m;
    st.home_radius_m = s_m.settings.home_radius_m;
    st.throttle = s_m.settings.throttle;
    st.approach_throttle = s_m.settings.approach_throttle;
    st.turn_right = s_m.settings.turn_right;
    st.dry_run = s_m.settings.dry_run;
    st.home_lat = s_m.have_origin ? s_m.home_lat_deg : 0.0;
    st.home_lon = s_m.have_origin ? s_m.home_lon_deg : 0.0;
    st.dist_target_m = finite_or(s_m.dist_target_m, 0.0f);
    st.bearing_target_deg = finite_or(s_m.bearing_target_deg, 0.0f);
    st.cross_track_m = s_m.state == MISSION_RETURN ? finite_or(s_m.line.cross_m, 0.0f) : 0.0f;
    st.wanted_heading_deg = finite_or(s_m.wanted_heading_deg, 0.0f);
    st.heading_deg = finite_or(in->heading_deg, 0.0f);
    st.beta_deg = finite_or(s_m.beta_deg, 0.0f);
    st.beta_valid = s_m.beta_valid;
    st.compass_bad = s_m.compass_bad;
    st.turned_deg = s_m.turned_deg;
    st.turn_peak_dps = s_m.turn_peak_dps;
    st.approach = s_m.approach;
    st.hold_active = drive->hold_active;
    st.p_switch = motor_control_p_assist_on();
    st.outages = s_m.outages;
    st.longest_outage_s = s_m.longest_outage_s;
    st.closest_m = (s_m.state == MISSION_RETURN || s_m.state == MISSION_DONE) ? s_m.closest_m : 0.0f;
    st.elapsed_s = s_m.run_id != 0 ? (is_active(s_m.state) ? in->t_s : s_m.t_state_s) - s_m.t_start_s : 0.0f;
    st.record_state = atomic_load(&s_rec_state);
    st.file_index = atomic_load(&s_rec_file_index);
    st.dist_home_m = s_m.have_pos ? s_m.dist_home_m : 0.0f;
    st.speed_acc_mps = finite_or(in->speed_acc_mps, 0.0f);
    st.course_samples = s_m.out_mean.n + s_m.beta_return_n;
    st.gps_outliers = s_m.outliers;
    st.sats = in->sats;
    st.pdop = finite_or(in->pdop, 0.0f);
    st.start_heading_deg = s_m.have_origin ? s_m.start_heading_deg : 0.0f;
    const bool returning = s_m.state == MISSION_RETURN || (s_m.state == MISSION_DONE && s_m.have_origin);
    st.return_start_e_m = returning ? s_m.return_start.e : 0.0f;
    st.return_start_n_m = returning ? s_m.return_start.n : 0.0f;

    portENTER_CRITICAL(&s_status_lock);
    s_status = st;
    s_status_generation++;
    portEXIT_CRITICAL(&s_status_lock);
    TaskHandle_t reader = (TaskHandle_t)atomic_load(&s_status_reader);
    if (reader) xTaskNotifyGive(reader);
}

static const char *reason_text(uint8_t r)
{
    switch (r) {
    case MISSION_DONE_OUT: return "out distance reached";
    case MISSION_DONE_TURNED: return "turned to face home";
    case MISSION_DONE_IN_ZONE: return "in the home zone";
    case MISSION_DONE_PASSED: return "passed home";
    case MISSION_DONE_NEAR: return "near home";
    case MISSION_ABORT_STOP: return "STOP";
    case MISSION_ABORT_MANUAL: return "manual input";
    case MISSION_ABORT_DISARMED: return "disarmed";
    case MISSION_ABORT_RAIL_CUT: return "servo rail cut";
    case MISSION_ABORT_OTHER_OWNER: return "something else took the jets";
    case MISSION_ABORT_STALE: return "setpoint stale";
    case MISSION_ABORT_GPS_LOST: return "GPS lost";
    case MISSION_ABORT_HEADING_LOST: return "heading lost";
    case MISSION_ABORT_HOME_NOT_STILL: return "home not still";
    case MISSION_ABORT_SPIN: return "spinning";
    case MISSION_ABORT_GPS_FROZEN: return "GPS frozen";
    case MISSION_REFUSE_SETTINGS: return "settings";
    case MISSION_REFUSE_NO_HOLD: return "no heading hold";
    case MISSION_REFUSE_BUSY: return "busy";
    case MISSION_REFUSE_NOT_ARMED: return "not armed";
    case MISSION_REFUSE_GPS_NOT_UBX: return "GPS not UBX";
    case MISSION_REFUSE_NO_3D_FIX: return "no 3D fix";
    case MISSION_REFUSE_FEW_SATS: return "few satellites";
    case MISSION_REFUSE_PDOP: return "pDOP";
    case MISSION_REFUSE_FIX_OLD: return "fix old";
    case MISSION_REFUSE_COMPASS: return "compass not calibrated";
    case MISSION_REFUSE_FIELD: return "magnetic field off";
    case MISSION_REFUSE_IMU: return "IMU";
    case MISSION_REFUSE_HEADING: return "heading";
    default: return "";
    }
}

/* The Avoidance slot (spec section 3): the last word before the setpoint.
 * Today it passes the mission's demand through unchanged.  Later it is the
 * emergency layer -- ToF and the camera model: slow down, stop, turn in
 * place -- allowed to override whatever guidance asked for, as SUSHI's
 * reactive control behaviours override its path follower. */
static mission_output_t avoidance(const mission_output_t *demand)
{
    return *demand;
}

/* One 50 ms step.  Everything the mission knows comes in here and its
 * setpoint goes out to the control task every step, active or not -- an idle
 * setpoint (active = false) is what keeps the jets unowned. */
static void autonomy_step(int64_t now_us)
{
    portENTER_CRITICAL(&s_cmd_lock);
    const bool start = s_cmd_start;
    const bool stop = s_cmd_stop;
    const boat_MissionCommand cmd = s_cmd;
    s_cmd_start = false;
    s_cmd_stop = false;
    portEXIT_CRITICAL(&s_cmd_lock);

    mission_input_t in;
    memset(&in, 0, sizeof(in));
    in.t_s = (float)((double)(now_us - s_t0_us) / 1e6);
    gather_inputs(&in, now_us);
    in.stop = stop;
    if (start) {
        in.start = true;
        in.request_id = cmd.request_id;
        in.settings = settings_from(&cmd);
    }

    /* mission -> navigation -> planner -> guidance (mission.c, path_follow.c,
     * planner.c), then avoidance, then the control task. */
    const mission_output_t demand = mission_step(&s_m, &s_cfg, &in);
    const mission_output_t out = avoidance(&demand);
    const auto_setpoint_t sp = {
        .run_id = s_m.run_id,
        .active = out.active,
        .drive = out.drive,
        .dry_run = out.dry_run,
        .heading_deg = out.heading_deg,
        .throttle = out.throttle,
        .stamp_us = now_us,
    };
    motor_control_set_auto_setpoint(&sp);

    motor_drive_snapshot_t drive;
    motor_control_get_drive_snapshot(&drive);

    /* A new run: start its record (a refusal never gets one). */
    if (s_m.run_id != s_prev_run && is_active(s_m.state)) {
        s_rec_run = s_m.run_id;
        s_rec_n = 0;
        s_rec_overflow = false;
        atomic_store(&s_rec_file_index, 0u);
        atomic_store(&s_rec_state, s_rec ? (uint32_t)REC_RECORDING : (uint32_t)REC_UNAVAILABLE);
        ESP_LOGW(TAG, "MISSION,start,run=%u,stage=%u,out=%.1f,radius=%.1f,thr=%.2f,approach=%.2f,%s%s",
                 (unsigned)s_m.run_id, (unsigned)s_m.settings.stage,
                 (double)s_m.settings.out_distance_m, (double)s_m.settings.home_radius_m,
                 (double)s_m.settings.throttle, (double)s_m.settings.approach_throttle,
                 s_m.settings.turn_right ? "right" : "left", s_m.settings.dry_run ? ",DRY RUN" : "");
    }
    if (s_m.run_id == s_rec_run && (is_active(s_m.state) || is_active(s_prev_state))) {
        record_sample(&in, &out, &drive);
    }
    if (s_m.state != s_prev_state || s_m.run_id != s_prev_run) {
        if (is_terminal(s_m.state)) {
            ESP_LOGW(TAG, "MISSION,%s,run=%u,reason=%u (%s),dist_home=%.2f,closest=%.2f,outages=%u",
                     s_m.state == MISSION_DONE ? "done" : s_m.state == MISSION_ABORTED ? "aborted" : "refused",
                     (unsigned)s_m.run_id, (unsigned)s_m.reason, reason_text(s_m.reason),
                     (double)s_m.dist_home_m, (double)s_m.closest_m, (unsigned)s_m.outages);
            /* A run that drove leaves a record to write. */
            if (s_m.run_id == s_rec_run && s_rec && atomic_load(&s_rec_state) == REC_RECORDING) {
                s_rec_final = s_m;
                atomic_store(&s_rec_state, (uint32_t)REC_SAVING);
                atomic_store(&s_save_pending, true);
            }
        } else {
            ESP_LOGI(TAG, "MISSION,state=%u,run=%u", (unsigned)s_m.state, (unsigned)s_m.run_id);
        }
        s_last_status_us = 0;                 /* a change goes out at once */
    }
    s_prev_state = s_m.state;
    s_prev_run = s_m.run_id;

    const int64_t period = is_active(s_m.state) ? STATUS_ACTIVE_US : STATUS_IDLE_US;
    if (s_last_status_us == 0 || now_us - s_last_status_us >= period) {
        s_last_status_us = now_us;
        commit_status(&in, &drive);
    }
}

static void task_autonomy(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(AUTONOMY_PERIOD_MS);
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        const int64_t started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_AUTONOMY, (uint64_t)started, (uint64_t)started);
        autonomy_step(started);
        runtime_metrics_cycle_end(RUNTIME_TASK_AUTONOMY, (uint64_t)esp_timer_get_time());
        vTaskDelayUntil(&next, period);
    }
}

/* ---- diagnostics task: the SD file ---------------------------------------- */
static bool write_record(void)
{
    if (!fs_sdcard_ready()) {
        ESP_LOGE(TAG, "MISSION,record_failed,no SD card");
        return false;
    }
    char name[16];
    uint32_t idx = 0;
    for (uint32_t k = 0; k < 999u; ++k) {
        const uint32_t i = ((s_rec_next_index - 1u + k) % 999u) + 1u;
        uint8_t probe = 0;
        size_t got = 0;
        snprintf(name, sizeof(name), "MSN_%03u.CSV", (unsigned)i);
        if (fs_sdcard_read(name, &probe, 1, &got) == ESP_ERR_NOT_FOUND) {
            idx = i;                           /* first free slot: never overwrite */
            break;
        }
    }
    if (idx == 0) {
        ESP_LOGE(TAG, "MISSION,record_failed,no free MSN_NNN.CSV name");
        return false;
    }
    const mission_t *m = &s_rec_final;
    char *c = s_rec_chunk;
    int n = snprintf(c, RECORD_CHUNK,
        "# out-and-back mission run %u (request %u), recorded on the boat at %u Hz\n"
        "# settings: stage %u, out %.1f m, home radius %.1f m, throttle %.2f, approach %.2f, turn %s, dry run %d\n"
        "# home: %.7f,%.7f  start heading %.1f\n"
        "# result: state %u reason %u (%s), dist_home %.2f m, closest %.2f m, turned %.0f deg, "
        "beta %.1f deg (valid %d, compass bad %d), outages %u (longest %.1f s), gps outliers %u, "
        "samples %u%s\n"
        "t_s,state,reason,lat,lon,sats,pdop,sacc,fix_age_s,speed,course,heading,yaw_dps,"
        "wanted_course,wanted_heading,dist_target,bearing_target,cross_track,progress,beta,turned,"
        "throttle,left,right,hold_target,p_term,i_term,link,gps_new,hold,approach,beta_valid\n",
        (unsigned)m->run_id, (unsigned)m->request_id, (unsigned)RECORD_HZ,
        (unsigned)m->settings.stage, (double)m->settings.out_distance_m,
        (double)m->settings.home_radius_m, (double)m->settings.throttle,
        (double)m->settings.approach_throttle, m->settings.turn_right ? "right" : "left",
        (int)m->settings.dry_run,
        m->home_lat_deg, m->home_lon_deg, (double)m->start_heading_deg,
        (unsigned)m->state, (unsigned)m->reason, reason_text(m->reason),
        (double)m->dist_home_m, (double)m->closest_m, (double)m->turned_deg,
        (double)m->beta_deg, (int)m->beta_valid, (int)m->compass_bad,
        (unsigned)m->outages, (double)m->longest_outage_s, (unsigned)m->outliers,
        (unsigned)s_rec_n, s_rec_overflow ? " -- BUFFER FULL, the rest of the run is missing" : "");
    if (n <= 0 || n >= RECORD_CHUNK || fs_sdcard_write(name, c, (size_t)n) != ESP_OK) {
        ESP_LOGE(TAG, "MISSION,record_failed,%s", name);
        return false;
    }
    n = 0;
    for (uint32_t i = 0; i < s_rec_n; ++i) {
        if (n > RECORD_CHUNK - 512) {
            if (fs_sdcard_append(name, c, (size_t)n) != ESP_OK) goto fail;
            n = 0;
        }
        const msn_sample_t *r = &s_rec[i];
        const int w = snprintf(c + n, (size_t)(RECORD_CHUNK - n),
            "%.2f,%u,%u,%.7f,%.7f,%u,%.2f,%.3f,%.2f,%.3f,%.1f,%.1f,%.2f,%.1f,%.1f,%.2f,%.1f,"
            "%.2f,%.3f,%.1f,%.1f,%.2f,%.3f,%.3f,%.1f,%.4f,%.4f,%u,%u,%u,%u,%u\n",
            (double)r->t_s, (unsigned)r->state, (unsigned)r->reason, r->lat, r->lon,
            (unsigned)r->sats, (double)r->pdop, (double)r->sacc, (double)r->fix_age_s,
            (double)r->speed, (double)r->course, (double)r->heading, (double)r->yaw_dps,
            (double)r->wanted_course, (double)r->wanted_heading, (double)r->dist_target,
            (double)r->bearing_target, (double)r->cross_track, (double)r->progress,
            (double)r->beta, (double)r->turned, (double)r->throttle, (double)r->left,
            (double)r->right, (double)r->hold_target, (double)r->p_term, (double)r->i_term,
            (unsigned)((r->flags & REC_F_LINK) != 0), (unsigned)((r->flags & REC_F_GPS_NEW) != 0),
            (unsigned)((r->flags & REC_F_HOLD) != 0), (unsigned)((r->flags & REC_F_APPROACH) != 0),
            (unsigned)((r->flags & REC_F_BETA) != 0));
        if (w < 0 || w >= RECORD_CHUNK - n) goto fail;
        n += w;
    }
    if (n > 0 && fs_sdcard_append(name, c, (size_t)n) != ESP_OK) goto fail;
    atomic_store(&s_rec_file_index, idx);
    s_rec_next_index = (idx % 999u) + 1u;
    ESP_LOGI(TAG, "MISSION,saved,%s,samples=%u", name, (unsigned)s_rec_n);
    return true;
fail:
    ESP_LOGE(TAG, "MISSION,record_failed,%s -- the file on the card is INCOMPLETE", name);
    return false;
}

void autonomy_diagnostics_tick(bool periodic)
{
    (void)periodic;                     /* the mission task paces the status itself */
    atomic_store(&s_status_reader, (uintptr_t)xTaskGetCurrentTaskHandle());
    if (atomic_load(&s_save_pending)) {
        const bool ok = write_record();
        atomic_store(&s_rec_state, ok ? (uint32_t)REC_SAVED : (uint32_t)REC_FAILED);
        atomic_store(&s_save_pending, false);
    }
    boat_MissionStatus st;
    portENTER_CRITICAL(&s_status_lock);
    const uint32_t gen = s_status_generation;
    st = s_status;
    portEXIT_CRITICAL(&s_status_lock);
    if (gen != 0 && gen != s_published_generation) {
        s_published_generation = gen;
        /* The record state may have moved since the status was built. */
        st.record_state = atomic_load(&s_rec_state);
        st.file_index = atomic_load(&s_rec_file_index);
        pipeline_publish_mission_status(&st);
    }
}

esp_err_t autonomy_init(void)
{
    mission_init(&s_m);
    s_cfg = mission_cfg_default();
    s_t0_us = esp_timer_get_time();
    atomic_store(&s_save_pending, false);
    atomic_store(&s_rec_state, (uint32_t)REC_NONE);
    atomic_store(&s_rec_file_index, 0u);
    atomic_store(&s_status_reader, (uintptr_t)NULL);

    /* The record lives in PSRAM; without it the mission still flies and says
     * "record unavailable". */
    s_rec_chunk = heap_caps_malloc(RECORD_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const uint32_t tries[2] = { RECORD_MAX_SAMPLES, RECORD_FALLBACK_SAMPLES };
    for (int i = 0; i < 2 && s_rec_chunk && !s_rec; ++i) {
        s_rec = heap_caps_malloc((size_t)tries[i] * sizeof(msn_sample_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_rec) s_rec_cap = tries[i];
    }
    if (!s_rec) {
        ESP_LOGW(TAG, "mission record buffer unavailable -- missions fly without an SD record");
    }

    pipeline_register_mission_handler(mission_command_handler);
    const esp_err_t err = runtime_task_create(RUNTIME_TASK_AUTONOMY, task_autonomy, NULL, NULL);
    if (err != ESP_OK) {
        /* No task, no mission: START gets no answer, the boat drives as before. */
        pipeline_register_mission_handler(NULL);
        ESP_LOGE(TAG, "mission task failed to start (%s) -- no missions this boot",
                 esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Mission task running (20 Hz, core 1); record %u s at %u Hz%s",
             (unsigned)(s_rec_cap / RECORD_HZ), (unsigned)RECORD_HZ, s_rec ? "" : " -- UNAVAILABLE");
    return ESP_OK;
}

#else  /* !AUTONOMY_BUILT */

esp_err_t autonomy_init(void)
{
    ESP_LOGI(TAG, "Mission not built (CONFIG_MISSION_ENABLE off or no heading hold)");
    return ESP_OK;
}

void autonomy_diagnostics_tick(bool periodic)
{
    (void)periodic;
}

#endif
