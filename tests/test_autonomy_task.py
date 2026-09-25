"""The mission task's glue (main/autonomy.c), on the host.

main/mission.c and the AUTO owner are tested on their own; this checks what
autonomy.c adds between them and the boat:
  * the inputs: GPS fields (UBX only, fixOk, pDOP from NAV-PVT, fix age, a
    new fix detected by its timestamp), fusion, compass calibrated, armed,
    busy, link;
  * the setpoint goes to the control task EVERY step, active or not;
  * STOP reaches the control task at once, from the RX task itself;
  * MissionStatus: 5 Hz while running, 1 Hz idle, at once on a change;
  * the record: filled at 20 Hz, handed to the diagnostics task at the end,
    written as MSN_NNN.CSV (never overwriting), status says SAVED + the file;
  * a START while that record is still being written is refused (busy).
Built on the REAL generated proto header (nanopb from managed_components).
"""
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NANOPB = ROOT / "managed_components" / "livekit__nanopb" / "include"

HEADERS = {
    "esp_err.h": r"""
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_NOT_FOUND 0x105
static inline const char *esp_err_to_name(esp_err_t e) { (void)e; return "err"; }
""",
    "esp_log.h": r"""
#pragma once
#include <stdio.h>
#define ESP_LOGD(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) do { (void)(tag); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGE(tag, ...) do { (void)(tag); printf("E: "); printf(__VA_ARGS__); printf("\n"); } while (0)
""",
    "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_heap_caps.h": r"""
#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
static inline void *heap_caps_malloc(size_t n, unsigned caps) { (void)caps; return malloc(n); }
""",
    "freertos/FreeRTOS.h": r"""
#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(l) ((void)(l))
#define portEXIT_CRITICAL(l) ((void)(l))
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdPASS 1
""",
    "freertos/task.h": r"""
#pragma once
#include "freertos/FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *prev, TickType_t period);
BaseType_t xTaskNotifyGive(TaskHandle_t t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
""",
    "runtime_metrics.h": r"""
#pragma once
#include <stdint.h>
#include "runtime_schedule.h"
void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t scheduled, uint64_t started);
void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t ended);
""",
    "runtime_task.h": r"""
#pragma once
#include "esp_err.h"
#include "freertos/task.h"
#include "runtime_schedule.h"
esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn, void *arg, TaskHandle_t *out);
""",
    "drivers/imu_driver.h": "#pragma once\n#include <stdbool.h>\nbool imu_icm_ok(void);\n",
    "sensor_fusion.h": r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { float pitch; float roll; float heading; bool heading_valid; float yaw_rate;
                 uint32_t sequence; uint64_t captured_us; } FusionResult;
void fusion_get_result(FusionResult *r);
float fusion_get_field_ratio(void);
""",
    "file_system.h": r"""
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
bool fs_sdcard_ready(void);
esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o);
esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l);
esp_err_t fs_sdcard_append(const char *p, const void *d, size_t l);
""",
    "pipeline.h": r"""
#pragma once
#include "proto/boat.pb.h"
typedef void (*mission_command_handler_fn)(const boat_MissionCommand *cmd);
void pipeline_register_mission_handler(mission_command_handler_fn handler);
void pipeline_publish_mission_status(const boat_MissionStatus *status);
""",
    "motor_control.h": r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "auto_drive.h"
bool motor_control_mission_built(void);
void motor_control_set_auto_setpoint(const auto_setpoint_t *sp);
void motor_control_request_auto_stop(void);
void motor_control_get_auto_abort(uint32_t *run_id, uint8_t *reason);
bool motor_control_jets_busy(void);
bool motor_control_link_alive(void);
bool motor_control_armed(void);
bool motor_control_p_assist_on(void);
typedef struct { float left; float right; bool hold_active; float hold_target_deg;
                 float p_term; float i_term; } motor_drive_snapshot_t;
void motor_control_get_drive_snapshot(motor_drive_snapshot_t *out);
""",
}

HARNESS = r"""
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "autonomy.c"

/* ---- the boat, scripted --------------------------------------------------- */
static int64_t now = 1000000;
static gps_fix_t fix;
static gps_protocol_authority_t protocol = GPS_PROTOCOL_UBX;
static FusionResult fus;
static float field = 1.0f;
static bool armed = true, busy = false, link = true, imu = true;
static uint32_t abort_run = 0; static uint8_t abort_why = 0;
static mission_command_handler_fn handler = NULL;
static TaskFunction_t task_fn = NULL;
static auto_setpoint_t last_sp; static unsigned sp_writes = 0;
static unsigned stop_requests = 0, status_publishes = 0;
static boat_MissionStatus last_status;
static char card[16][32]; static unsigned card_files = 0;
static char file_body[1 << 20]; static size_t file_len = 0;

int64_t esp_timer_get_time(void) { return now; }
TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskDelayUntil(TickType_t *prev, TickType_t period) { (void)prev; (void)period; }
BaseType_t xTaskNotifyGive(TaskHandle_t t) { (void)t; return pdPASS; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t a, uint64_t b) { (void)id; (void)a; (void)b; }
void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t e) { (void)id; (void)e; }
esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn, void *arg, TaskHandle_t *out) {
    assert(id == RUNTIME_TASK_AUTONOMY); (void)arg; (void)out; task_fn = fn; return ESP_OK;
}
esp_err_t gps_driver_get_fix(gps_fix_t *out) { *out = fix; return ESP_OK; }
esp_err_t gps_driver_get_runtime_status(gps_runtime_status_t *out) {
    memset(out, 0, sizeof(*out)); out->protocol_authority = protocol; return ESP_OK;
}
bool imu_icm_ok(void) { return imu; }
void fusion_get_result(FusionResult *r) { *r = fus; }
float fusion_get_field_ratio(void) { return field; }
bool fs_sdcard_ready(void) { return true; }
esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o) {
    (void)b; (void)c; if (o) *o = 0;
    for (unsigned i = 0; i < card_files; ++i) if (!strcmp(card[i], p)) return ESP_OK;
    return ESP_ERR_NOT_FOUND;
}
esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l) {
    assert(card_files < 16); strcpy(card[card_files++], p);
    memcpy(file_body, d, l); file_len = l; return ESP_OK;
}
esp_err_t fs_sdcard_append(const char *p, const void *d, size_t l) {
    assert(!strcmp(card[card_files - 1], p)); assert(file_len + l < sizeof(file_body));
    memcpy(file_body + file_len, d, l); file_len += l; return ESP_OK;
}
void pipeline_register_mission_handler(mission_command_handler_fn h) { handler = h; }
void pipeline_publish_mission_status(const boat_MissionStatus *s) { last_status = *s; ++status_publishes; }
bool motor_control_mission_built(void) { return true; }
void motor_control_set_auto_setpoint(const auto_setpoint_t *sp) { last_sp = *sp; ++sp_writes; }
void motor_control_request_auto_stop(void) { ++stop_requests; }
void motor_control_get_auto_abort(uint32_t *r, uint8_t *w) { *r = abort_run; *w = abort_why; }
bool motor_control_jets_busy(void) { return busy; }
bool motor_control_link_alive(void) { return link; }
bool motor_control_armed(void) { return armed; }
bool motor_control_p_assist_on(void) { return true; }
void motor_control_get_drive_snapshot(motor_drive_snapshot_t *o) {
    *o = (motor_drive_snapshot_t){.left = 0.3f, .right = 0.5f, .hold_active = true,
                                  .hold_target_deg = last_sp.heading_deg, .p_term = 0.01f};
}

/* A boat going wherever the setpoint says at 0.4 m/s when driving. */
static double lat = 21.0417483, lon = 105.8862132;
static float heading = 30.0f;
static void step(int n) {
    for (int i = 0; i < n; ++i) {
        now += 50000;
        if (last_sp.active && last_sp.drive) {
            heading = last_sp.heading_deg;
            lat += 0.4 * 0.05 * cos(heading * M_PI / 180.0) / 110540.0;
            lon += 0.4 * 0.05 * sin(heading * M_PI / 180.0) / (111320.0 * cos(21.04 * M_PI / 180.0));
        }
        if (i % 2 == 0) {                               /* GPS 10 Hz */
            fix.valid = true; fix.fix_quality = 3; fix.satellites = 12; fix.hdop = 1.4f;
            fix.latitude = lat; fix.longitude = lon;
            fix.speed_mps = last_sp.drive ? 0.4f : 0.02f; fix.course_deg = heading;
            fix.speed_acc_mps = 0.08f; fix.last_update_us = now;
        }
        fus.heading = heading; fus.heading_valid = true; fus.yaw_rate = 0.0f;
        fus.sequence++; fus.captured_us = (uint64_t)now;
        autonomy_step(now);
        autonomy_diagnostics_tick(false);
    }
}

static boat_MissionCommand start_cmd(uint32_t id) {
    return (boat_MissionCommand){.start = true, .request_id = id, .stage = 3, .out_distance_m = 10.0f,
                                 .home_radius_m = 2.5f, .throttle = 0.40f, .approach_throttle = 0.20f,
                                 .turn_right = true};
}

int main(void) {
    assert(autonomy_init() == ESP_OK);
    assert(handler && task_fn && s_rec && s_rec_cap >= RECORD_FALLBACK_SAMPLES);

    /* Idle: a setpoint every step, inactive; status at 1 Hz. */
    step(40);
    assert(sp_writes == 40 && !last_sp.active && last_sp.run_id == 0);
    assert(status_publishes >= 2 && status_publishes <= 3);
    assert(last_status.state == 0 && last_status.sats == 12 && fabsf(last_status.pdop - 1.4f) < 1e-6f);

    /* Input mapping: a GPS that is not UBX refuses; so does no compass. */
    protocol = GPS_PROTOCOL_NMEA;
    boat_MissionCommand c = start_cmd(1); handler(&c); step(1);
    assert(s_m.state == MISSION_REFUSED && s_m.reason == MISSION_REFUSE_GPS_NOT_UBX);
    assert(last_status.state == MISSION_REFUSED && last_status.reason == MISSION_REFUSE_GPS_NOT_UBX);
    protocol = GPS_PROTOCOL_UBX; field = 0.0f;
    c = start_cmd(2); handler(&c); step(1);
    assert(s_m.reason == MISSION_REFUSE_COMPASS);
    field = 1.0f;
    fix.valid = false;                                  /* fixOk off: not a 3D fix */
    c = start_cmd(3); handler(&c); now += 50000; autonomy_step(now);
    assert(s_m.reason == MISSION_REFUSE_NO_3D_FIX);
    busy = true;
    c = start_cmd(4); handler(&c); step(1);
    assert(s_m.reason == MISSION_REFUSE_BUSY);
    busy = false;
    assert(atomic_load(&s_rec_state) == REC_NONE);      /* refusals are never recorded */

    /* A real start: HOME, then OUTBOUND on the start heading. */
    c = start_cmd(5); handler(&c); step(1);
    assert(s_m.state == MISSION_HOME && last_sp.active && !last_sp.drive && last_sp.run_id == s_m.run_id);
    assert(atomic_load(&s_rec_state) == REC_RECORDING);
    step(60);
    assert(s_m.state == MISSION_OUTBOUND && last_sp.drive && fabsf(last_sp.throttle - 0.40f) < 1e-6f);
    assert(fabsf(last_sp.heading_deg - 30.0f) < 1.0f);

    /* 5 Hz status while running. */
    unsigned before = status_publishes;
    step(20);                                           /* 1 s */
    assert(status_publishes - before >= 5 && status_publishes - before <= 6);
    assert(last_status.state == MISSION_OUTBOUND && last_status.hold_active && last_status.p_switch);
    assert(fabsf(last_status.speed_acc_mps - 0.08f) < 1e-6f);
    step(60);                                           /* past the 3 s settle: beta samples */
    assert(s_m.state == MISSION_OUTBOUND && last_status.course_samples > 0);

    /* STOP: the control task hears it from the RX task, at once. */
    boat_MissionCommand stop = {.stop = true};
    handler(&stop);
    assert(stop_requests == 1);
    const uint32_t run = s_m.run_id;
    now += 50000; autonomy_step(now);
    assert(s_m.state == MISSION_ABORTED && s_m.reason == MISSION_ABORT_STOP);
    assert(!last_sp.active);
    /* the record is handed over; a START meanwhile is refused as busy */
    assert(atomic_load(&s_save_pending) && atomic_load(&s_rec_state) == REC_SAVING);
    c = start_cmd(6); handler(&c); now += 50000; autonomy_step(now);
    assert(s_m.state == MISSION_REFUSED && s_m.reason == MISSION_REFUSE_BUSY);
    autonomy_diagnostics_tick(false);                   /* the diagnostics task writes it */
    assert(!atomic_load(&s_save_pending) && atomic_load(&s_rec_state) == REC_SAVED);
    assert(card_files == 1 && !strcmp(card[0], "MSN_001.CSV"));
    assert(atomic_load(&s_rec_file_index) == 1u);
    file_body[file_len] = 0;
    assert(strstr(file_body, "# out-and-back mission run"));
    assert(strstr(file_body, "reason 10 (STOP)"));
    const char *data = strstr(file_body, "beta_valid\n");
    assert(data);
    data += strlen("beta_valid\n");
    unsigned rows = 0;
    for (const char *p = data; *p; ++p) rows += (*p == '\n');
    printf("rows=%u samples=%u\n", rows, (unsigned)s_rec_n);
    assert(rows == s_rec_n && rows > 60);
    step(1);
    assert(last_status.record_state == REC_SAVED && last_status.file_index == 1u && last_status.run_id == run + 1);

    /* The next run's record takes the next free name. */
    c = start_cmd(7); handler(&c); step(80);
    handler(&stop); step(2);
    assert(card_files == 2 && !strcmp(card[1], "MSN_002.CSV"));
    printf("OK\n");
    return 0;
}
"""


def test_the_mission_task_glue():
    if not (NANOPB / "pb.h").exists():
        pytest.skip("nanopb headers absent (managed_components not fetched)")
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        for relative, content in HEADERS.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        shutil.copy(ROOT / "main" / "autonomy.c", tmpdir / "autonomy.c")
        (tmpdir / "harness.c").write_text(HARNESS)
        binary = tmpdir / "autonomy_test"
        subprocess.run(
            [os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
             "-DCONFIG_MISSION_ENABLE=1", "-DCONFIG_STABILITY_TRIMLEARN_ENABLE=1",
             "-I", str(tmpdir), "-I", str(ROOT / "main"), "-I", str(ROOT / "main" / "drivers"),
             "-I", str(NANOPB),
             str(tmpdir / "harness.c"),
             *[str(ROOT / "main" / s) for s in ("mission.c", "nav_geo.c", "course_error.c")],
             "-lm", "-o", str(binary)],
            check=True)
        res = subprocess.run([str(binary)], capture_output=True, text=True)
        assert res.returncode == 0 and "OK" in res.stdout, res.stdout[-3000:] + res.stderr[-3000:]
