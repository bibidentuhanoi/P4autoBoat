import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

STUB_HEADERS = {
    # The REAL main/include/common.h is used (it only needs esp_err.h), so
    # the calibration layout under test can never drift from the firmware's.
    "esp_err.h": "#pragma once\ntypedef int esp_err_t;\n",
    "sdkconfig.h": r"""
#pragma once
#define CONFIG_FUSION_COMPLEMENTARY_ALPHA "0.96"
#define CONFIG_FUSION_YAW_ALPHA "0.98"
#define CONFIG_HEADING_DECLINATION_DEG "0"
""",
    "esp_log.h": "#pragma once\n",
    "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "runtime_metrics.h": r"""
#pragma once
#include <stdint.h>
#define RUNTIME_TASK_FUSION 0
#define RUNTIME_EVENT_SENSOR_SKIP 0
void runtime_metrics_count(int task, int event);
void runtime_metrics_cycle_begin(int task, uint64_t scheduled, uint64_t started);
void runtime_metrics_cycle_end(int task, uint64_t ended);
""",
    "freertos/FreeRTOS.h": r"""
#pragma once
#include <stdint.h>
typedef int BaseType_t;
#define pdTRUE 1
#define portMAX_DELAY ((uint32_t)-1)
""",
    "freertos/task.h": r"""
#pragma once
#include "freertos/FreeRTOS.h"
typedef void *TaskHandle_t;
TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, uint32_t wait);
""",
    "sensor_task.h": r"""
#pragma once
#include "freertos/task.h"
#include "sample_snapshot.h"
sample_snapshot_t *sensor_imu_sample_snapshot(void);
void sensor_task_register_fusion_task(TaskHandle_t task);
""",
}

HARNESS_STUBS = r"""
#include <stdint.h>
#include "sample_snapshot.h"
#include "freertos/task.h"
int64_t esp_timer_get_time(void) { return 9000000; }
void runtime_metrics_count(int task, int event) { (void)task; (void)event; }
void runtime_metrics_cycle_begin(int task, uint64_t scheduled, uint64_t started) {
    (void)task; (void)scheduled; (void)started;
}
void runtime_metrics_cycle_end(int task, uint64_t ended) { (void)task; (void)ended; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
uint32_t ulTaskNotifyTake(BaseType_t clear, uint32_t wait) { (void)clear; (void)wait; return 0; }
sample_snapshot_t *sensor_imu_sample_snapshot(void) { return 0; }
void sensor_task_register_fusion_task(TaskHandle_t task) { (void)task; }
"""


class SensorFusionTest(unittest.TestCase):
    def test_fusion_consumes_timestamped_samples_and_publishes_coherent_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmpdir = Path(tmp)
            for relative, content in STUB_HEADERS.items():
                path = tmpdir / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            (tmpdir / "stubs.c").write_text(HARNESS_STUBS)
            binary = tmpdir / "sensor_fusion_test"
            subprocess.run(
                [
                    os.environ.get("CC", "cc"),
                    "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread",
                    "-I", str(tmpdir), "-I", str(ROOT / "main"), "-I", str(ROOT / "main" / "include"),
                    str(ROOT / "main" / "sample_snapshot.c"),
                    str(ROOT / "main" / "sensor_fusion.c"),
                    str(ROOT / "main" / "mag_cal.c"),
                    str(ROOT / "tests" / "test_sensor_fusion.c"),
                    str(tmpdir / "stubs.c"), "-lm", "-o", str(binary),
                ],
                check=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
