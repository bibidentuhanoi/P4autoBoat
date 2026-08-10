import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_every_runtime_task_is_created_from_its_pinned_schedule_entry():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        (tmpdir / "freertos").mkdir()
        (tmpdir / "esp_err.h").write_text(
            "#pragma once\ntypedef int esp_err_t;\n"
            "#define ESP_OK 0\n#define ESP_ERR_INVALID_ARG 0x102\n#define ESP_ERR_NO_MEM 0x101\n"
        )
        (tmpdir / "esp_log.h").write_text(
            "#pragma once\nvoid test_log(const char *, const char *, ...);\n"
            "#define ESP_LOGE(...) test_log(__VA_ARGS__)\n"
        )
        (tmpdir / "freertos" / "FreeRTOS.h").write_text(
            "#pragma once\n#include <stdint.h>\n"
            "typedef int BaseType_t; typedef unsigned UBaseType_t;\n"
            "#define pdPASS 1\n"
        )
        (tmpdir / "freertos" / "task.h").write_text(
            "#pragma once\n#include <stdint.h>\n#include \"freertos/FreeRTOS.h\"\n"
            "typedef void *TaskHandle_t; typedef void (*TaskFunction_t)(void *);\n"
            "BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char *, uint32_t, "
            "void *, UBaseType_t, TaskHandle_t *, BaseType_t);\n"
        )
        binary = tmpdir / "runtime_task_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DESP_PLATFORM",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(ROOT / "main" / "runtime_schedule.c"),
                str(ROOT / "main" / "runtime_task.c"),
                str(ROOT / "tests" / "test_runtime_task.c"),
                "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
