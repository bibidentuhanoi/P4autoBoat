"""Two VL53L5CX sensors on one bus: the REAL tof_init() (main/drivers/tof_driver.c)
against a simulated bus.

Kiet, 2026-09-26: "right now we have one tofs but we gonna have another tofs so
2 tofs would that break the code?"

Both chips wake at the default address 0x29; LPn (A: GPIO 52, B: GPIO 29) turns
a chip's I2C interface on or off, and an address change survives a warm boot.
tof_init brings A up and moves it to 0x22 while B is held off, then wakes B and
moves it to 0x24. Two chips answering one address is the dangerous case: they
have the same ID, so the probe LOOKS healthy while every write reaches both and
every read is the two answers wired together. The model counts that as a
collision -- which must never happen.
"""
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "managed_components" / "rjrp44__vl53l5cx"

STUBS = {
    "driver/gpio.h": r"""
#pragma once
#include <stdint.h>
typedef int gpio_num_t;
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
typedef struct { uint64_t pin_bit_mask; int mode; int pull_up_en; int pull_down_en; int intr_type; } gpio_config_t;
int gpio_config(const gpio_config_t *c);
int gpio_set_level(gpio_num_t gpio, uint32_t level);
int gpio_set_direction(gpio_num_t gpio, int mode);
""",
    "driver/i2c_master.h": r"""
#pragma once
#include <stddef.h>
#include <stdint.h>
typedef struct sim_dev *i2c_master_dev_handle_t;
typedef struct sim_bus *i2c_master_bus_handle_t;
typedef enum { I2C_ADDR_BIT_LEN_7 = 0 } i2c_addr_bit_len_t;
typedef struct { int unused; } i2c_master_bus_config_t;
typedef struct { i2c_addr_bit_len_t dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz; } i2c_device_config_t;
int i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *cfg, i2c_master_dev_handle_t *out);
int i2c_master_bus_rm_device(i2c_master_dev_handle_t h);
""",
    "esp_err.h": r"""
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NOT_FINISHED 0x10C
#define ESP_ERR_INVALID_CRC 0x109
""",
    "esp_log.h": r"""
#pragma once
#include <stdio.h>
#define ESP_LOGI(tag, ...) do { (void)(tag); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGW(tag, ...) do { (void)(tag); printf("W: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGE(tag, ...) do { (void)(tag); printf("E: "); printf(__VA_ARGS__); printf("\n"); } while (0)
""",
    "freertos/FreeRTOS.h": "#pragma once\n#define pdMS_TO_TICKS(ms) (ms)\n#define portTICK_PERIOD_MS 1\n",
    "freertos/task.h": "#pragma once\n#include <stdint.h>\nvoid vTaskDelay(uint32_t ticks);\n",
    "sdkconfig.h": r"""
#pragma once
#define CONFIG_TOF_A_LPN_PIN 52
#define CONFIG_TOF_B_LPN_PIN 29
#define CONFIG_TOF_A_ADDR 0x22
#define CONFIG_TOF_B_ADDR 0x24
#define CONFIG_TOF_I2C_FREQ_HZ 1000000
#define CONFIG_TOF_RANGING_FREQ_HZ 10
#define CONFIG_TOF_INTEGRATION_TIME_MS 10
""",
    "file_system.h": r"""
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
bool fs_load_tof_xtalk(const char *key, uint8_t *data, size_t len);
bool fs_save_tof_xtalk(const char *key, const uint8_t *data, size_t len);
""",
}

HARNESS = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tof_driver.h"

/* ---- the simulated bus ---- */
typedef struct {
    const char *name; int lpn_pin; bool present, enabled, stuck_addr, fw_fail, initialized;
    uint16_t addr;
} sim_sensor_t;
static sim_sensor_t S[2];
struct sim_dev { uint16_t addr; bool used; };
static struct sim_dev devs[32];
static unsigned collisions;

static int responders(uint16_t addr, sim_sensor_t **who) {
    int n = 0;
    for (int i = 0; i < 2; ++i)
        if (S[i].present && S[i].enabled && S[i].addr == addr) { if (who) who[n] = &S[i]; ++n; }
    return n;
}
int gpio_config(const gpio_config_t *c) { (void)c; return 0; }
int gpio_set_direction(gpio_num_t g, int m) { (void)g; (void)m; return 0; }
int gpio_set_level(gpio_num_t pin, uint32_t level) {
    for (int i = 0; i < 2; ++i) if (S[i].lpn_pin == pin) S[i].enabled = level != 0;
    return 0;
}
void vTaskDelay(uint32_t t) { (void)t; }
int i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *cfg,
                              i2c_master_dev_handle_t *out) {
    (void)bus;
    for (int i = 0; i < 32; ++i) if (!devs[i].used) {
        devs[i].used = true; devs[i].addr = cfg->device_address; *out = &devs[i]; return 0;
    }
    return -1;
}
int i2c_master_bus_rm_device(i2c_master_dev_handle_t h) { h->used = false; return 0; }
bool fs_load_tof_xtalk(const char *k, uint8_t *d, size_t l) { (void)k; memset(d, 0, l); return true; }
bool fs_save_tof_xtalk(const char *k, const uint8_t *d, size_t l) { (void)k; (void)d; (void)l; return true; }

/* ---- the sensor library, as the chips on this bus answer it ---- */
static uint16_t at(VL53L5CX_Configuration *d) { return d->platform.handle ? d->platform.handle->addr : 0xFFFF; }
static int touch(VL53L5CX_Configuration *d, sim_sensor_t **who) {
    const int n = responders(at(d), who);
    if (n >= 2) ++collisions;               /* both answer: it LOOKS fine, it is not */
    return n;
}
uint8_t vl53l5cx_is_alive(VL53L5CX_Configuration *d, uint8_t *alive) {
    const int n = touch(d, NULL);
    *alive = n > 0; return n > 0 ? 0 : 255;
}
uint8_t vl53l5cx_set_i2c_address(VL53L5CX_Configuration *d, uint16_t addr8) {
    sim_sensor_t *who[2]; const int n = touch(d, who);
    for (int i = 0; i < n; ++i) if (!who[i]->stuck_addr) who[i]->addr = (uint16_t)(addr8 >> 1);
    return 0;                                /* its status is unreliable anyway */
}
uint8_t vl53l5cx_init(VL53L5CX_Configuration *d) {
    sim_sensor_t *who[2]; const int n = touch(d, who);
    if (n == 0) return 255;
    for (int i = 0; i < n; ++i) { if (who[i]->fw_fail) return 66; who[i]->initialized = true; }
    return 0;
}
uint8_t vl53l5cx_set_caldata_xtalk(VL53L5CX_Configuration *d, uint8_t *x) { (void)x; return touch(d, NULL) ? 0 : 255; }
uint8_t vl53l5cx_calibrate_xtalk(VL53L5CX_Configuration *d, uint16_t r, uint8_t n, uint16_t mm) { (void)d; (void)r; (void)n; (void)mm; return 0; }
uint8_t vl53l5cx_get_caldata_xtalk(VL53L5CX_Configuration *d, uint8_t *x) { (void)d; (void)x; return 0; }
uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *d, uint8_t r) { (void)r; touch(d, NULL); return 0; }
uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *d, uint8_t f) { (void)f; touch(d, NULL); return 0; }
uint8_t vl53l5cx_set_integration_time_ms(VL53L5CX_Configuration *d, uint32_t t) { (void)t; touch(d, NULL); return 0; }
uint8_t vl53l5cx_set_target_order(VL53L5CX_Configuration *d, uint8_t o) { (void)o; touch(d, NULL); return 0; }
uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *d) { return touch(d, NULL) ? 0 : 255; }
uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *d, uint8_t *r) { (void)d; *r = 0; return 0; }
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *d, VL53L5CX_ResultsData *r) { (void)d; (void)r; return 0; }

static tof_devices_t tof;
static void boot(bool a, bool b, uint16_t a_addr, uint16_t b_addr) {
    memset(S, 0, sizeof(S)); memset(devs, 0, sizeof(devs)); memset(&tof, 0, sizeof(tof)); collisions = 0;
    S[0] = (sim_sensor_t){.name = "A", .lpn_pin = 52, .present = a, .enabled = true, .addr = a_addr};
    S[1] = (sim_sensor_t){.name = "B", .lpn_pin = 29, .present = b, .enabled = true, .addr = b_addr};
}
static void run(void) { assert(tof_init((i2c_master_bus_handle_t)0, &tof) == 0); }
static void always(void) {
    assert(collisions == 0);                                  /* never two chips on one address */
    for (int i = 0; i < 2; ++i) {
        const bool ok = i == 0 ? tof.a_ok : tof.b_ok;
        if (S[i].present && !ok) assert(!S[i].enabled);       /* a chip that did not come up is switched off */
        if (ok) assert(S[i].enabled && S[i].initialized && S[i].addr == (i == 0 ? 0x22 : 0x24));
    }
}

#define CASE(name) printf("case: %s\n", name)
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    /* both, cold boot (both at the default address) */
    CASE("both, cold boot"); boot(true, true, 0x29, 0x29); run(); always(); assert(tof.a_ok && tof.b_ok);
    /* only A, only B (today's boat has one) */
    CASE("only A"); boot(true, false, 0x29, 0x29); run(); always(); assert(tof.a_ok && !tof.b_ok);
    CASE("only B"); boot(false, true, 0x29, 0x29); run(); always(); assert(!tof.a_ok && tof.b_ok);
    /* warm boot: the addresses survived */
    CASE("both, warm boot"); boot(true, true, 0x22, 0x24); run(); always(); assert(tof.a_ok && tof.b_ok);
    /* A answers at 0x29 but its address change does not take: B must still come up */
    CASE("A stuck at 0x29"); boot(true, true, 0x29, 0x29); S[0].stuck_addr = true; run(); always();
    assert(!tof.a_ok && tof.b_ok);
    /* A moves but its firmware load fails: switched off, B fine */
    CASE("A firmware fails"); boot(true, true, 0x29, 0x29); S[0].fw_fail = true; run(); always();
    assert(!tof.a_ok && tof.b_ok);
    /* B stuck at 0x29: A fine, B switched off */
    CASE("B stuck at 0x29"); boot(true, true, 0x29, 0x29); S[1].stuck_addr = true; run(); always();
    assert(tof.a_ok && !tof.b_ok);
    printf("OK\n");
    return 0;
}
"""


def test_two_tof_sensors_come_up_without_ever_sharing_an_address():
    if not (COMPONENT / "include" / "vl53l5cx_api.h").exists():
        pytest.skip("VL53L5CX component absent (managed_components not fetched)")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        for rel, content in STUBS.items():
            (d / rel).parent.mkdir(parents=True, exist_ok=True)
            (d / rel).write_text(content)
        (d / "harness.c").write_text(HARNESS)
        binary = d / "tof_init_test"
        subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-parameter", "-DCONFIG_VL53L5CX_NB_TARGET_PER_ZONE=4",
                        "-I", str(d), "-I", str(ROOT / "main" / "drivers"),
                        "-I", str(COMPONENT / "include"),
                        str(ROOT / "main" / "drivers" / "tof_driver.c"), str(d / "harness.c"),
                        "-o", str(binary)], check=True)
        res = subprocess.run([str(binary)], capture_output=True, text=True)
        assert res.returncode == 0 and "OK" in res.stdout, res.stdout[-3000:] + res.stderr[-2000:]
