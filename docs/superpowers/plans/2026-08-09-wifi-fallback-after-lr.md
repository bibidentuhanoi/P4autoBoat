# Wi-Fi Fallback After ESP-NOW LR Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore ordinary Wi-Fi fallback after an unsuccessful ESP-NOW LR probe without changing the working LR path.

**Architecture:** Keep the C6 protocol bitmap at B/G/N/LR when fallback begins, avoiding the unmatched ESP-Hosted station-stop event caused by changing it on a running radio. A host-side C harness compiles the real `wifi_manager.c` against a deterministic fake ESP-IDF boundary and verifies that `wifi_connect()` reaches its observable success result.

**Tech Stack:** ESP-IDF C, ESP-Hosted remote Wi-Fi, host C compiler, Python `unittest` test runner.

## Global Constraints

- Do not change the successful ESP-NOW/LR activation path.
- Do not modify managed Espressif component sources.
- Do not require an S3 firmware change.
- Preserve the existing 30-second connection timeout and disconnect retry behavior.

---

### Task 1: Restore Wi-Fi fallback

**Files:**
- Create: `tests/wifi_fallback_stubs/test_esp_idf.h`
- Create: `tests/wifi_fallback_stubs/esp_err.h`
- Create: `tests/wifi_fallback_stubs/sdkconfig.h`
- Create: `tests/wifi_fallback_stubs/esp_log.h`
- Create: `tests/wifi_fallback_stubs/esp_wifi.h`
- Create: `tests/wifi_fallback_stubs/esp_event.h`
- Create: `tests/wifi_fallback_stubs/esp_netif.h`
- Create: `tests/wifi_fallback_stubs/freertos/FreeRTOS.h`
- Create: `tests/wifi_fallback_stubs/freertos/task.h`
- Create: `tests/wifi_fallback_stubs/freertos/event_groups.h`
- Create: `tests/test_wifi_fallback.c`
- Create: `tests/test_wifi_fallback.py`
- Modify: `main/wifi_manager.c:129-144`

**Interfaces:**
- Consumes: `esp_err_t wifi_start_radio(void)` and `esp_err_t wifi_connect(void)` from `main/wifi_manager.h`.
- Produces: a host test whose process exit status is zero only when the real fallback returns `ESP_OK` after the fake remote station supplies `IP_EVENT_STA_GOT_IP`.

- [ ] **Step 1: Write the failing host-side behavior test**

Create minimal ESP-IDF stub headers. Each public-name wrapper contains only:

```c
#pragma once
#include "test_esp_idf.h"
```

`test_esp_idf.h` defines the exact ESP-IDF surface referenced by
`wifi_manager.c`: `esp_err_t`; `ESP_OK`, `ESP_ERR_NO_MEM`,
`ESP_ERR_INVALID_STATE`, and `ESP_ERR_TIMEOUT`; the `WIFI_EVENT` and
`IP_EVENT` bases; station start, stop, disconnect, and got-IP IDs;
`esp_event_handler_t`; `wifi_init_config_t`; `wifi_config_t` with 32-byte SSID
and 64-byte password arrays; `wifi_interface_t`; `wifi_mode_t`;
`wifi_ps_type_t`; `ip_event_got_ip_t`; `esp_netif_t`; FreeRTOS tick and event
group types; `BIT0`, `BIT1`, `pdFALSE`, `pdTRUE`, `portMAX_DELAY`, and
`pdMS_TO_TICKS`; no-op logging macros; and declarations for every `esp_netif`,
event-loop, Wi-Fi, event-group, and delay function called by the production
translation unit. Its `WIFI_INIT_CONFIG_DEFAULT()` macro returns a zeroed
`wifi_init_config_t`, and its `ESP_ERROR_CHECK()` macro aborts on non-OK
results so initialization failures cannot pass silently.

In `tests/test_wifi_fallback.c`, implement deterministic fakes with this behavior:

```c
esp_err_t esp_wifi_start(void)
{
    fake_netif_started = true;
    post_wifi_event(WIFI_EVENT_STA_START, NULL);
    return ESP_OK;
}

esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t bitmap)
{
    (void)ifx;
    (void)bitmap;
    fake_netif_started = false;
    post_wifi_event(WIFI_EVENT_STA_STOP, NULL);
    return ESP_OK;
}

esp_err_t esp_wifi_connect(void)
{
    if (fake_netif_started) {
        ip_event_got_ip_t got_ip = {0};
        post_ip_event(IP_EVENT_STA_GOT_IP, &got_ip);
    }
    return ESP_OK;
}

int main(void)
{
    if (wifi_start_radio() != ESP_OK) {
        return 1;
    }
    return wifi_connect() == ESP_OK ? 0 : 2;
}
```

`tests/test_wifi_fallback.py` compiles `main/wifi_manager.c` and the C harness
in a temporary directory, runs the resulting executable, and asserts a zero
exit status. Define the compiler command as:

```python
ROOT = Path(__file__).resolve().parents[1]
COMPILE_COMMAND = [
    os.environ.get("CC", "cc"),
    "-std=c11",
    "-Wall",
    "-Werror",
    "-I", str(ROOT / "tests" / "wifi_fallback_stubs"),
    "-I", str(ROOT / "main"),
    str(ROOT / "main" / "wifi_manager.c"),
    str(ROOT / "tests" / "test_wifi_fallback.c"),
]
```

Then run and assert the harness result:

```python
class WifiFallbackTest(unittest.TestCase):
    def test_fallback_gets_ip_without_stopping_hosted_netif(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "wifi_fallback_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)
```

The production mutation this catches is adding a protocol change to the already-started fallback path: the fake reproduces its station-stop side effect, preventing the IP event and making `wifi_connect()` return `ESP_ERR_TIMEOUT`.

- [ ] **Step 2: Run the test to verify it fails for the reported bug**

Run:

```bash
python -m unittest -v tests.test_wifi_fallback
```

Expected: FAIL because the harness exits with status 2 after the current `esp_wifi_set_protocol()` call marks the fake hosted netif stopped.

- [ ] **Step 3: Make the minimal firmware change**

Delete only the fallback-time protocol-reset block from `wifi_connect()`:

```c
esp_err_t wifi_connect(void)
{
    if (!s_wifi_events) {
        ESP_LOGE(TAG, "wifi_connect() before wifi_start_radio()");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);
```

Do not change the C6 ESP-NOW initialization that sets B/G/N/LR.

- [ ] **Step 4: Run the focused test to verify it passes**

Run:

```bash
python -m unittest -v tests.test_wifi_fallback
```

Expected: PASS; the fake netif stays started, the simulated IP event is delivered, and `wifi_connect()` returns `ESP_OK`.

- [ ] **Step 5: Run all Python tests**

Run:

```bash
python -m unittest discover -v tests
```

Expected: all tests PASS with zero failures and zero errors.

- [ ] **Step 6: Build the P4 firmware**

Run:

```bash
idf.py build
```

Expected: exit status 0 and the final ESP-IDF build-complete message.

- [ ] **Step 7: Review and commit the isolated fix**

Run:

```bash
git diff --check
git diff -- main/wifi_manager.c tests/test_wifi_fallback.c tests/test_wifi_fallback.py tests/wifi_fallback_stubs
git add main/wifi_manager.c tests/test_wifi_fallback.c tests/test_wifi_fallback.py tests/wifi_fallback_stubs
git add -f docs/superpowers/plans/2026-08-09-wifi-fallback-after-lr.md
git commit -m "fix(wifi): preserve hosted netif during LR fallback"
```

Do not stage the pre-existing `tools/espnow_drive.py`, other existing files under `tests/`, or `tools/esp-dl` changes unless they are one of the paths listed above.

Hardware acceptance after flashing the P4 is: boot with the S3 off, observe `No ground station — falling back to WiFi`, then observe `Got IP` instead of `WiFi connection timed out`.
