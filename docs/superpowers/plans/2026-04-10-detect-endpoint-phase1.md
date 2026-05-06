# /detect Endpoint Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `GET /detect` HTTP endpoint to the boat firmware that runs ESPDet-Pico cat detection on an embedded test JPEG and returns JSON, to validate co-residency of on-demand inference with IMU/ToF/WS/camera subsystems.

**Architecture:** Single new C++ module (`main/detection/`) quarantined behind an `extern "C"` API so the rest of the pure-C firmware is unaffected. ESPDet-Pico cat model lives in its own `cat_det` flash partition (not rodata), preempting L2 cache contention. Lazy initialization on first `/detect` hit; subsequent calls reuse the allocated model and decoded image. Runs in the existing httpd worker task — no new tasks, no mutexes, no IMU/ToF/WS changes.

**Tech Stack:** ESP-IDF 5.4.3, C++17, `espressif/cat_detect` managed component (via `override_path` to in-repo `docs/esp-dl/models/cat_detect`), esp-dl, `esp_http_server`, cJSON-free hand-written JSON marshalling.

**Spec:** [docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md](../specs/2026-04-10-detect-endpoint-phase1-design.md)

---

## Task 1: Tier 0 — Dependency and partition plumbing (dry build, no C++ code)

**Rationale:** Before committing to ~200 lines of C++, prove that adding `cat_detect` as a managed component, adding the `cat_det` data partition, and flipping the FLASH_PARTITION sdkconfig lines actually builds. If Tier 0 fails, the problem is infrastructure, not code. Per the spec's bisectability principle.

**Files:**
- Modify: `main/idf_component.yml`
- Modify: `main/CMakeLists.txt` (add one line to `PRIV_REQUIRES`)
- Modify: `sdkconfig.defaults`
- Modify: `partitions.csv`

- [ ] **Step 1.1: Append `cat_detect` dependency to `main/idf_component.yml`**

Current file ends with the `livekit/nanopb` block. Append:

```yaml
  ## Detection model (phase 1: cat_detect for co-residency test)
  espressif/cat_detect:
    version: "*"
    override_path: "../docs/esp-dl/models/cat_detect"
```

Full resulting file:

```yaml
## IDF Component Manager Manifest File
dependencies:
  idf:
    version: '>=5.3'
  rjrp44/vl53l5cx:
    version: "^4.0.0"
  ## Camera: video framework + JPEG encoder
  espressif/esp_video:
    version: "2.0.1"
  espressif/esp_new_jpeg:
    version: "1.0.0"
  ## WiFi on ESP32-P4 (via coprocessor, e.g. ESP32-C6)
  espressif/esp_wifi_remote:
    version: "1.4.2"
  ## Protobuf encoding for telemetry pipeline
  livekit/nanopb:
    version: "^0.4.9"
  ## Detection model (phase 1: cat_detect for co-residency test)
  espressif/cat_detect:
    version: "*"
    override_path: "../docs/esp-dl/models/cat_detect"
```

- [ ] **Step 1.2: Add `cat_detect` to `PRIV_REQUIRES` in `main/CMakeLists.txt`**

Find the `PRIV_REQUIRES` block and append `cat_detect` to the last line. Before:

```cmake
    PRIV_REQUIRES
        spi_flash esp_timer nvs_flash driver
        esp_video esp_wifi esp_wifi_remote esp_netif
        esp_http_server esp_driver_jpeg esp_driver_ppa
        livekit__nanopb
```

After:

```cmake
    PRIV_REQUIRES
        spi_flash esp_timer nvs_flash driver
        esp_video esp_wifi esp_wifi_remote esp_netif
        esp_http_server esp_driver_jpeg esp_driver_ppa
        livekit__nanopb cat_detect
```

- [ ] **Step 1.3: Append three lines to `sdkconfig.defaults`**

Add at the bottom of the file:

```
CONFIG_FLASH_ESPDET_PICO_224_224_CAT=y
CONFIG_CAT_DETECT_MODEL_IN_FLASH_PARTITION=y
# CONFIG_CAT_DETECT_MODEL_IN_FLASH_RODATA is not set
```

- [ ] **Step 1.4: Add `cat_det` partition to `partitions.csv`**

Current file:

```
# Name,   Type, SubType,  Offset,   Size
nvs,      data, nvs,      0x9000,   0x6000
phy_init, data, phy,      0xF000,   0x1000
factory,  app,  factory,  0x10000,  2048K
```

Append one line — **the name MUST be exactly `cat_det`**, hardcoded in [docs/esp-dl/models/cat_detect/cat_detect.cpp](../../docs/esp-dl/models/cat_detect/cat_detect.cpp):

```
# Name,   Type, SubType,  Offset,   Size
nvs,      data, nvs,      0x9000,   0x6000
phy_init, data, phy,      0xF000,   0x1000
factory,  app,  factory,  0x10000,  2048K
cat_det,  data, spiffs,   ,         1M
```

The empty offset field (`,`) means ESP-IDF auto-places this partition immediately after `factory` — no manual offset arithmetic needed.

- [ ] **Step 1.5: Reconfigure and verify component resolution**

Run:
```bash
cd /workspaces/BoatEspP4
rm -rf build
idf.py reconfigure 2>&1 | tee /tmp/tier0_reconfigure.log
```

Expected in the log (search for `cat_detect`):
```
Solved dependencies: ... espressif/cat_detect (...override_path: ../docs/esp-dl/models/cat_detect) ...
```

Pass criteria:
- The log mentions `cat_detect`.
- It explicitly shows `override_path`, NOT a registry version like `(version: 0.2.1)` by itself.

If the log shows a registry version without override_path, the manifest syntax is wrong. Recheck step 1.1 indentation and spelling.

**EXPECTED CONFLICT — esp_new_jpeg version:** the reconfigure is very likely to fail with a dependency resolution error mentioning `espressif/esp_new_jpeg`. Boat's `main/idf_component.yml` pins `1.0.0`; esp-dl 3.3.0 (transitively via `cat_detect` → `esp-dl`) requires `^0.6.1` = `>=0.6.1, <0.7.0`. These are incompatible.

Fix: widen esp-dl's constraint in-repo (it's an override_path so we own the copy). Edit [docs/esp-dl/esp-dl/idf_component.yml](../../docs/esp-dl/esp-dl/idf_component.yml) and change:

```yaml
  espressif/esp_new_jpeg: "^0.6.1"
```

to:

```yaml
  espressif/esp_new_jpeg: ">=0.6.1"
```

This unpins the upper bound so the boat's `1.0.0` satisfies it. Rerun `idf.py reconfigure`. If it still fails with a different conflict, surface the exact error before continuing — do not iterate blind. Record the vendored manifest edit in the Task 1.7 commit.

- [ ] **Step 1.6: Build and verify partition table**

Run:
```bash
idf.py build 2>&1 | tee /tmp/tier0_build.log
idf.py partition-table 2>&1 | tee /tmp/tier0_parts.log
```

Pass criteria (build log):
- Link succeeds: `Generating binary image from built executable` appears.
- Final section near the end shows app binary size `esp32p4.app.bin binary size ...` — record this number. It should be well under 2048KB (the model is NOT in the app binary).

Pass criteria (partition log):
- Output contains a row for `cat_det` with subtype `spiffs` and size `0x100000` (1MB).
- `factory` row shows size `0x200000` (2048KB) unchanged.

If the app binary exceeds 2000KB: stop and investigate what got pulled in transitively. Do NOT bump the partition as a reflex — the spec explicitly calls this a signal, not a routine fix.

- [ ] **Step 1.7: Commit**

```bash
cd /workspaces/BoatEspP4
git add main/idf_component.yml main/CMakeLists.txt sdkconfig.defaults partitions.csv
# If you applied the esp_new_jpeg widening fix from Step 1.5, also stage it:
#   git add docs/esp-dl/esp-dl/idf_component.yml
git status  # verify the set of staged files matches what you intend
git commit -m "$(cat <<'EOF'
feat(detect): Tier 0 plumbing — add cat_detect dep and cat_det partition

Wire up the espressif/cat_detect managed component via override_path
to the in-repo model at docs/esp-dl/models/cat_detect, add a 1MB
cat_det data partition (name is hardcoded in cat_detect.cpp), and
switch to CAT_DETECT_MODEL_IN_FLASH_PARTITION so the model lives in
its own partition instead of app rodata. Factory partition size
unchanged. No detection code yet — this commit is the build-plumbing
gate per the design spec's Tier 0 bisectability principle.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 2: Embed the test JPEG via configure_file

**Rationale:** `EMBED_FILES` needs a path relative to the component directory. Rather than use an absolute path (which would mangle the generated binary symbol name across machines), copy `tools/esp-detection/espdet.jpg` into `main/` at CMake configure time with a deterministic basename. Resulting symbol: `_binary_espdet_jpg_start`.

**Files:**
- Modify: `main/CMakeLists.txt` (add `configure_file` + `EMBED_FILES`)
- Modify: `.gitignore` (exclude the build-artifact copy)

- [ ] **Step 2.1: Add `configure_file` and `EMBED_FILES` to `main/CMakeLists.txt`**

Before the `idf_component_register(...)` call, add:

```cmake
# Copy the canonical test JPEG into the component directory so that
# EMBED_FILES can reference it by basename (absolute paths mangle
# symbol names across checkouts). Source of truth lives in
# tools/esp-detection/; this copy is gitignored as a build artifact.
configure_file(
    "${CMAKE_SOURCE_DIR}/tools/esp-detection/espdet.jpg"
    "${CMAKE_CURRENT_LIST_DIR}/espdet.jpg"
    COPYONLY)
```

Inside `idf_component_register(...)`, add `EMBED_FILES "espdet.jpg"` on a new line after `EMBED_TXTFILES "dashboard.html"`:

Before:
```cmake
    INCLUDE_DIRS "include" "drivers" "." "proto"
    EMBED_TXTFILES "dashboard.html")
```

After:
```cmake
    INCLUDE_DIRS "include" "drivers" "." "proto"
    EMBED_TXTFILES "dashboard.html"
    EMBED_FILES "espdet.jpg")
```

- [ ] **Step 2.2: Add the build-artifact copy to `.gitignore`**

Append to the root `.gitignore`:

```
# Build artifact: configure_file copy of tools/esp-detection/espdet.jpg
main/espdet.jpg
```

- [ ] **Step 2.3: Rebuild and verify the embedded symbol**

Run:
```bash
cd /workspaces/BoatEspP4
idf.py reconfigure
idf.py build 2>&1 | tail -20
ls -l main/espdet.jpg
xtensa-esp32p4-elf-nm build/esp-idf/main/libmain.a 2>/dev/null | grep espdet_jpg || \
    riscv32-esp-elf-nm build/BoatMain.elf 2>/dev/null | grep espdet_jpg
```

**Note:** ESP32-P4 uses RISC-V, so the correct tool is `riscv32-esp-elf-nm`. If the project uses a different .elf name, adjust — common names are `BoatMain.elf`, `boat.elf`, or `project.elf`. Find it via `ls build/*.elf`.

Pass criteria:
- `main/espdet.jpg` exists and matches size of `tools/esp-detection/espdet.jpg` (98890 bytes).
- `nm` output contains both `_binary_espdet_jpg_start` and `_binary_espdet_jpg_end` symbols (type `D` or `R`).

If symbols are missing: the EMBED_FILES line is syntactically wrong or the configure_file copy didn't happen. Run `stat main/espdet.jpg` — if "no such file," configure_file failed; re-check the source path.

- [ ] **Step 2.4: Commit**

```bash
cd /workspaces/BoatEspP4
git add main/CMakeLists.txt .gitignore
git commit -m "$(cat <<'EOF'
feat(detect): embed tools/esp-detection/espdet.jpg as test input

CMake configure_file copies the canonical test JPEG from tools/ into
main/ at configure time so EMBED_FILES can reference it by basename,
producing a deterministic _binary_espdet_jpg_start symbol across
checkouts. The main/espdet.jpg copy is gitignored as a build artifact.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 3: detection.h — C API header

**Rationale:** Define the C API that http_server.c will call. All types and function declarations live here; implementation is Task 4. Putting the header first lets us commit a reviewable interface before 200 lines of C++.

**Files:**
- Create: `main/detection/detection.h`

- [ ] **Step 3.1: Create `main/detection/` directory and the header file**

Create `main/detection/detection.h` with this exact content:

```c
// SPDX-License-Identifier: MIT
//
// detection.h — C API wrapper around the cat_detect C++ model.
//
// Phase 1 of the /detect endpoint integration. This header is included
// ONLY by http_server.c. detection.cpp holds all C++ state and symbols;
// nothing C++-flavored crosses this interface.
//
// See docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Failure stages for graceful errors returned by detection_run_on_embedded.
 *
 * NOTE: "model_init" is intentionally NOT a stage. C++ exceptions are
 * disabled (CONFIG_COMPILER_CXX_EXCEPTIONS is not set), so model-load
 * failures panic and reboot rather than returning a catchable error.
 * The serial log is the bisectability layer for init crashes.
 */
typedef enum {
    DETECTION_STAGE_OK        = 0,
    DETECTION_STAGE_DECODE    = 1,  /* sw_decode_jpeg returned NULL */
    DETECTION_STAGE_INFERENCE = 2,  /* detect->run() returned malformed result */
    DETECTION_STAGE_MARSHAL   = 3,  /* JSON builder OOM / snprintf error */
} detection_stage_t;

/**
 * Result of a single detection run. Ownership: caller owns out->json
 * and must call detection_response_free() when done.
 */
typedef struct {
    bool              ok;             /* true if inference succeeded */
    int               http_status;    /* 200 on success, 500 on graceful failure */
    detection_stage_t failed_stage;   /* DETECTION_STAGE_OK when ok == true */
    char             *json;           /* malloc'd JSON body (caller frees) */
    size_t            json_len;       /* length of json, excluding NUL */
} detection_response_t;

/**
 * Run ESPDet-Pico cat detection on the embedded espdet.jpg.
 *
 * Lazy initialization: the first call allocates CatDetect, triggers
 * model weight load from the cat_det flash partition, and decodes the
 * embedded JPEG. Subsequent calls reuse all three.
 *
 * LOAD-BEARING INVARIANT: only callable from the esp_http_server worker
 * task. The internal static state is not mutex-protected; it relies on
 * the single-threaded httpd task for serialization.
 *
 * Always returns ESP_OK — failure information is encoded in *out
 * (out->ok, out->failed_stage, out->http_status). The out->json is
 * always set (either the success body or an error body).
 *
 * Caller must call detection_response_free(out) when done.
 */
esp_err_t detection_run_on_embedded(detection_response_t *out);

/**
 * Frees out->json. Safe to call with NULL or a zeroed struct.
 * Sets out->json to NULL and out->json_len to 0.
 */
void detection_response_free(detection_response_t *out);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 3.2: Commit**

```bash
cd /workspaces/BoatEspP4
mkdir -p main/detection
git add main/detection/detection.h
git commit -m "$(cat <<'EOF'
feat(detect): add detection.h C API for the /detect endpoint

Declares the extern C interface http_server.c will call:
detection_run_on_embedded() + detection_response_free() + the
detection_stage_t enum for graceful failure reporting. Implementation
follows in the next commit. model_init is intentionally not a stage
because C++ exceptions are disabled — init crashes go to serial log.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 4: detection.cpp — C++ implementation

**Rationale:** The one C++ file in the firmware. Owns `CatDetect*`, the decoded image, and all JSON marshalling. Uses range-for iteration over `std::list<dl::detect::result_t>&` per the template pattern.

**Files:**
- Create: `main/detection/detection.cpp`
- Modify: `main/CMakeLists.txt` (add to SRCS, add detection to INCLUDE_DIRS)

- [ ] **Step 4.1: Create `main/detection/detection.cpp`**

Create with this exact content:

```cpp
// SPDX-License-Identifier: MIT
//
// detection.cpp — ESPDet-Pico cat_detect wrapper with JSON marshalling.
//
// =====================================================================
// LOAD-BEARING INVARIANT
// =====================================================================
// All callers of detection_run_on_embedded() MUST come from the
// esp_http_server worker task. The static globals below are accessed
// without a mutex; the httpd task's single-threaded handler dispatch
// provides the serialization guarantee.
//
// If you add a second caller (camera timer, CLI command, WS command
// handler, a new task), you MUST add a mutex around s_detect,
// s_decoded_img, and s_decode_done BEFORE that second caller runs.
// =====================================================================
//
// See docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md

#include "detection/detection.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <list>
#include <new>   // std::nothrow

#include "cat_detect.hpp"
#include "dl_image_jpeg.hpp"
#include "dl_detect_define.hpp"

#include "esp_log.h"
#include "esp_timer.h"

/* Embedded test JPEG (main/espdet.jpg via EMBED_FILES) */
extern const uint8_t espdet_jpg_start[] asm("_binary_espdet_jpg_start");
extern const uint8_t espdet_jpg_end[]   asm("_binary_espdet_jpg_end");

static const char *TAG = "DETECT";

/* Lazy-initialized process-lifetime singletons. See invariant above. */
static CatDetect         *s_detect      = nullptr;
static dl::image::img_t   s_decoded_img = {};
static bool               s_decode_done = false;

// ---------------------------------------------------------------------
// Minimum-viable JSON builder (no cJSON dep).
// Expanding buffer with realloc; caller frees via detection_response_free.
// ---------------------------------------------------------------------
struct jsonbuf_t {
    char   *buf;
    size_t  len;
    size_t  cap;
};

static bool jbuf_ensure(jsonbuf_t &j, size_t extra)
{
    if (j.len + extra + 1 <= j.cap) return true;
    size_t new_cap = j.cap ? j.cap : 256;
    while (new_cap < j.len + extra + 1) new_cap *= 2;
    char *nb = (char *)realloc(j.buf, new_cap);
    if (!nb) return false;
    j.buf = nb;
    j.cap = new_cap;
    return true;
}

static bool jbuf_appendf(jsonbuf_t &j, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int needed = vsnprintf(nullptr, 0, fmt, ap);
    va_end(ap);
    if (needed < 0) { va_end(ap_copy); return false; }
    if (!jbuf_ensure(j, (size_t)needed)) { va_end(ap_copy); return false; }
    vsnprintf(j.buf + j.len, j.cap - j.len, fmt, ap_copy);
    va_end(ap_copy);
    j.len += (size_t)needed;
    return true;
}

// ---------------------------------------------------------------------
// Error JSON builder (short, fixed shape).
// ---------------------------------------------------------------------
static char *build_error_json(detection_stage_t stage,
                              const char       *msg,
                              size_t           *out_len)
{
    const char *stage_str = "unknown";
    switch (stage) {
        case DETECTION_STAGE_DECODE:    stage_str = "decode";    break;
        case DETECTION_STAGE_INFERENCE: stage_str = "inference"; break;
        case DETECTION_STAGE_MARSHAL:   stage_str = "marshal";   break;
        case DETECTION_STAGE_OK:
        default: break;
    }
    jsonbuf_t j = {};
    if (!jbuf_appendf(j, "{\"ok\":false,\"stage\":\"%s\",\"error\":\"%s\"}",
                      stage_str, msg)) {
        free(j.buf);
        return nullptr;
    }
    *out_len = j.len;
    return j.buf;
}

// ---------------------------------------------------------------------
// Success JSON builder.
// ---------------------------------------------------------------------
static char *build_success_json(int img_w, int img_h, int inference_ms,
                                const std::list<dl::detect::result_t> &results,
                                size_t *out_len)
{
    jsonbuf_t j = {};
    if (!jbuf_appendf(j,
            "{\"ok\":true,"
            "\"model\":\"espdet_pico_224_224_cat\","
            "\"source\":\"embedded:espdet.jpg\","
            "\"image\":{\"w\":%d,\"h\":%d},"
            "\"inference_ms\":%d,"
            "\"detections\":[",
            img_w, img_h, inference_ms)) {
        free(j.buf);
        return nullptr;
    }

    bool first = true;
    for (const auto &res : results) {
        // result_t::box is std::vector<int> of length 4: [x1, y1, x2, y2]
        if (res.box.size() < 4) continue;  // defensive: skip malformed
        if (!jbuf_appendf(j,
                "%s{\"category\":%d,\"score\":%.3f,"
                "\"box\":[%d,%d,%d,%d]}",
                first ? "" : ",",
                res.category,
                (double)res.score,
                res.box[0], res.box[1], res.box[2], res.box[3])) {
            free(j.buf);
            return nullptr;
        }
        first = false;
    }

    if (!jbuf_appendf(j, "]}")) {
        free(j.buf);
        return nullptr;
    }

    *out_len = j.len;
    return j.buf;
}

// ---------------------------------------------------------------------
// Public C API
// ---------------------------------------------------------------------
extern "C" esp_err_t detection_run_on_embedded(detection_response_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    // Lazy instantiate. CatDetect() default ctor sets lazy_load=true,
    // so the first run() call below triggers load_model() which reads
    // ~487KB from the cat_det flash partition and allocates PSRAM
    // working buffers. Expect first-call latency of several hundred ms.
    if (!s_detect) {
        ESP_LOGI(TAG, "first /detect hit — instantiating CatDetect");
        s_detect = new (std::nothrow) CatDetect();
        if (!s_detect) {
            out->ok = false;
            out->http_status = 500;
            out->failed_stage = DETECTION_STAGE_INFERENCE;
            out->json = build_error_json(DETECTION_STAGE_INFERENCE,
                                         "CatDetect allocation failed",
                                         &out->json_len);
            return ESP_OK;
        }
    }

    // Decode embedded JPEG once, keep decoded buffer alive for process life.
    if (!s_decode_done) {
        dl::image::jpeg_img_t jpeg_img = {
            .data     = (void *)espdet_jpg_start,
            .data_len = (size_t)(espdet_jpg_end - espdet_jpg_start),
        };
        s_decoded_img = dl::image::sw_decode_jpeg(
            jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);

        if (!s_decoded_img.data) {
            out->ok = false;
            out->http_status = 500;
            out->failed_stage = DETECTION_STAGE_DECODE;
            out->json = build_error_json(DETECTION_STAGE_DECODE,
                                         "sw_decode_jpeg returned NULL",
                                         &out->json_len);
            return ESP_OK;
        }
        s_decode_done = true;
        ESP_LOGI(TAG, "decoded espdet.jpg: %dx%d",
                 (int)s_decoded_img.width, (int)s_decoded_img.height);
    }

    // Run inference. This is the only line that should dominate latency.
    int64_t t0 = esp_timer_get_time();
    auto &results = s_detect->run(s_decoded_img);
    int64_t t1 = esp_timer_get_time();
    int inference_ms = (int)((t1 - t0) / 1000);

    ESP_LOGI(TAG, "/detect: %d detections in %d ms",
             (int)results.size(), inference_ms);

    // Marshal to JSON.
    size_t json_len = 0;
    char *json = build_success_json((int)s_decoded_img.width,
                                    (int)s_decoded_img.height,
                                    inference_ms, results, &json_len);
    if (!json) {
        out->ok = false;
        out->http_status = 500;
        out->failed_stage = DETECTION_STAGE_MARSHAL;
        out->json = build_error_json(DETECTION_STAGE_MARSHAL,
                                     "JSON marshal OOM",
                                     &out->json_len);
        return ESP_OK;
    }

    out->ok = true;
    out->http_status = 200;
    out->failed_stage = DETECTION_STAGE_OK;
    out->json = json;
    out->json_len = json_len;
    return ESP_OK;
}

extern "C" void detection_response_free(detection_response_t *out)
{
    if (!out) return;
    if (out->json) {
        free(out->json);
        out->json = nullptr;
    }
    out->json_len = 0;
}
```

- [ ] **Step 4.2: Add `detection.cpp` to `main/CMakeLists.txt` SRCS and include dirs**

Find the `SRCS` block and append `"detection/detection.cpp"`. Before:

```cmake
        "transports/ws_transport.c"
        "proto/boat.pb.c"
    PRIV_REQUIRES
```

After:

```cmake
        "transports/ws_transport.c"
        "proto/boat.pb.c"
        "detection/detection.cpp"
    PRIV_REQUIRES
```

Find the `INCLUDE_DIRS` line and add `"detection"`. Before:

```cmake
    INCLUDE_DIRS "include" "drivers" "." "proto"
```

After:

```cmake
    INCLUDE_DIRS "include" "drivers" "." "proto" "detection"
```

- [ ] **Step 4.3: Build and verify detection.cpp compiles**

Run:
```bash
cd /workspaces/BoatEspP4
idf.py build 2>&1 | tee /tmp/task4_build.log
```

Pass criteria:
- Line matching `Compiling ... detection.cpp.obj` appears in the log.
- Link succeeds; no unresolved symbols related to `CatDetect`, `sw_decode_jpeg`, or `_binary_espdet_jpg_start`.
- Final binary size reported.

Common failure: `undefined reference to 'CatDetect::CatDetect(...)'` → `cat_detect` missing from `PRIV_REQUIRES` (re-check Task 1.2).

Common failure: `fatal error: cat_detect.hpp: No such file or directory` → the cat_detect component path is wrong in `idf_component.yml` override_path (re-check Task 1.1).

- [ ] **Step 4.4: Commit**

```bash
cd /workspaces/BoatEspP4
git add main/detection/detection.cpp main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat(detect): detection.cpp — lazy CatDetect wrapper with JSON marshalling

Implements detection_run_on_embedded() per the C API in detection.h.
Lazy-instantiates CatDetect on first call (allocates in PSRAM, loads
model weights from the cat_det flash partition), decodes the embedded
JPEG once, then runs inference and serializes std::list<result_t> to
a hand-built JSON body. Graceful errors return an error JSON with a
stage tag; hard init crashes panic (C++ exceptions disabled by design).

The single-thread httpd invariant is documented as a load-bearing
comment at the top of the file — any future caller outside the httpd
worker task must add a mutex around the static state.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 5: http_server.c — /detect handler

**Rationale:** Register the URI, bump stack size for the inference call path, and bump `max_uri_handlers` so the new registration doesn't silently fail.

**Files:**
- Modify: `main/http_server.c`

- [ ] **Step 5.1: Add `#include` at the top of `main/http_server.c`**

After the existing includes (around line 7), add:

```c
#include "detection/detection.h"
```

- [ ] **Step 5.2: Bump `stack_size` and `max_uri_handlers` in `http_server_start()`**

Find these two lines in `http_server_start()`:

```c
    cfg.stack_size       = 8192;
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 4;
```

Change to:

```c
    cfg.stack_size       = 16384;  /* +8KB for C++ inference call path (NMS, preprocessor intermediates) */
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 5;      /* bumped for /detect */
```

- [ ] **Step 5.3: Add `detect_handler` function**

Add this function immediately AFTER the existing `dashboard_handler` function (around line 22):

```c
static esp_err_t detect_handler(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    if (ret != ESP_OK) {
        /* detection_run_on_embedded always returns ESP_OK in phase 1,
         * but be defensive in case that changes. */
        httpd_resp_send_500(req);
        return ret;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req,
        out.http_status == 200 ? "200 OK" : "500 Internal Server Error");

    if (out.json && out.json_len > 0) {
        httpd_resp_send(req, out.json, out.json_len);
    } else {
        httpd_resp_send_500(req);
    }

    detection_response_free(&out);
    return ESP_OK;
}
```

- [ ] **Step 5.4: Register `/detect` URI in `http_server_start()`**

Find the existing `dashboard_uri` registration block (around line 44-51):

```c
    const httpd_uri_t dashboard_uri = {
        .uri = "/", .method = HTTP_GET,
        .handler = dashboard_handler, .user_ctx = NULL,
    };
    ret = httpd_register_uri_handler(server, &dashboard_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register / failed: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }
```

Add immediately after the closing brace of that `if (ret != ESP_OK) { ... }` block:

```c
    const httpd_uri_t detect_uri = {
        .uri = "/detect", .method = HTTP_GET,
        .handler = detect_handler, .user_ctx = NULL,
    };
    ret = httpd_register_uri_handler(server, &detect_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register /detect failed: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }
```

- [ ] **Step 5.5: Update the ready log line**

Find the final `ESP_LOGI(TAG, "HTTP server ready ...")` line:

```c
    ESP_LOGI(TAG, "HTTP server ready on port %d: / (dashboard), /ws (protobuf)",
             CONFIG_HTTP_API_PORT);
```

Change to:

```c
    ESP_LOGI(TAG, "HTTP server ready on port %d: / (dashboard), /ws (protobuf), /detect (cat)",
             CONFIG_HTTP_API_PORT);
```

- [ ] **Step 5.6: Build and verify**

Run:
```bash
cd /workspaces/BoatEspP4
idf.py build 2>&1 | tail -20
```

Pass criteria:
- Build succeeds.
- No warnings about undefined `detection_run_on_embedded` or `detection_response_free` (those would mean the header include path is wrong — check Task 4.2 include dir bump).

- [ ] **Step 5.7: Commit**

```bash
cd /workspaces/BoatEspP4
git add main/http_server.c
git commit -m "$(cat <<'EOF'
feat(detect): register GET /detect URI handler on the boat httpd

Adds detect_handler() calling detection_run_on_embedded() and sending
the JSON body with the appropriate status. Bumps httpd stack_size
8192 -> 16384 (stack headroom for the C++ inference call path — NMS,
ImagePreprocessor intermediates, std::vector manipulations) and
max_uri_handlers 4 -> 5 so the new registration doesn't silently fail.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 6: Tier 1 — Smoke test (flash and curl)

**Rationale:** Manual verification that the endpoint works end-to-end on real hardware before automating. If this fails, the bisectability ladder in the spec tells us which layer to investigate.

**Files:** None (flash + curl verification)

- [ ] **Step 6.1: Full flash (NOT app-flash — populates cat_det partition)**

```bash
cd /workspaces/BoatEspP4
idf.py flash 2>&1 | tee /tmp/tier1_flash.log
```

Pass criteria:
- The flash log shows TWO partition writes: one at `0x10000` (factory, the app binary) and one at an auto-computed offset (cat_det, the ~500KB packed model).
- `Hard resetting via RTS pin...` appears at the end.

**Critical:** Do NOT use `idf.py app-flash`. It only writes the factory partition, leaving `cat_det` stale or empty, and `/detect` will crash on first hit with an obscure model-load failure.

- [ ] **Step 6.2: Open monitor in a separate terminal**

In a second terminal:

```bash
cd /workspaces/BoatEspP4
idf.py monitor
```

Wait for the boot sequence to complete. Expected log lines:
```
I (xxxx) MAIN: === SYSTEM BOOT ===
...
I (xxxx) HTTP_SERVER: HTTP server ready on port 80: / (dashboard), /ws (protobuf), /detect (cat)
I (xxxx) MAIN: System running.
```

Note the boat's IP address from the `wifi_manager` or `esp_netif` log lines (`sta ip: 192.168.x.x`).

- [ ] **Step 6.3: Smoke test — first curl (first-run warmup)**

In a third terminal:

```bash
BOAT_IP=192.168.x.x   # fill in from Step 6.2
curl -sS -w '\nHTTP %{http_code} in %{time_total}s\n' http://$BOAT_IP/detect | tee /tmp/tier1_first.json
```

Pass criteria:
- HTTP 200.
- JSON body has `"ok":true`, `"model":"espdet_pico_224_224_cat"`, `"source":"embedded:espdet.jpg"`.
- `detections` array has at least 1 entry.
- First entry has `"category":0` (cat class) and `"score"` > 0.6.
- The serial monitor shows:
  ```
  I (...) DETECT: first /detect hit — instantiating CatDetect
  I (...) DETECT: decoded espdet.jpg: <w>x<h>
  I (...) DETECT: /detect: N detections in <ms> ms
  ```
- Total time may be several hundred ms on the first call (model load).

If the serial log instead shows a panic / guru meditation / reboot: the cat_det partition may be stale — re-run `idf.py flash` (not app-flash) and retry.

- [ ] **Step 6.4: Second curl (steady state — verify baseline latency)**

```bash
curl -sS -w '\nHTTP %{http_code} in %{time_total}s\n' http://$BOAT_IP/detect | tee /tmp/tier1_second.json
```

Pass criteria:
- HTTP 200, same JSON shape as first call.
- The serial monitor's `inference in N ms` is in [40, 70] ms. Baseline is 51ms per the memory.
- No `first /detect hit` log line this time (the instantiation happened on the first call).

- [ ] **Step 6.5: Verify dashboard and WS still work**

```bash
curl -sS http://$BOAT_IP/ -o /tmp/dashboard_after.html -w 'HTTP %{http_code} size %{size_download}\n'
```

Pass criteria:
- HTTP 200, size equal to `main/dashboard.html` size.
- Open `http://$BOAT_IP/` in a browser. Dashboard should load and WS sensor data should update in real time (pitch/roll/heading, ToF grids).

- [ ] **Step 6.6: Commit a note in the spec if Tier 1 passed**

No code change. Just record the Tier 1 result as a note. Skip this step if nothing worth recording.

If everything passed, add a brief note to the spec's test plan section (or to a new `docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-tier1-results.md` if you prefer a separate file). One paragraph is enough:

```markdown
**Tier 1 results (YYYY-MM-DD):** Smoke test passed on real hardware.
First-call latency: <X>ms (model load). Steady-state inference_ms:
<Y>ms (baseline 51). Detection: 1 cat at score <Z>. Dashboard and WS
unaffected during and after the curl.
```

---

## Task 7: tools/test_detect_coresidency.py — Tier 2 harness

**Rationale:** Automate the co-residency assertion: 100 `/detect` hits while WS sensor broadcast and MJPEG stream are active, verify zero reboots, IMU heading continuous, WS cadence unbroken.

**Files:**
- Create: `tools/test_detect_coresidency.py`

- [ ] **Step 7.1: Create the harness script**

Create `tools/test_detect_coresidency.py` with this exact content:

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Tier 2 co-residency stability test for the /detect endpoint.
#
# Runs 100 GET /detect calls while a WebSocket client stays connected
# to /ws and asserts:
#   - zero HTTP 500s
#   - WS sensor frame cadence stays bounded (no gap > 100ms)
#   - inference_ms p50 <= 60, p95 <= 70
#   - IMU heading continuous (no jump > 3 degrees between consecutive WS frames)
#
# Assumes the boat is static (on the bench) during the run.
#
# Usage:
#   python3 tools/test_detect_coresidency.py --host 192.168.1.42
#
# Deps:
#   pip install requests websocket-client

import argparse
import json
import statistics
import sys
import threading
import time

import requests
import websocket  # pip install websocket-client


class WSCollector:
    """Connects to /ws and records (timestamp, heading_deg) for each frame."""

    def __init__(self, url: str):
        self.url = url
        self.samples = []  # list of (t_sec, heading_deg_or_None)
        self.errors = []
        self._ws = None
        self._thread = None
        self._stop = threading.Event()

    def _on_message(self, ws, message):
        t = time.monotonic()
        heading = None
        # The boat sends protobuf over the WS. For this test we don't
        # strictly need to decode it — cadence is the primary signal.
        # Heading decode is best-effort: if we can parse it, use it;
        # otherwise skip the heading assertion for that sample.
        try:
            if isinstance(message, (bytes, bytearray)):
                # Best-effort heading extraction is left to the user;
                # phase 1 asserts on cadence only if heading is unavailable.
                heading = None
            else:
                data = json.loads(message)
                heading = data.get("heading")
        except Exception:
            heading = None
        self.samples.append((t, heading))

    def _on_error(self, ws, error):
        self.errors.append(f"ws error: {error}")

    def _run(self):
        self._ws = websocket.WebSocketApp(
            self.url,
            on_message=self._on_message,
            on_error=self._on_error,
        )
        self._ws.run_forever()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        time.sleep(1.0)  # allow connect

    def stop(self):
        if self._ws:
            self._ws.close()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def cadence_gaps_ms(self, t_start: float, t_end: float):
        """Return list of inter-frame gaps (in ms) within [t_start, t_end]."""
        window = [s for s in self.samples if t_start <= s[0] <= t_end]
        if len(window) < 2:
            return []
        return [(window[i+1][0] - window[i][0]) * 1000.0
                for i in range(len(window) - 1)]

    def heading_deltas(self, t_start: float, t_end: float):
        """Return list of abs heading deltas (deg) within the window."""
        window = [s for s in self.samples
                  if t_start <= s[0] <= t_end and s[1] is not None]
        if len(window) < 2:
            return []
        return [abs(window[i+1][1] - window[i][1])
                for i in range(len(window) - 1)]


def percentile(xs, p):
    if not xs:
        return float("nan")
    xs_sorted = sorted(xs)
    k = int(round((p / 100.0) * (len(xs_sorted) - 1)))
    return xs_sorted[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="boat IP (e.g. 192.168.1.42)")
    ap.add_argument("--rounds", type=int, default=100)
    ap.add_argument("--http-port", type=int, default=80)
    args = ap.parse_args()

    http_base = f"http://{args.host}:{args.http_port}"
    ws_url    = f"ws://{args.host}:{args.http_port}/ws"

    print(f"== Tier 2 co-residency test ==")
    print(f"target        : {http_base}")
    print(f"ws            : {ws_url}")
    print(f"rounds        : {args.rounds}")
    print()

    ws = WSCollector(ws_url)
    ws.start()

    if not ws._ws or not ws.samples:
        time.sleep(2.0)
    if not ws.samples:
        print("WARN: no WS samples after 3s — continuing anyway", file=sys.stderr)

    # 30s baseline window (WS cadence and heading stability before load)
    print("collecting 30s baseline ...")
    t_base_start = time.monotonic()
    time.sleep(30.0)
    t_base_end = time.monotonic()
    base_gaps = ws.cadence_gaps_ms(t_base_start, t_base_end)
    base_mean_gap = statistics.mean(base_gaps) if base_gaps else 0.0
    base_heading_deltas = ws.heading_deltas(t_base_start, t_base_end)
    print(f"baseline  : n={len(base_gaps)} gaps, mean={base_mean_gap:.1f}ms, "
          f"max={max(base_gaps) if base_gaps else 0.0:.1f}ms")

    # Loop: N /detect hits
    inference_ms = []
    http_500_count = 0
    t_loop_start = time.monotonic()
    for i in range(args.rounds):
        try:
            r = requests.get(f"{http_base}/detect", timeout=10.0)
            if r.status_code != 200:
                http_500_count += 1
                print(f"  [{i+1}] HTTP {r.status_code}")
                continue
            body = r.json()
            inference_ms.append(body.get("inference_ms", -1))
            if (i + 1) % 10 == 0:
                p50 = percentile(inference_ms, 50)
                p95 = percentile(inference_ms, 95)
                print(f"  [{i+1}/{args.rounds}] p50={p50:.0f}ms p95={p95:.0f}ms")
        except Exception as e:
            http_500_count += 1
            print(f"  [{i+1}] ERROR: {e}")
    t_loop_end = time.monotonic()

    # 5s settle window
    print("5s settle ...")
    time.sleep(5.0)

    loop_gaps = ws.cadence_gaps_ms(t_loop_start, t_loop_end)
    loop_heading_deltas = ws.heading_deltas(t_loop_start, t_loop_end)
    max_gap = max(loop_gaps) if loop_gaps else 0.0
    max_heading_delta = max(loop_heading_deltas) if loop_heading_deltas else 0.0

    ws.stop()

    # ---- Assertions ----
    failures = []

    if http_500_count > 0:
        failures.append(f"http_500_count={http_500_count} (expected 0)")

    if not inference_ms:
        failures.append("inference_ms list empty — all requests failed")
    else:
        p50 = percentile(inference_ms, 50)
        p95 = percentile(inference_ms, 95)
        if p50 > 60:
            failures.append(f"inference_ms p50={p50:.0f} > 60")
        if p95 > 70:
            failures.append(f"inference_ms p95={p95:.0f} > 70")

    if loop_gaps and max_gap > 100.0:
        failures.append(f"ws cadence gap max={max_gap:.1f}ms > 100ms")

    if loop_heading_deltas and max_heading_delta > 3.0:
        failures.append(f"imu heading delta max={max_heading_delta:.2f}deg > 3deg")

    # ---- Report ----
    print()
    print("== RESULTS ==")
    print(f"http_500        : {http_500_count}")
    if inference_ms:
        print(f"inference_ms    : p50={percentile(inference_ms, 50):.0f} "
              f"p95={percentile(inference_ms, 95):.0f} "
              f"max={max(inference_ms):.0f} n={len(inference_ms)}")
    print(f"ws gaps (loop)  : max={max_gap:.1f}ms  mean baseline={base_mean_gap:.1f}ms")
    if loop_heading_deltas:
        print(f"heading delta   : max={max_heading_delta:.2f}deg")
    else:
        print(f"heading delta   : (not checked — no heading in WS payload)")
    print()

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)

    print("PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 7.2: Verify the script is syntactically valid**

```bash
cd /workspaces/BoatEspP4
python3 -c "import ast; ast.parse(open('tools/test_detect_coresidency.py').read()); print('OK')"
```

Expected: `OK`

- [ ] **Step 7.3: Commit**

```bash
cd /workspaces/BoatEspP4
git add tools/test_detect_coresidency.py
git commit -m "$(cat <<'EOF'
test(detect): add Tier 2 co-residency harness

Runs 100 GET /detect hits while holding a WS connection open,
asserts zero HTTP 500s, inference_ms p50<=60 / p95<=70, WS cadence
gap <=100ms, and (best-effort) IMU heading continuous. Assumes the
boat is static during the run. Heading check is skipped when the WS
payload is protobuf-binary and the client can't decode it cheaply —
phase 2 can wire in the real proto parser.

Spec: docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md
EOF
)"
```

---

## Task 8: Tier 2 — Execute the co-residency harness

**Rationale:** Run the harness end-to-end and confirm all criteria pass. This is the gating test for phase 1 completion.

**Files:** None (execution only, with notes captured if anything needs investigation)

- [ ] **Step 8.1: Ensure dashboard is OPEN in a browser during the test**

Open `http://$BOAT_IP/` in a browser tab and verify the dashboard shows live-updating pitch/roll/heading/ToF. Leave the tab open for the entire test run — this is what "co-residency" means: WS broadcast to a real client while `/detect` runs.

- [ ] **Step 8.2: Start the harness**

```bash
cd /workspaces/BoatEspP4
BOAT_IP=192.168.x.x   # same as Tier 1
python3 tools/test_detect_coresidency.py --host $BOAT_IP --rounds 100 \
    2>&1 | tee /tmp/tier2_run.log
```

The run takes roughly: 30s baseline + ~100 × (inference + network RTT) ≈ 30 + 15 = 45–60s, plus 5s settle.

Pass criteria (script exit code 0):
- `http_500 : 0`
- `inference_ms : p50<=60 p95<=70`
- `ws gaps (loop) : max <= 100ms`
- `PASS` on the final line.

- [ ] **Step 8.3: Check the serial monitor log for reboots during the test window**

In the monitor terminal (still running from Tier 1), scroll through the window corresponding to the test run. Look for any of:
- `Guru Meditation`
- `rst:0x...` (reset reason — should NOT appear during the test)
- `CPU halted`
- `backtrace:`

Pass criteria: none of the above appear during the ~60s test window. The only `DETECT:` log lines should be the expected inference logs.

- [ ] **Step 8.4: Verify dashboard health after the run**

Check the browser tab:
- Dashboard is still connected (WS indicator green / frames updating).
- Pitch/roll/heading values look sane (no snap to zero, no NaN, no frozen display).
- ToF grids are still updating.

- [ ] **Step 8.5: If all pass, tag phase 1 complete**

If everything in Steps 8.2–8.4 passed:

```bash
cd /workspaces/BoatEspP4
git tag -a detect-phase1-complete -m "Phase 1 /detect endpoint co-residency verified"
```

This tag marks the commit where the boat firmware first successfully runs on-demand ESPDet-Pico cat detection alongside IMU/ToF/WS/camera streaming. Phase 2 starts from here.

- [ ] **Step 8.6: If ANY step failed — diagnose per the spec's bisectability ladder**

Do NOT iterate fixes blind. Open the spec's [Bisectability ladder](../specs/2026-04-10-detect-endpoint-phase1-design.md#bisectability-ladder-what-each-failure-mode-tells-you) and match the failure mode to the first-action entry. Apply ONE fix, rerun Step 8.2, document what changed.

Common failures and their ladder entries:
- Tier 2 reboot mid-loop → heap/stack under co-residency
- Tier 2 inference_ms p95 > 70 → inspect concurrent sensor spikes (not cache — that's preempted)
- Tier 2 WS cadence gap > 100ms → investigate httpd close_fn path
- Tier 2 IMU heading jump > 3° → task starvation (the non-event that became an event; fix is dedicated inference task or priority tuning, and it's a real finding worth a follow-up spec)

---

## Post-plan: Memory updates (one-time, not per-task)

After phase 1 is tagged complete, update the project memory at `/root/.claude/projects/-workspaces-BoatEspP4/memory/` to record the phase 1 milestone:

- Update `project_overview.md` to mention `/detect` endpoint is live on port 80 with cat_detect.
- Optionally add a new `reference_detect_endpoint_howto.md` with the curl invocation, expected JSON, and phase-2 swap recipe (from the spec's Phase 2 Preview).

This is a one-time human-in-the-loop action, not part of the executable plan.
