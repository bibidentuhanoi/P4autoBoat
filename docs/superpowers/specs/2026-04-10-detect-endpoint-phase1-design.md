# /detect Endpoint Phase 1 — ESPDet-Pico Cat Demo Co-Residency

**Date:** 2026-04-10
**Status:** Design
**Phase:** 1 of N (test co-residency with an embedded demo model before swapping in the user's own model)

---

## Summary

Add an on-demand HTTP endpoint `GET /detect` to the boat firmware that runs ESPDet-Pico cat detection on an embedded test JPEG and returns detections as JSON. The endpoint runs inside the existing `esp_http_server` on port 80. No camera capture, no new tasks, no changes to existing subsystems. Lazy-initialized on first hit.

The purpose of this phase is to prove that model inference can coexist with IMU fusion, ToF, camera MJPEG streaming, WebSocket sensor broadcasting, and WiFi without destabilizing any of them. When this is stable, phase 2 swaps the embedded JPEG for a live camera frame grab, swaps JSON for a protobuf entity on the WS pipeline, and swaps the cat model for the user's own maritime model. None of those swaps require touching the endpoint plumbing.

## Goals

1. **One endpoint:** `GET /detect` returns JSON with detections from the embedded cat image.
2. **Zero impact on existing subsystems** when `/detect` is not called.
3. **Bounded impact on existing subsystems** when `/detect` is called: WS broadcasting continues, MJPEG stream continues, IMU heading stays continuous, no reboots.
4. **Bisectable failure modes** — when `/detect` fails, the failure can be attributed to a specific layer (build, load, decode, inference, marshal) without a debugger.
5. **Forward-compatible shape** — the endpoint contract and module structure are phase-2 stable (only internal details change when swapping JPEG→camera, JSON→protobuf, cat→maritime).

## Non-Goals (Phase 1)

- No live camera capture for inference input.
- No protobuf; JSON only.
- No `POST /detect` with uploaded body; embedded JPEG only.
- No dedicated inference task; runs inside the httpd task.
- No request rate limiting, auth, or query params.
- No dashboard UI integration (curl is the only client).
- No maritime / custom model; cat model only.
- No changes to the WDT configuration.

## Context (memory-backed constraints)

From `project_yolo26_psram_findings`:
- ESPDet-Pico cat 224 model is **5000/5000 bulletproof** at ~51ms inference.
- YOLO26n is unstable under sustained load — do not use.
- PSRAM retune destabilizes things — do not touch.

From `reference_espdet_pico_howto`:
- Allocate `CatDetect` once, call `run()` repeatedly. Never new/delete per frame.
- The C++ class has `lazy_load = true` by default — model weights load on first `run()`.
- Use `CONFIG_FLASH_ESPDET_PICO_224_224_CAT=y`.

From the template at [docs/esp-detection/deploy/espdet_example_template](../../docs/esp-detection/deploy/espdet_example_template) — this is what `espdet_run.py` generates for user-trained models, so phase 2's maritime model output will look like this:
- Test image name is canonically `espdet.jpg`, symbol `_binary_espdet_jpg_start`.
- Template ships two partition variants: [partitions.csv](../../docs/esp-detection/deploy/espdet_example_template/partitions.csv) with factory = 8000K for `IN_FLASH_RODATA`, and [partitions2.csv](../../docs/esp-detection/deploy/espdet_example_template/partitions2.csv) with factory = 2000K + a dedicated 4M `custom_det` spiffs partition for `IN_FLASH_PARTITION`. **Phase 1 adopts the FLASH_PARTITION variant** — it preempts the L2 cache thrash concern and keeps the boat's factory partition at its current 2048K (no size bump).
- Custom-trained models default `score_thr=0.25`, `nms_thr=0.7`. The cat_detect component used by phase 1 defaults to `score_thr=0.6`. Phase 1 uses the cat defaults; phase 2 inherits whatever the generated component ships.
- Result iteration pattern from [app_main.cpp](../../docs/esp-detection/deploy/espdet_example_template/main/app_main.cpp) is range-based: `for (const auto &res : detect_results) { res.category, res.score, res.box[0..3] }`. `box[0..3]` are `int` (x1, y1, x2, y2) in pre-letterbox image coordinates.
- Phase 2 swap from cat to maritime is three lines: `#include "cat_detect.hpp"` → `#include "espdet_detect.hpp"`, `CatDetect` → `ESPDetDetect`, and `espressif/cat_detect` → `espressif/maritime_detect` in `main/idf_component.yml`.

From `feedback_incremental_integration`:
- Default to on-demand execution when integrating heavy subsystems. Background tasks hide culprits.

From `feedback_ws_debugging`:
- Diagnose fully before writing code. No piecemeal fixes.

From verified sdkconfig:
- `CONFIG_SPIRAM=y`, `SPEED_200M`, `XIP_FROM_PSRAM`, `CACHE_L2_CACHE_256KB=y`, `LINE_128B=y`, `IDF_EXPERIMENTAL_FEATURES=y`, `PARTITION_TABLE_CUSTOM=y` — all prerequisites for cat_detect already in place.
- `CONFIG_COMPILER_CXX_EXCEPTIONS is not set` — C++ exceptions are disabled. Load failures panic, they do not return catchable errors.
- `CONFIG_ESP_TASK_WDT_EN=n`, `CONFIG_ESP_INT_WDT=n` — no watchdog safety net. Hangs are visible hangs.

## Architecture

### Component layout

```
main/
├── detection/
│   ├── detection.h        ← C API (included only by http_server.c)
│   └── detection.cpp      ← C++ impl, contains all C++ symbols
├── http_server.c          ← adds /detect URI handler
├── CMakeLists.txt         ← adds detection.cpp, configure_file(espdet.jpg), PRIV_REQUIRES cat_detect
├── idf_component.yml      ← adds espressif/cat_detect via override_path
├── espdet.jpg             ← gitignored, copied by configure_file at build time
└── ... (unchanged files)

partitions.csv             ← adds `cat_det` 1M data partition (factory size unchanged)
sdkconfig.defaults         ← three pinned Kconfig lines appended (FLASH_PARTITION, not RODATA)
.gitignore                 ← adds main/espdet.jpg
```

Flash layout after the change:

```
offset    size    name        contents
0x009000  24K     nvs         calibration data (preserved)
0x00F000  4K      phy_init    RF calibration
0x010000  2048K   factory     boat app binary (unchanged size)
(auto)    1M      cat_det     cat_detect .espdl model (new)
```

### C/C++ boundary

The boat firmware is pure C. `detection.cpp` is the only C++ file. It includes `cat_detect.hpp` and `dl_image_jpeg.hpp`, wraps them in `extern "C"` functions, and exposes nothing C++-flavored across `detection.h`. If `detection.cpp` needs to be ripped out, it can be — the rest of the codebase never references a C++ symbol, and removing `detection/` from `main/CMakeLists.txt` is a one-line operation.

### C API (detection.h)

```c
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DETECTION_STAGE_OK,
    DETECTION_STAGE_DECODE,
    DETECTION_STAGE_INFERENCE,
    DETECTION_STAGE_MARSHAL,
} detection_stage_t;

typedef struct {
    bool               ok;
    int                http_status;   /* 200 on success, 500 on graceful failure */
    detection_stage_t  failed_stage;  /* DETECTION_STAGE_OK when ok=true */
    char              *json;          /* malloc'd, caller owns */
    size_t             json_len;
} detection_response_t;

/* Lazy init on first call: allocates CatDetect, decodes embedded JPEG,
 * runs inference, marshals JSON. Subsequent calls reuse the allocated
 * model and decoded image. Must be called from the httpd worker task
 * only (see "Concurrency" section). */
esp_err_t detection_run_on_embedded(detection_response_t *out);

/* Frees out->json. Safe to call with out->json == NULL. */
void detection_response_free(detection_response_t *out);

#ifdef __cplusplus
}
#endif
```

### Where inference runs

Inside the `esp_http_server` worker task. No dedicated inference task.

`esp_http_server` is single-worker: one FreeRTOS task services all sockets via `select()`. Handlers are naturally serialized. This gives us:
- No mutex needed for the static pointers in `detection.cpp` (single-threaded access).
- `GET /` dashboard and `/detect` are mutually exclusive — a concurrent dashboard load would wait ~51ms behind inference, invisible to the user.
- `ws_tx_task` (in `main/transports/ws_transport.c`) is a **separate** task. WS broadcasting is unaffected by inference.
- The MJPEG stream on port 81 runs on a **separate httpd instance** (see `main/camera_stream.c`). Also unaffected.

### Why no pause / priority dance / core pinning

ESPDet-Pico inference is ~51ms per the memory baseline. Under that workload:
- **IMU task (10ms period):** Complementary filter is `dt`-based. Skipping 5 cycles = one 51ms integration step, mathematically equivalent to five 10ms steps for slow boat rotations. No visible heading discontinuity.
- **ToF task (5Hz = 200ms period):** 51ms is 25% of one cycle, shifts one read, does not skip.
- **WS broadcast:** Separate task, untouched.
- **MJPEG stream:** Separate task on a separate httpd, untouched.

Preemptively engineering around a non-event violates YAGNI and fights the bisectability principle. If Tier 2 testing shows heading drift correlated with `/detect` hits, that's a concrete finding with a concrete fix (priority adjustment, or lift inference to a dedicated task). Measure first.

### Lazy init mechanics

```cpp
static CatDetect        *s_detect       = nullptr;
static dl::image::img_t  s_decoded_img  = {};
static bool              s_decode_done  = false;
```

First call to `detection_run_on_embedded`:
1. If `!s_detect` → `s_detect = new CatDetect()`. The `CatDetect` default constructor has `lazy_load = true`; weights load on first `run()`.
2. If `!s_decode_done` → `s_decoded_img = dl::image::sw_decode_jpeg({espdet_jpg_start, len}, DL_IMAGE_PIX_TYPE_RGB888)`. Record success/failure. Set flag.
3. Time `s_detect->run(s_decoded_img)` with `esp_timer_get_time()`.
4. Marshal results to JSON in a stack/heap buffer.
5. Fill `out` and return `ESP_OK`.

Subsequent calls: skip steps 1 and 2, jump to step 3.

**Load-bearing invariant, documented at the top of detection.cpp:** *all calls to `detection_run_on_embedded` come from the httpd worker task.* If phase 2 adds a second caller (camera-triggered timer, CLI command, WS command handler), the invariant breaks and a mutex must be added around the static pointers.

## Endpoint Contract

### URL & method

`GET /detect` on `CONFIG_HTTP_API_PORT` (80), served by the existing httpd in [main/http_server.c](../../main/http_server.c). Not on the MJPEG server (:81).

### Success response (200 OK, application/json)

```json
{
  "ok": true,
  "model": "espdet_pico_224_224_cat",
  "source": "embedded:espdet.jpg",
  "image": { "w": 640, "h": 480 },
  "inference_ms": 51,
  "detections": [
    { "category": 0, "score": 0.87, "box": [120, 80, 380, 410] }
  ]
}
```

Field notes:
- `model` and `source` are self-describing so test logs are greppable.
- `image.w/h` are the decoded RGB888 dimensions of the embedded JPEG. `box` coordinates are in pre-letterbox image-pixel space (matches `image.w/h`). Dashboard overlay math can use these directly in phase 2.
- `inference_ms` measures only `s_detect->run(img)` — not JPEG decode, not JSON marshal, not HTTP send. Compare against the 51ms baseline from the memory.
- `detections` may be empty. `category` is the raw class index from the model (phase 1 has one class, label resolution is client-side). `box` is `[x1, y1, x2, y2]` floats rounded to ints, top-left origin.

### Graceful failure response (500, application/json)

```json
{
  "ok": false,
  "stage": "decode" | "inference" | "marshal",
  "error": "short human-readable string"
}
```

**Honest scope of `stage`:** this field catches *graceful* failures — `sw_decode_jpeg` returned NULL, `run()` completed but the result vector is malformed, JSON marshal OOM. It does **not** catch hard crashes in `new CatDetect()` or `load_model()`. Because `CONFIG_COMPILER_CXX_EXCEPTIONS` is disabled, a corrupt `.espdl` or missing model symbol causes a `std::terminate`/panic → guru meditation → reboot. In that case, bisectability comes from the serial log, not HTTP. This is documented in the bisectability ladder below.

The stage `"model_init"` from earlier drafts is removed to avoid promising an API we can't deliver.

### No query params, no request body, no rate limiting

Score/NMS thresholds use `cat_detect` defaults (0.6 / 0.7) — the exact values the 5000/5000 torture test was run against. No tunables exposed via URL.

## Configuration Changes

### sdkconfig.defaults (append)

```
CONFIG_FLASH_ESPDET_PICO_224_224_CAT=y
CONFIG_CAT_DETECT_MODEL_IN_FLASH_PARTITION=y
# CONFIG_CAT_DETECT_MODEL_IN_FLASH_RODATA is not set
```

Rationale: putting the model in a dedicated `cat_det` partition (instead of rodata) prevents the 487KB model from evicting app code from the 256KB L2 cache during inference, and keeps the app partition at its current 2048K size (no guess-bump needed). This mirrors the template's [partitions2.csv](../../docs/esp-detection/deploy/espdet_example_template/partitions2.csv) variant, which is the canonical pattern for custom-trained models.

The existing `SPIRAM=y`, `SPIRAM_SPEED_200M=y`, `SPIRAM_XIP_FROM_PSRAM=y`, `CACHE_L2_CACHE_256KB=y`, `CACHE_L2_CACHE_LINE_128B=y`, `IDF_EXPERIMENTAL_FEATURES=y` lines stay as-is — all prerequisites for cat_detect are already satisfied.

### main/idf_component.yml (append)

```yaml
  espressif/cat_detect:
    version: "*"
    override_path: "../docs/esp-dl/models/cat_detect"
```

The override_path is relative to `main/` and resolves to the in-repo model component at [docs/esp-dl/models/cat_detect](../../docs/esp-dl/models/cat_detect). That component's own `idf_component.yml` pulls `espressif/esp-dl` via `override_path: ../../esp-dl` → [docs/esp-dl/esp-dl](../../docs/esp-dl/esp-dl), also in-repo.

**First-build verification gate (Tier 0):** component resolution log must show `espressif/cat_detect (override_path: ...)`. If it instead downloads from the registry, the manifest is wrong and the build proceeds with a possibly-mismatched `esp-dl` version — stop and fix before writing code.

### main/CMakeLists.txt

Before `idf_component_register(...)`:

```cmake
configure_file(
    "${CMAKE_SOURCE_DIR}/tools/esp-detection/espdet.jpg"
    "${CMAKE_CURRENT_LIST_DIR}/espdet.jpg"
    COPYONLY)
```

Inside `idf_component_register(...)`:
- Add `"detection/detection.cpp"` to `SRCS`.
- Add `"detection"` to `INCLUDE_DIRS`.
- Add `cat_detect` to `PRIV_REQUIRES`.
- Add `EMBED_FILES "espdet.jpg"`.

**Why `configure_file` instead of `EMBED_FILES "${CMAKE_SOURCE_DIR}/.../espdet.jpg"`:** using an absolute path in `EMBED_FILES` would mangle the generated symbol name (e.g., `_binary__workspaces_BoatEspP4_tools_esp_detection_espdet_jpg_start`), which is not portable across checkouts. Copying to the component directory first guarantees the symbol is `_binary_espdet_jpg_start` on every machine.

### .gitignore

Append: `main/espdet.jpg`

The copy is a build artifact, not source. The canonical image lives at [tools/esp-detection/espdet.jpg](../../tools/esp-detection/espdet.jpg).

### partitions.csv

Current:
```
nvs,      data, nvs,      0x9000,   0x6000
phy_init, data, phy,      0xF000,   0x1000
factory,  app,  factory,  0x10000,  2048K
```

Change to:
```
nvs,      data, nvs,      0x9000,   0x6000
phy_init, data, phy,      0xF000,   0x1000
factory,  app,  factory,  0x10000,  2048K
cat_det,  data, spiffs,   ,         1M
```

What changes:
- **Factory stays at 2048K.** No size bump, no link-overflow risk from the model itself (it's not in the app binary anymore).
- **Adds `cat_det` data partition**, 1MB. Partition name MUST be exactly `cat_det` — that string is hardcoded in [cat_detect.cpp](../../docs/esp-dl/models/cat_detect/cat_detect.cpp) (`static const char *path = "cat_det";` when `CONFIG_CAT_DETECT_MODEL_IN_FLASH_PARTITION` is set).
- 1MB is more than 2× the 487KB model, leaves margin for any phase-2 retraining. Template uses 4M; we use 1M to avoid wasting flash for a known-size model. If phase 2's maritime model exceeds 1MB, bump to 2M/4M at that time.
- Subtype `spiffs` is just a type tag — the model is accessed as raw bytes via `dl::Model(path, ...)`, not as a filesystem mount. No spiffs filesystem overhead.

This is **lossless for NVS**: `nvs` stays at its existing `0x9000/24K` offset. Calibration data is preserved across the partition change. The new `cat_det` partition is appended after factory (its offset is computed automatically from the free space).

**First-time flashing requires `idf.py flash`, not `idf.py app-flash`.** The cat_detect component's CMakeLists adds a dependency on the `flash` target (`add_dependencies(flash CAT_DETECT_MODEL)` and `esptool_py_flash_to_partition(flash "cat_det" ...)`), so the model partition is populated only during a full flash. Subsequent `idf.py flash` calls re-flash both automatically. `idf.py app-flash` only writes the factory partition and leaves `cat_det` stale — if you re-run `app-flash` after rebuilding the model, inference will fail. The Tier 0 / Tier 1 scripts use `idf.py flash` explicitly.

### main/http_server.c

In `http_server_start()`:
- Change `cfg.stack_size` from `8192` → `16384`. Rationale: the C++ inference path uses the caller's stack for preprocessor intermediates, postprocessor NMS scratch, and `std::vector` manipulations. 8KB is tight for a task that also handles `/` and `/ws` upgrade. 16KB is cheap on ESP32-P4. This is not about scheduling or starvation — purely stack headroom.
- Change `cfg.max_uri_handlers` from `4` → `5`. Without this bump, `httpd_register_uri_handler` for `/detect` silently fails to register.

Add after the existing `dashboard_uri` registration:
```c
static esp_err_t detect_handler(httpd_req_t *req) {
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ret;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req,
        out.http_status == 200 ? "200 OK" : "500 Internal Server Error");
    httpd_resp_send(req, out.json, out.json_len);
    detection_response_free(&out);
    return ESP_OK;
}

static const httpd_uri_t detect_uri = {
    .uri = "/detect", .method = HTTP_GET,
    .handler = detect_handler, .user_ctx = NULL,
};
ret = httpd_register_uri_handler(server, &detect_uri);
if (ret != ESP_OK) { /* existing error path */ }
```

Include: `#include "detection/detection.h"` at the top of the file.

## Test Plan

### Tier 0 — Prerequisites (gate before writing detection.cpp)

Goal: prove the build, dependency, and partition plumbing works before committing to ~200 lines of C++.

Steps:
1. Apply only: the `idf_component.yml` dep, the `CMakeLists.txt` `PRIV_REQUIRES cat_detect` line, the three `sdkconfig.defaults` lines, and the `partitions.csv` edit adding the `cat_det` partition. No `detection/` dir yet. No `configure_file`, no `EMBED_FILES`, no `detect_handler`.
2. `idf.py reconfigure` — watch for component resolution log.
3. `idf.py build` — watch for link success and app binary size.

Pass criteria:
- Log contains `espressif/cat_detect (override_path: ...)` — NOT `(version: *)` from registry.
- Build succeeds.
- `idf.py size` reports the app binary size ≤ 2000KB (plenty of headroom in the 2048K factory partition; the 487KB model is NOT in the app binary).
- `idf.py partition-table` reports the `cat_det` partition with size ≥ 1M at offset after factory.

Sanity check only; no contingent actions expected. If the app binary somehow exceeds 2000K, investigate what got pulled in transitively before bumping the partition — a surprise of that magnitude is a signal, not a routine fix.

If Tier 0 fails: the problem is dependency/build/partition, not detection code. Fix before proceeding.

### Tier 1 — Smoke test (one curl)

Goal: end-to-end functional verification.

Preconditions: `detection.cpp`, `detection.h`, `configure_file`, `EMBED_FILES`, `.gitignore`, and `http_server.c` changes applied. Firmware flashed and WiFi connected.

Command:
```bash
curl -sS http://$BOAT_IP/detect | jq
```

Pass criteria:
- HTTP 200.
- `ok == true`, `model == "espdet_pico_224_224_cat"`, `source == "embedded:espdet.jpg"`.
- `detections.length >= 1`, `detections[0].category == 0`, `detections[0].score > 0.6`.
- `inference_ms` ∈ [40, 70]. First-call warmup may be higher (model load) — see note below.
- Dashboard `GET /` loads afterward.
- WS connection from dashboard remains connected throughout.
- On next reboot, `esp_reset_reason()` is not `ESP_RST_PANIC` or `ESP_RST_WDT` (inspect serial log).

First-call warmup note: the very first `/detect` after boot triggers `load_model()` (reads ~487KB from rodata, allocates PSRAM working buffers). Expect first-call latency of several hundred ms. `cfg.send_wait_timeout = 10s` accommodates this comfortably. The 40–70ms `inference_ms` bound applies to calls 2+ (and also to call 1 since `inference_ms` times only `run()`, not `load_model()`).

### Tier 2 — Co-residency stability (required, the whole point of this phase)

Goal: prove inference does not destabilize IMU, ToF, WS broadcast, or MJPEG streaming.

**Test conditions:** boat is static (sitting on the bench, not in motion). This is how we isolate inference-induced variance from actual sensor motion — the IMU heading assertion below assumes near-zero expected angular motion during the test window.

Harness: new Python script at `tools/test_detect_coresidency.py`, ~50 lines. Depends on `requests`, `websocket-client`.

What it does:
1. Connects to `ws://$BOAT_IP/ws`. Logs arrival timestamp of every protobuf frame to a buffer.
2. Optional: opens HTTP connection to `http://$BOAT_IP:81/stream` and counts MJPEG frame boundaries (`--boundary--`).
3. Sleeps 30s to collect a sensor-frame cadence baseline + an IMU heading baseline (from parsed protobuf — dashboard already does this, reuse parser).
4. In a loop, 100 iterations: `requests.get(f'http://{BOAT_IP}/detect')`, parse JSON, append `inference_ms` to a list.
5. Sleeps 5s post-loop to let sensor cadence re-stabilize.
6. Disconnects WS.

Pass criteria (asserted by the script, non-zero exit on any failure):
- **Zero HTTP 500s** from `/detect` during the 100 calls.
- **Zero reboots.** Check via `esp_reset_reason` exposed as an optional `GET /health` endpoint, OR by parsing serial log for `rst:` or `Guru Meditation` markers during the test window.
- **WS cadence unbroken:** the max inter-frame gap on `/ws` during the loop is ≤ 2× the pre-loop mean (mean is ~50ms for 20Hz sensor snapshots, so gap ≤ 100ms).
- **MJPEG did not stall:** at least 1 frame boundary observed per 1 second of test (very loose threshold; the stream's baseline is higher).
- **Inference latency bounded:** `inference_ms` p50 ≤ 60, p95 ≤ 70. Baseline is 51ms; with the model in its own partition (no L2 cache contention with app code) there is no known reason for drift under co-residency. Any sustained p95 > 70 is a real finding worth investigating.
- **IMU heading continuous:** |heading(t) − heading(t-1)| across consecutive WS frames during the loop is ≤ 2° (normal noise floor) + a 1° tolerance. No step > 3° between consecutive frames.

If Tier 2 passes: phase 1 is done. The co-residency story is proven for on-demand inference with the cat model. Phase 2 can begin.

### Tier 3 — Torture (optional, deferred to phase 1.5)

Mirror of the `project_yolo26_psram_findings` 5000/5000 pattern: 500 `/detect` rounds × 10 board resets, concurrent with sensor traffic. Target: 5000/5000 success. Not gating phase 1.

### Bisectability ladder (what each failure mode tells you)

| Failure | Layer | First action |
|---|---|---|
| Tier 0 log does not show `override_path` | component manager | fix idf_component.yml |
| Tier 0 `cat_det` partition missing from `partition-table` output | partitions.csv | verify the `cat_det` line syntax, rerun reconfigure |
| Tier 0 app binary > 2000K | unexpected transitive pull | investigate what got linked in — don't just bump partition |
| Tier 1 HTTP 500 `stage=decode` | `sw_decode_jpeg` | verify embedded JPEG symbol, re-check `configure_file` copy |
| Tier 1 HTTP 500 `stage=inference` | `run()` returned malformed result | log result vector, check score thresholds |
| Tier 1 HTTP 500 `stage=marshal` | JSON formatter | log input, likely OOM or snprintf bug |
| Tier 1 no response, reboot at first `/detect` | hard crash in model init | serial log; **first suspect: `idf.py app-flash` left stale `cat_det`** — re-run `idf.py flash` |
| Tier 1 reboot persists after full flash | corrupt `.espdl` / missing symbol | check model symbol in elf, check esp-dl version alignment |
| Tier 2 reboot mid-loop | heap/stack under co-residency | `heap_caps_get_info` at `/health`, check free PSRAM before/after loop |
| Tier 2 `inference_ms` p95 > 70 | unexpected co-residency cost | inspect what ran concurrently — most likely a sensor task spike, not cache (which is preempted) |
| Tier 2 IMU heading jump | task starvation (the non-event that was an event) | move inference to dedicated task or adjust httpd priority |
| Tier 2 WS cadence gap > 100ms | unexpected httpd→ws_tx interference | investigate `cfg.close_fn` path |

## Known Limits and Phase-2 Levers

1. **C++ exceptions disabled** → `stage: "model_init"` does not exist. Hard init failures = guru meditation. If phase 2 needs graceful init failure reporting, enable `CONFIG_COMPILER_CXX_EXCEPTIONS` (adds binary weight).

2. **Single-thread invariant on statics** → documented in a comment at the top of `detection.cpp`. Any phase-2 addition of a second caller (camera timer, CLI, WS command handler) must add a mutex around `s_detect`, `s_decoded_img`, `s_decode_done`.

3. **`idf.py app-flash` is unsafe after model rebuild** — see partitions.csv section. Use `idf.py flash`.

4. **No WDT safety net** (existing firmware choice). A hang in inference is a visible hang, not a silent reset. Consistent with `feedback_incremental_integration`.

5. **Embedded test image is static** — one cat, one expected result. Does not exercise input variance. Phase 2 replaces with live camera frames; phase 1 is about *plumbing*, not model accuracy validation.

6. **No `/health` endpoint yet.** Tier 2 optionally uses one to read `esp_reset_reason` and heap stats. If added, it's one more URI handler on the existing httpd (bump `max_uri_handlers` to 6). Optional for this spec.

**Preempted from the DA pass, no longer a concern for phase 1:**
- ~~Cat model in flash rodata competes with 256KB L2 cache~~ — model now lives in its own `cat_det` partition via `CAT_DETECT_MODEL_IN_FLASH_PARTITION=y`, so inference reads don't evict app code.
- ~~Partition headroom unknown~~ — factory stays at 2048K because the model is no longer in the app binary. Tier 0 dry build still measures app size as a sanity check, but no contingent bump is expected.

## Phase 2 Preview (out of scope, for context only)

The template at [docs/esp-detection/deploy/espdet_example_template](../../docs/esp-detection/deploy/espdet_example_template) is what `espdet_run.py` generates when the user trains a custom maritime model. Because phase 1 adopts the template's partition layout (`FLASH_PARTITION`) and file naming (`espdet.jpg`, `_binary_espdet_jpg_start`), phase 2 integration is a pure swap, not a rework:

1. **Run `espdet_run.py`** with the maritime dataset → generates a custom model component at `deploy/maritime_detect/` with `espdet_detect.hpp`, `espdet_detect.cpp`, `models/p4/espdet_pico_224_224_maritime.espdl`, and a `Kconfig` menu.
2. **Copy or symlink** that generated component into `docs/esp-dl/models/maritime_detect/` (or wherever you keep it).
3. **Edit `main/idf_component.yml`:** `espressif/cat_detect` → `espressif/maritime_detect`, update `override_path`.
4. **Edit `main/detection/detection.cpp`:**
   - `#include "cat_detect.hpp"` → `#include "espdet_detect.hpp"`
   - `CatDetect` → `ESPDetDetect` (class rename, API identical)
   - update `s_model_name` string in the JSON response
5. **Partition rename:** `cat_det` → `espdet_det` (or whatever the generated `espdet_detect.cpp` hardcodes). Partition name is the only bit that isn't copy-paste.
6. **Swap embedded image:** `tools/esp-detection/espdet.jpg` is already the canonical test image — no change needed unless the user wants a maritime-specific test image.

Other phase 2 items (independent of the swap above):
- Replace `detection_run_on_embedded` with `detection_run_on_camera_frame` — same return type, same JSON shape, different source. The camera path adds a `camera_driver_grab_one()` API.
- Add protobuf `DetectionResult` message to `main/proto/boat.proto`, broadcast via `pipeline.c` → WS. JSON output stays available on `/detect` for out-of-band testing.
- Optionally move inference to a dedicated task if Tier 2 exposed any co-residency cost (unlikely given ESPDet-Pico's 51ms baseline).
- Dashboard UI overlay for detection boxes over the MJPEG stream.

None of these changes affect the public contract of `GET /detect` or the `detection.h` C API.

## File Manifest

| Path | Status | Notes |
|---|---|---|
| `docs/superpowers/specs/2026-04-10-detect-endpoint-phase1-design.md` | new | this file |
| `main/detection/detection.h` | new | C API |
| `main/detection/detection.cpp` | new | C++ impl, ~200 lines. Iterates `detect->run(img)` results with range-for, serializes `res.category / res.score / res.box[0..3]` to JSON. |
| `main/espdet.jpg` | new (build artifact, gitignored) | `configure_file` copy from `tools/esp-detection/espdet.jpg` |
| `main/idf_component.yml` | modified | add `espressif/cat_detect` with override_path |
| `main/CMakeLists.txt` | modified | `configure_file`, `detection.cpp` in `SRCS`, `detection` in `INCLUDE_DIRS`, `cat_detect` in `PRIV_REQUIRES`, `EMBED_FILES "espdet.jpg"` |
| `main/http_server.c` | modified | `#include`, `stack_size` 8192→16384, `max_uri_handlers` 4→5, `detect_handler`, URI registration |
| `sdkconfig.defaults` | modified | append 3 lines: `FLASH_ESPDET_PICO_224_224_CAT=y`, `CAT_DETECT_MODEL_IN_FLASH_PARTITION=y`, `# ...IN_FLASH_RODATA is not set` |
| `.gitignore` | modified | append `main/espdet.jpg` |
| `partitions.csv` | modified | append `cat_det, data, spiffs, , 1M` line. Factory partition unchanged. |
| `tools/test_detect_coresidency.py` | new | Tier 2 harness |
| `main/main.c` | **untouched** | no init call, no task create |
| existing sensor/WS/camera files | **untouched** | |

## Open Questions

None blocking. Everything deferrable is explicit in "Known Limits" or "Phase 2 Preview".
