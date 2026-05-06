# Maritime ESPDet-Pico Phase 1 — Land Rebalance Design Spec

## Goal

Rebalance the maritime detection dataset's class-1 ("land") labeling from the current 12-strip **upper-frustum curtain** (`get_curtains` + `curtains_to_yolo`) to a single upper-frustum bbox per image, so that obstacle-class gradient share during training is freed from land-label domination. Produce a new dedicated Kaggle notebook, cloned from `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle.ipynb`, that carries the label change and is explicitly framed as a **Phase 1 measurement experiment** — not a claim that obstacle recall will be fixed.

**Out of scope:** bench script (`/tmp/bench_maritime.py`) redesign, input resolution changes, model architecture changes, on-device firmware changes.

## Constraints

- **ESPDet-Pico at 224×224**: non-negotiable. 50ms inference target on ESP32-P4 is the hard latency budget; larger inputs break it.
- **2 classes**: obstacle (0), land (1). Unchanged. `data.yaml` stays as `names: [obstacle, land]`, `nc: 2`.
- **No firmware changes**: `detect_server.c`, `detection.cpp`, `dashboard.html` are untouched. Agent scan confirmed land class is purely visual — no downstream compatibility constraints.
- **Kaggle session budget**: single overnight retrain. Dataset generation + training + QAT + export must fit within a standard Kaggle kernel session.
- **Respect the original working version**: minimize the blast radius. The new notebook is a clone with targeted edits, not a refactor.
- **Semantic preservation**: class-1 "land" continues to mean **the above-horizon no-go region**, not the horizon line itself. The new labeling is a consolidation of the existing 12-strip union, not a redefinition of what "land" means.

## Background

### What the "curtain" actually is

The current `get_curtains(points, img_w, img_h, num_strips=12)` in `ESPDet_Pico_Maritime_Kaggle.ipynb` divides the image into 12 equal-width vertical strips and for each strip emits a bbox from `(x_start, y=0)` down to `(x_end, local_waterline_y)`. Each strip is ~`img_w/12` wide and runs from the top of the image to the local sea-edge y-value for that column. The union of all 12 strips covers the **entire above-horizon region** — sky + land — as a navigability no-go mask, discretized column-wise into YOLO-compatible axis-aligned rectangles.

`curtains_to_yolo(boxes, img_w, img_h)` normalizes each strip to YOLO format with class ID 1.

### Why the 12-strip curtain hurts obstacle training

Combined with typical 1-3 obstacles per image, 12 land strips make land labels dominate positive-anchor assignment by ~4:1. The on-device bench (prior user analysis) confirmed the symptom: on 3 test images with 4 total obstacles, obstacle recall is 0% (at both top_k=10 and top_k=20) while land recall is 83-100% when scored strip-by-strip.

The 12-strip design is semantically correct (it IS the navigability mask) but is discretized more finely than necessary. Each strip also individually touches `y=0`, so the model is already learning to emit boxes anchored at the top of the image — consolidating them into one box doesn't introduce a new shape, just a larger instance of what the assigner already handles.

### Why Phase 1 is a measurement, not a fix

The obstacle R=0% has two plausible causes, and Phase 1 isolates them:

1. **Gradient starvation** — obstacles lose the training signal competition because land labels dominate. Fixable by relabeling. *This is what Phase 1 tests.*
2. **Architectural floor** — at 224×224 with a stride-8 head, the detector's effective minimum detection size is ~25 training px. A 40 px obstacle in a 1278×958 source frame becomes ~7 px after resize, below the floor.

If Phase 1 rebalances gradients and obstacle mAP comes off zero → hypothesis 1 was (at least partially) true. If mAP stays at zero → hypothesis 2 is the real bottleneck, and Phase 2 needs architectural work.

### Why not tile-based training, obstacle crops, or horizon-band labels

All three were considered and rejected:

- **Tile-based training** has a train-inference mismatch: inference receives a whole camera frame, so 224-tile-trained features never see their input distribution. Also breaks the 50ms budget if inference also tiles.
- **Obstacle-centered crops** create severe positional/presence bias and fragment the land region per crop.
- **Horizon-band labels** (a thin strip at the sea-edge y-range) would **change the semantic meaning** of class 1 from "above-horizon no-go region" to "sea-land boundary line." The detector loses the navigability signal; the boat's original design intent is subverted for a label-economy win that the simpler upper-frustum consolidation already achieves.

## Design

### Section 1 — Core function: `sea_edge_to_upper_frustum`

Add a new function in the same cell as `get_curtains`. **Do not delete** the existing `get_curtains` or `curtains_to_yolo` — leave them as reference; the user can clean up post-Phase-1.

```python
def sea_edge_to_upper_frustum(points, img_w, img_h):
    """Option A replacement for get_curtains + curtains_to_yolo: one single
    upper-frustum bbox per call covering (0, 0) → (img_w, max_waterline_y).
    Semantically identical to the union of the 12 curtain strips — class-1
    "land" still means "above-horizon no-go region."

    Accepts MODD2 sea_edge polylines (N, 2) or cv2 contour arrays (N, 1, 2)
    or a concatenation of multiple contours' points. Returns [] on empty or
    degenerate input.

    For multi-landmass scenes, the caller should concatenate all contours'
    points into one array so max(y) spans every landmass's waterline — the
    frustum is conservative (over-includes navigable water where the lowest
    waterline passes through sea pixels of a higher-horizon landmass) rather
    than under-inclusive (which would be dangerous: boat would think land
    is water).
    """
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim == 3:                              # cv2 contour shape (N, 1, 2)
        pts = pts[:, 0, :]
    if np.any(np.isnan(pts)):
        pts = pts[~np.isnan(pts).any(axis=1)]
    if pts.shape[0] < 2:
        return []

    waterline_y = float(pts[:, 1].max())
    waterline_y = max(0.0, min(float(img_h - 1), waterline_y))
    if waterline_y <= 2:
        return []

    cx = 0.5
    cy = (waterline_y / 2.0) / img_h
    w_norm = 1.0
    h_norm = waterline_y / img_h
    return [f"1 {cx:.6f} {cy:.6f} {w_norm:.6f} {h_norm:.6f}"]
```

**Why these choices:**

- **Single function for both datasets**: MODD2's `sea_edge` polyline and MoDS's combined contour points have different shapes; handling both inside the function keeps the caller sites simple.
- **`max(waterline_y)` over all points**: conservative — the frustum extends down to the deepest (closest-to-boat) horizon across ALL points. Over-includes water in some cases; never under-includes land. Safe-biased for navigation.
- **No min-height floor**: the frustum is always naturally tall (`waterline_y` is usually 30-70% of `img_h`), so stride-8 anchor coverage is never an issue.
- **`waterline_y <= 2` guard**: matches the existing `get_curtains` trivial-strip filter. A horizon within 2 px of image top means effectively no land; skip the label.
- **Full-width `x=0..img_w` always**: consistent aspect ratio; matches the curtain's column-union shape exactly.
- **No densification**: not needed for a max-y computation.

### Section 2 — Dataset adapter swaps

Both callers become a **one-line swap**. The existing `process_mods()` per-contour loop stays intact — each contour gets its own `sea_edge_to_upper_frustum` call, producing one frustum per landmass.

**Change 2.1 — `process_modd2()`:**

Locate these two lines inside `process_modd2()`:

```python
curtains = get_curtains(edge, w, h)
labels.extend(curtains_to_yolo(curtains, w, h))
```

Replace them with one line:

```python
labels.extend(sea_edge_to_upper_frustum(edge, w, h))
```

The `edge` array is still NaN-filtered upstream in `process_modd2()`. The function tolerates this (its own NaN filter is a no-op on already-clean data).

**Change 2.2 — `process_mods()`:**

Locate these two lines **inside** the `for cnt in contours` loop:

```python
curtains = get_curtains(cnt, w, h)
labels.extend(curtains_to_yolo(curtains, w, h))
```

Replace them with one line:

```python
labels.extend(sea_edge_to_upper_frustum(cnt, w, h))
```

The outer `for cnt in contours` loop stays. The `cv2.contourArea(cnt) > 500` filter stays. Each contour (landmass) produces exactly one upper-frustum box via its own function call. MoDS images with N landmasses → N land labels (typically 1-3 per image).

**Behavioral note:**

- **MODD2**: 1 sea_edge polyline per image → 1 land label per image (always)
- **MoDS**: N landmass contours per image → N land labels per image (typically 1-3)
- **Aggregate** across the mixed dataset: ~1.5 average land positives per image vs ~3 obstacle positives. Previously ~12 land : 3 obstacles (land dominated). Now ~1.5 land : 3 obstacles → **obstacle class gains ~4× more gradient share than land.** Strong rebalancing achieved.

**Why per-landmass is the right granularity:**

1. Each contour from `cv2.findContours` is a connected landmass with its own local waterline. Emitting one frustum per contour preserves multi-island accuracy (an island left-of-center and a distant shoreline right-of-center get their own tight boxes).
2. The per-landmass frustum's `max(y)` is only computed over that one contour's points, so there's no over-inclusive union artifact when landmasses sit at different depths.
3. The existing loop structure doesn't need restructuring — this is the minimal-blast-radius change, matching the "respect the original" constraint.

### Section 3 — Validation via existing notebook machinery

**No new validation cells.** The notebook already has the tools.

**3.1 — `preview_samples()` spot-check (before retrain):** Run it immediately after the label swap. Expected: each image shows **exactly one** green (lime) box — full-width, top-left-anchored, extending down to the horizon. No strips. If strips are still visible, the function swap didn't take; debug before training.

**3.2 — Ultralytics per-class metrics (after retrain):** The training cell already runs val at the end. The per-class mAP table in the trainer output is the experiment's result. Compare against the baseline curtain run's final table. No new code.

**3.3 — On-device bench deferred:** The strip-by-strip scorer in `/tmp/bench_maritime.py` will see the new model's 1-box land predictions as missing most of the 12 GT strips. Focus on the obstacle column when you rerun it; ignore land precision. A region-IoU patch for the bench is a follow-up task.

### Section 4 — Success criteria

**Precondition check — Phase 0 cell:** Before kicking off the retrain, run a data-health cell that counts training-set obstacles whose post-resize size is ≥ 25 px (the stride-8 floor). If < 5% learnable, stop and go to Phase 2 planning.

```python
# Phase 0: Are obstacles large enough to survive 224x224 resize?
from pathlib import Path
import cv2 as _phase0_cv2
STRIDE_8_FLOOR_TRAIN_PX = 25

counts = {"learnable": 0, "below_floor": 0, "total": 0}
for lbl_path in Path(OUTPUT_DIR, "labels", "train").glob("*.txt"):
    img_path = Path(OUTPUT_DIR, "images", "train") / (lbl_path.stem + ".jpg")
    if not img_path.exists():
        continue
    img = _phase0_cv2.imread(str(img_path))
    if img is None:
        continue
    h_src, w_src = img.shape[:2]
    scale = 224 / max(w_src, h_src)
    for line in lbl_path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cls, _xc, _yc, w, h = parts
        if int(cls) != 0:
            continue
        w_train = float(w) * w_src * scale
        h_train = float(h) * h_src * scale
        counts["total"] += 1
        if min(w_train, h_train) >= STRIDE_8_FLOOR_TRAIN_PX:
            counts["learnable"] += 1
        else:
            counts["below_floor"] += 1

ratio = counts["learnable"] / counts["total"] if counts["total"] else 0.0
print(f"[Phase 0] Obstacle counts: {counts}")
print(f"[Phase 0] Learnable ratio: {ratio:.1%}")
if ratio < 0.05:
    raise RuntimeError(
        f"Learnable ratio {ratio:.1%} < 5%. Dataset is architecturally "
        f"mismatched; stop here and go to Phase 2 planning."
    )
elif ratio < 0.20:
    print("[Phase 0] WARNING: borderline ratio. Expect weak signal from Phase 1.")
else:
    print("[Phase 0] OK — enough learnable obstacles for Phase 1 to be meaningful.")
```

**Success thresholds:**

| Metric                      | Baseline (curtain)     | Phase 1 target            | Phase 1 hard floor |
| :-------------------------- | :--------------------- | :------------------------ | :----------------- |
| `obstacle` mAP50            | ~0% (on-device R=0%)   | **>10%**                  | **>3%**            |
| `obstacle` mAP50-95         | ~0%                    | **>5%**                   | **>1%**            |
| `land` mAP50                | high (strip-inflated)  | ≥ 60%                     | ≥ 40%              |
| `land` mAP50-95             | moderate               | ≥ 35%                     | ≥ 20%              |

**Decision matrix:**

- **Target hit on both** → ship. New labels deploy. Bench script update becomes follow-up.
- **Floor hit but target missed** → ship labels + begin Phase 2 planning.
- **Floor missed on obstacle** → do not ship retrained model. Keep labels as Phase 2 baseline.
- **Land regressed below floor** → investigate for bug in `sea_edge_to_upper_frustum`.

### What stays untouched in the cloned notebook

- `get_curtains`, `curtains_to_yolo`, `obstacle_to_yolo`, `densify_polyline` — left in place as reference
- `NUM_STRIPS = 12` constant — vestigial but stays
- All dataset download cells
- All train/val split-assignment logic
- `data.yaml` generation (still `names: [obstacle, land]`)
- `preview_samples()` visualization code
- Training cell (hyperparameters, epoch count, optimizer)
- QAT cell, calibration set, export pipeline

## Out of Scope

- Bench script redesign (`/tmp/bench_maritime.py` stays as-is)
- Input resolution changes (224×224 non-negotiable)
- Model architecture changes (ESPDet-Pico head stays)
- Firmware changes (none needed)
- Obstacle fix strategies beyond gradient rebalancing (tile-based training, obstacle-centered crops — all Phase 2+)
- New class definitions
- QAT / esp-ppq pipeline changes

## Phase 2 Pointers (if Phase 1 fails)

If Phase 1 produces obstacle mAP50 below the 3% hard floor, the architectural floor hypothesis is confirmed. Phase 2 options (multi-week projects):

- **Resolution bump**: 224 → 320 or 416. Re-benchmark 50ms.
- **Tile training + tiled inference**: both stages tile, accept 10× inference call count.
- **Model swap**: YOLOv26n at 224×224 instead of ESPDet-Pico.
- **Dataset curation**: filter sub-floor obstacles; synthetic large-obstacle augmentation.

Phase 2 is a separate spec.

## File Outputs

- `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — the cloned notebook with the label change
- `tools/esp-detection/test_sea_edge_upper_frustum.py` — self-contained pytest file with the function + 9 unit tests
- Trained artifact: `espdet_pico_224_maritime_phase1.espdl` (naming adjustable at QAT config)

## Timeline

One overnight Kaggle retrain after file edits complete. Decision point (target hit / floor hit / miss) reached via per-class mAP the next morning.
