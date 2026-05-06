# Maritime Phase 1 — Below-Floor Rate-Limit

**Spec · 2026-04-13**

## Context

`tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` aborts at the Phase 0 precondition check with a learnable ratio of 1.7 %. Of 70,770 obstacle bboxes in the merged MODD2 + MoDS training set, only 1,238 survive the 800→224 resize above the stride-8 25 px floor. The remaining 69,532 are "below-floor" — architecturally undetectable by the existing ESPDet-Pico head at 224.

Phase 1's intent was to measure whether consolidating land into a single upper-frustum box (the `sea_edge_to_upper_frustum` change) recovers obstacle recall. Phase 0 blocks that experiment because the dataset is too skewed for any rebalance to help — the obstacles are too small to begin with.

Design brainstorming established:

- The boat's sensor stack (dual VL53L5CX ToF + 800×640 camera at ~53° HFOV) already covers 0–4 m via ToF. The camera model's real task is classification at ~4 m distance for obstacles ≥ ~30 cm in real-world size.
- At 224 full-frame with 53° HFOV, pixels-per-meter at 4 m ≈ 56. A 60 cm kayak/log becomes ~34 px — above the floor. The architecture is adequate; the dataset imbalance is the blocker.
- The Phase 0 guard counts every obstacle in MODD2 / MoDS including far-field positives the boat doesn't need. The 98 % "below_floor" figure is a dataset-distribution artefact, not an architectural verdict.

## Goal

Run one training cycle with below-floor label dominance neutralised at data-ingest time, producing a measurable data point that tells us whether ESPDet-Pico 224 can learn maritime obstacles when fed a clean gradient signal.

## Non-goals

- No model, head, or input-size change.
- No custom augmentation pipeline (Ultralytics built-ins are sufficient).
- No boat-captured validation set (tracked as a separate future phase).
- No change to `sea_edge_to_upper_frustum` land-rebalance logic, export cells, or ESPDL quantisation.

## Design

Two edits, one verification. Respect the existing notebook structure — no new helpers, no new cells, no pipeline rebuild.

### Edit 1 — Phase 0 guard softens to a warning

**Cell:** the Phase 0 precondition-check cell (contains `STRIDE_8_FLOOR_TRAIN_PX = 25` and prints `[Phase 0] Obstacle counts`).

**Change:** replace the `raise RuntimeError(...)` branch with an informational `print(...)` of equal content. The warn and OK branches remain. The measurement stays; the abort goes.

Rationale: the ratio is useful diagnostic output. The stop was correct when the boat stack was unknown; now we know ToF covers the below-floor regime and the camera only needs the learnable subset.

### Edit 2 — Rate-limit below-floor labels at ingest in MODD2 and MoDS processors

**Cells:** the MODD2 processor (`process_modd2` or equivalent, contains `sea_edge_to_upper_frustum` + obstacle loop) and the MoDS processor (same shape, consumes mask-derived contours). Both already iterate obstacles with pixel-box `bx, by, bw, bh` and source-image dims `w, h` in scope, and call `obstacle_to_yolo(bx, by, bw, bh, w, h)` before appending to the running `labels` list.

**Change:** just before the `obstacle_to_yolo` call (or equivalent labels append in the MoDS contour branch), compute the same min-side-at-train threshold Phase 0 uses, in source-pixel form, and skip below-floor samples probabilistically:

```python
if min(bw, bh) * (224 / max(w, h)) < 25 and random.random() > (2 / 56):
    continue
```

Equivalent to Phase 0's `min(_w_train, _h_train) < STRIDE_8_FLOOR_TRAIN_PX` because `_w_train = nw * w * (224/max(w,h)) = bw * (224/max(w,h))` and likewise for height. Using the pre-normalisation pixel box avoids any drift between the two forms.

**Where in each processor:**
- **MODD2**: inside the `for ob in obstacles` loop, before `xc, yc, nw, nh = obstacle_to_yolo(bx, by, bw, bh, w, h)`.
- **MoDS**: inside the contour-to-bbox branch that produces obstacle boxes, same position — before the yolo conversion. If MoDS derives boxes via `cv2.boundingRect(cnt)`, the four returned ints serve as `bx, by, bw, bh`.

This retains ≈ 2,476 below-floor obstacles out of 69,532 (∼2× the 1,238 learnable count), flipping the gradient ratio from 56 : 1 noise-dominant to ≈ 2 : 1 signal-dominant.

**Reproducibility:** set `random.seed(42)` (or equivalent) at the start of the data-preparation cell so the rate-limit is deterministic across rebuilds.

**Why 2/56 and not looser.** ESPDet-Pico has 0.36 M parameters. Tiny models can't absorb label noise the way YOLOv8-L can; each wasted parameter is a parameter not learning real obstacles. A clean gradient on 1,238 learnable positives plus ∼2,500 background noise labels produces a cleaner experimental signal than loose ratios would. If Run 1 underperforms, loosening to 10/56 is a one-character follow-up.

### Verification (not an edit) — existing training args

The current Phase 2 training cell calls `model.train(..., mosaic=1.0, copy_paste=0.1, ...)` and inherits Ultralytics defaults for `scale=0.5`, `translate=0.1`, `fliplr=0.5`. Those defaults already give:

- ±50 % random zoom per sample (each learnable obstacle seen at many sizes per epoch).
- ±10 % random translation (breaks any centred-obstacle prior from training composition).
- 50 % horizontal flip (harmless for water scenes).

No augmentation-side edits are needed. If Run 1 mAP is borderline, the next tuning knob is `scale=0.7` for more aggressive zoom, not a custom pipeline.

## Success criteria

Single training cycle on MODD2 + MoDS with Edits 1 and 2 applied. Report at end of run:

- `val/mAP50` on the obstacle class (class 0).
- Count of retained below-floor labels (expect ∼2,500 ± stochastic).
- Count of retained learnable labels (expect unchanged at 1,238).

**Pre-declared decision guidance** (so results aren't rationalised after the fact):

| Outcome | Interpretation | Next step |
| --- | --- | --- |
| `mAP50 ≥ 0.30` on obstacle class | Architecture works; data was the problem | Start boat-capture phase for fine-tune |
| `mAP50 < 0.10` | Architecture floor hypothesis confirmed | Revisit input size or two-stage design |
| `0.10 ≤ mAP50 < 0.30` | Borderline | One rerun with 10/56 ratio before architecture discussion |

## Risks

- **`random.seed()` not set.** Rate-limit becomes non-reproducible across rebuilds. Mitigation: explicit seed in the imports cell or at the top of the processor cell, before MODD2 / MoDS calls.
- **`random` not imported in the processor cell's scope.** `NameError` at build time. Mitigation: verify `import random` is visible, or add at the top of each processor cell.
- **Formula drift between Phase 0 and Edit 2.** If the rate-limit min-side formula drifts from Phase 0's reported formula, the filter threshold disagrees with the ratio being printed. Mitigation: reuse the `25` constant verbatim and copy Phase 0's formula literally.
- **Single-seed training noise.** ±3–5 mAP seed variance on a small dataset could flip the verdict near the thresholds. If the result lands in the borderline range, rerun 2–3 seeds before architectural conclusions.
- **Class balance skew after rate-limit.** The class-1 (land) label count stays constant while obstacle count drops from ~70 k to ~3.7 k. Ultralytics' built-in class-balance loss should compensate, but if mAP50 on class 0 is low and class 1 is high, that's a signal to rebalance explicitly.

## Deferred to future phases

- Boat-captured validation set (`val_boat`) — critical but separable.
- Fine-tune on boat data — depends on `val_boat` existing first.
- Custom crop augmentation pipeline — explicitly rejected here as over-engineering for the current setup; revisit only if Ultralytics built-ins demonstrably can't carry us.
- Pixel-accurate camera-crop window derivation from `projectZone()` geometry — not needed while camera works full-frame.

## Pointers

- Existing Phase 1 spec: `docs/superpowers/specs/2026-04-11-maritime-phase1-land-rebalance-design.md`.
- Phase 0 precondition cell and MODD2 / MoDS processors: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb`.
- Dashboard geometry reference (camera + ToF projection, 53° HFOV implicit): `main/dashboard.html`, `projectZone()` at L572.
