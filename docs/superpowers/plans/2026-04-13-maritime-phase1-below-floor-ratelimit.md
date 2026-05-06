# Maritime Phase 1 — Below-Floor Rate-Limit: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Neutralise the 56 : 1 below-floor-to-learnable label ratio in the MODD2 + MoDS training set by randomly subsampling below-floor obstacles at ingest, so ESPDet-Pico can train on a clean gradient signal. Softens the Phase 0 abort to a warning.

**Architecture:** Three small edits to a single Jupyter notebook. No new modules. No new pipeline stages. The filter uses Phase 0's own formula, evaluated per-obstacle during existing MODD2 and MoDS processor loops. Ultralytics built-in augmentation (already in the training cell via defaults) handles zoom/translation variance without a custom pipeline.

**Tech Stack:** Jupyter / Python 3 / Ultralytics YOLO 8.x / ESPDet-Pico (custom nn.tasks `parse_model` patch). The notebook runs on Kaggle (T4 GPU) but edits can be made locally to the `.ipynb` and re-uploaded.

**Governing spec:** `docs/superpowers/specs/2026-04-13-maritime-phase1-below-floor-ratelimit-design.md`

---

## File Structure

**Modified:**
- `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — five cells touched total (imports, Phase 0 check, MODD2 processor, MoDS processor, data-build invocation for re-run verification).

**Not modified:**
- Model YAML, esp_tasks custom parse_model, training hyperparameters, export / quantize / deploy cells, `sea_edge_to_upper_frustum` land logic.

**Nested git note:** `tools/esp-detection/` has its own `.git` directory (see `project_nested_git_in_tools.md`). Do **not** run git commit / git add for these changes from the outer repo. Edits are in-place, saved, and that's the end of the implementation step. Training run + result capture is the next human step.

---

## Task 1: Seed `random` for reproducible rate-limiting

**Files:**
- Modify: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — the imports / top-of-notebook setup cell (the first code cell after the intro markdown, around notebook cell index 1 — contains `import os`, `import numpy as np`, `import cv2`, or similar).

- [ ] **Step 1: Confirm current state — check whether `random` is already imported and seeded**

Run this verification command from the repo root:

```bash
python3 -c "
import json
with open('tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb') as f:
    nb = json.load(f)
for i, cell in enumerate(nb['cells']):
    src = ''.join(cell.get('source', []))
    if 'import random' in src or 'random.seed' in src:
        print(f'Cell {i} has random-related code:')
        print(src[:500])
        print('---')
"
```

Expected: either **no output** (neither imported nor seeded — we'll need to add both), or output showing where it's currently set. If seeded already, Step 3 becomes "verify seed is 42; change if different."

- [ ] **Step 2: Identify the target cell**

The first code cell (after the intro markdown) should be the imports cell. If imports are split across multiple cells, use the cell that contains the dataset-prep imports (likely near `import cv2` and the `CLASS_NAMES` definition around notebook cell index 2–3).

- [ ] **Step 3: Add `import random` and `random.seed(42)` to that cell**

Add these two lines to the imports cell. Place `import random` with the other standard-lib imports (near `import os`), and place `random.seed(42)` immediately after all imports, at the end of that cell, so any downstream cell that uses `random` inherits the seed.

If `import random` is already present, only add `random.seed(42)`. If `random.seed(<other value>)` is already present, change it to `42` for alignment with the spec.

- [ ] **Step 4: Verify the change landed**

Rerun the same verification command from Step 1. Expected: the output now includes the cell containing both `import random` and `random.seed(42)`.

---

## Task 2: Soften the Phase 0 guard to a warning

**Files:**
- Modify: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — the Phase 0 precondition check cell (contains `STRIDE_8_FLOOR_TRAIN_PX = 25` and `[Phase 0] Learnable ratio:` prints).

- [ ] **Step 1: Failing test — run the Phase 0 cell on the current (unsoftened) notebook**

Open the notebook and run the Phase 0 cell. Expected output:

```
[Phase 0] Obstacle counts: {'learnable': 1238, 'below_floor': 69532, 'total': 70770}
[Phase 0] Learnable ratio: 1.7%
RuntimeError: Learnable ratio 1.7% < 5%. Dataset is architecturally mismatched; ...
```

This is the current broken state — the guard aborts training. Confirm this is what you see.

- [ ] **Step 2: Locate the `if _ratio < 0.05:` block in that cell**

The block contains `raise RuntimeError(...)` with the message beginning `"Learnable ratio {_ratio:.1%} < 5%."` The surrounding `elif _ratio < 0.10:` and `else:` branches print warning / OK messages. Only the `raise` branch changes.

- [ ] **Step 3: Apply the edit — replace `raise` with `print`**

Replace:

```python
if _ratio < 0.05:
    raise RuntimeError(
        f"Learnable ratio {_ratio:.1%} < 5%. Dataset is architecturally "
        f"mismatched; stop here and go to Phase 2 planning. Rebalancing "
        f"land labels cannot fix sub-floor obstacles."
    )
```

With:

```python
if _ratio < 0.05:
    print(
        f"[Phase 0] WARNING: learnable ratio {_ratio:.1%} < 5% — training anyway. "
        f"Below-floor labels are rate-limited at ingest to ~2x learnable count. "
        f"See spec: docs/superpowers/specs/2026-04-13-maritime-phase1-below-floor-ratelimit-design.md"
    )
```

Leave the `elif _ratio < 0.10:` and `else:` branches unchanged.

- [ ] **Step 4: Passing test — rerun the Phase 0 cell**

Rerun the Phase 0 cell. Expected output:

```
[Phase 0] Obstacle counts: {'learnable': 1238, 'below_floor': 69532, 'total': 70770}
[Phase 0] Learnable ratio: 1.7%
[Phase 0] WARNING: learnable ratio 1.7% < 5% — training anyway. ...
```

No RuntimeError. Training can now proceed.

Note: the counts `{'learnable': 1238, 'below_floor': 69532}` still reflect the **un-filtered** dataset at this point. Tasks 3 and 4 then rate-limit at data-build time, and Task 5 re-runs the pipeline to confirm the below-floor count drops.

---

## Task 3: Rate-limit below-floor labels in the MODD2 processor

**Files:**
- Modify: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — the MODD2 processor cell (header markdown says `# MODD2 Processor — curtain land + obstacle boxes`; the code defines `process_modd2(...)` or iterates `.mat` files and extracts `obstacles`).

- [ ] **Step 1: Failing test — inspect current below-floor count from MODD2**

Add a temporary counter just above the MODD2 processor's obstacle loop (or read it off the Phase 0 cell's output after data build). Expected: roughly the 69,532 figure is dominated by MODD2 obstacles (run + count to confirm). This measurement is the baseline you compare against in Step 4.

Keep this counter local / ephemeral — it's a pre-change measurement, not a permanent diagnostic. You'll remove it (or replace with a summary print) in Step 5.

- [ ] **Step 2: Locate the obstacle loop in the MODD2 processor**

The loop iterates the structured-array `obs = annots['obstacles']` and for each obstacle extracts `bx, by, bw, bh` (pixel coordinates in the source image) and calls:

```python
xc, yc, nw, nh = obstacle_to_yolo(bx, by, bw, bh, w, h)
```

`w` and `h` are the source image dimensions, in scope because they were read from the image earlier in the function.

- [ ] **Step 3: Insert the rate-limit gate just before the `obstacle_to_yolo` call**

Add these three lines immediately before `xc, yc, nw, nh = obstacle_to_yolo(...)`:

```python
# Rate-limit below-floor obstacles — see spec 2026-04-13-maritime-phase1-below-floor-ratelimit-design.md
if min(bw, bh) * (224 / max(w, h)) < 25 and random.random() > (2 / 56):
    continue
```

This computes the same min-side-at-train threshold Phase 0 uses (equivalent to `min(_w_train, _h_train) < 25`), in source-pixel form so no division or float precision drift. Skips with probability `1 - 2/56 ≈ 0.964` when below floor; always keeps learnable obstacles.

- [ ] **Step 4: Passing test — rerun the data-build flow up through Phase 0 and check the learnable count is unchanged**

Rerun all data-prep cells from `process_modd2` onward, then rerun Phase 0. Expected:

```
[Phase 0] Obstacle counts: {'learnable': 1238, 'below_floor': <SMALLER>, 'total': <SMALLER>}
```

The `learnable` count should stay **exactly 1238** (the gate only affects below-floor samples). The `below_floor` count should drop significantly — the final value after Task 4 (MoDS also rate-limited) is the relevant check. After this task alone, expect a partial drop proportional to MODD2's share of the total.

- [ ] **Step 5: Remove the temporary Step-1 counter if you added one**

Leave only the rate-limit gate in the processor. No leftover debug prints.

---

## Task 4: Rate-limit below-floor labels in the MoDS processor

**Files:**
- Modify: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — the MoDS processor cell (header says `# MoDS Processor — curtain land from mask + obstacle boxes`).

- [ ] **Step 1: Locate the MoDS obstacle extraction**

MoDS derives obstacle boxes from binary segmentation masks using `cv2.boundingRect(cnt)` on connected components (or a similar pixel-bbox extraction). The returned four-tuple `(x, y, w_box, h_box)` is the pixel bbox of the obstacle in the source image. This tuple then feeds `obstacle_to_yolo`, identical to MODD2.

Read the cell source. Identify the line where `cv2.boundingRect` (or the equivalent obstacle-pixel-bbox) is assigned, followed shortly by the `obstacle_to_yolo` call. If MoDS uses different variable names (e.g. `rx, ry, rw, rh`), adapt the gate accordingly in Step 2.

- [ ] **Step 2: Insert the rate-limit gate just before the `obstacle_to_yolo` call**

Add, using whatever the processor's pixel-bbox variable names are. If they match MODD2's (`bx, by, bw, bh`):

```python
# Rate-limit below-floor obstacles — see spec 2026-04-13-maritime-phase1-below-floor-ratelimit-design.md
if min(bw, bh) * (224 / max(w, h)) < 25 and random.random() > (2 / 56):
    continue
```

If they differ (e.g. `rw`, `rh` from `cv2.boundingRect`), substitute the correct names but preserve the formula exactly: `min(<pixel_w>, <pixel_h>) * (224 / max(<source_w>, <source_h>)) < 25`.

- [ ] **Step 3: Passing test — rerun the full data-build and Phase 0**

Rerun all data-prep cells. Then rerun Phase 0. Expected:

```
[Phase 0] Obstacle counts: {'learnable': 1238, 'below_floor': ~2500, 'total': ~3738}
[Phase 0] Learnable ratio: ~33.1%
[Phase 0] OK — enough learnable obstacles for Phase 1 to be meaningful.
```

Acceptance range: `below_floor` should be **2,200–2,800** (the random filter is stochastic around `2/56 * 69,532 ≈ 2,483`). `learnable` must be **exactly 1,238**. The `OK` branch is hit because the ratio is now well above 10 %.

If `below_floor` is wildly outside that range (e.g. 500 or 10,000), the gate is placed incorrectly or the formula diverged — re-read Task 3 / Task 4 Step 2 and compare the processor edits.

- [ ] **Step 4: Optional — save a checkpoint of the Phase 0 output for the ML log**

Copy the `[Phase 0] Obstacle counts:` and `Learnable ratio:` lines from Phase 0's output into a comment block at the top of the Phase 0 cell, so future reruns can be compared. Example:

```python
# Phase 0 reference values (post-rate-limit, seed=42):
#   learnable=1238, below_floor=~2483, total=~3721, ratio~33%
```

Purely informational. Not required for training.

---

## Task 5: Full end-to-end data build + training kickoff sanity check

**Files:**
- No file edits. This task is a runtime verification.

- [ ] **Step 1: Clear notebook state**

In Kaggle / Jupyter: kernel → Restart & Clear All. Ensures seed is applied from scratch and no stale state from earlier runs.

- [ ] **Step 2: Run all cells from the top through Phase 0**

Expected final output of the Phase 0 cell (counts approximate due to randomness):

```
[Phase 0] Obstacle counts: {'learnable': 1238, 'below_floor': ~2483, 'total': ~3721}
[Phase 0] Learnable ratio: ~33.3%
[Phase 0] OK — enough learnable obstacles for Phase 1 to be meaningful.
```

If this output is reproduced on a clean kernel run, the three edits have landed correctly and the seed is propagating. Ratio should be high-20s to mid-30s.

- [ ] **Step 3: Visual spot-check of the dataset samples cell**

The notebook has a samples visualisation cell (`plt.suptitle(f'Maritime {split} samples (red=obstacle, green=land)', ...)`). Run it. Confirm:
- Obstacle (red) boxes appear large enough to see at 224 — not dominated by tiny specks near the water line.
- Land (green) upper-frustum boxes still span the top half of frames with landmass.

If obstacles are still tiny-dominated, the rate-limit isn't firing — re-verify Tasks 3 and 4.

- [ ] **Step 4: Kick off training (optional — human decision whether to proceed now)**

Run the Phase 2 training cell. First 2–3 epochs should print:

```
[Phase 2] Training from scratch, batch=512, imgsz=224, epochs=1200 ...
Epoch  1/1200: box_loss=<x>, cls_loss=<y>, dfl_loss=<z>, val/mAP50=<v>
```

Box/cls losses decreasing over the first few epochs = training is moving. If mAP50 stays at 0.0 past epoch 50, something is upstream-broken (not this plan's fault).

Full 1,200 epochs will take many hours on a T4. The intent of this plan ends at "the data pipeline produces the expected counts and training starts." The actual `val/mAP50` number used for the pre-declared decision table (see spec) is gathered separately after training completes.

---

## Self-review checklist

Before handing this plan off for execution, verify:

- [x] Every task lists exact file paths.
- [x] Every edit shows exact old / new code where replacement is deterministic.
- [x] The formula `min(bw, bh) * (224 / max(w, h)) < 25` is identical in Tasks 3 and 4.
- [x] The `2 / 56` constant appears in Tasks 3, 4 and nowhere else.
- [x] Seed is set once (Task 1), in a cell that runs before the processors.
- [x] Phase 0 expected-output numbers in Task 4 Step 3 match the spec's success-criteria section (`~2,500` retained below-floor).
- [x] No git commit / git add steps (nested-git-under-tools rule).
- [x] Execution budget: Tasks 1–4 are each ≤5 min of editing; Task 5 is a re-run, another ~5 min; training is out-of-scope for this plan.
- [x] No TODOs, no TBDs, no placeholders.

---

## Out of scope (do not attempt in this plan)

- Collecting boat-captured validation frames.
- Building a custom crop augmentation module.
- Changing any training hyperparameter (the existing `mosaic=1.0`, `copy_paste=0.1`, default `scale=0.5`, default `translate=0.1` stay).
- Changing input size from 224 or the number of classes.
- Running the full 1,200-epoch training (human decision, after plan completes).

These are explicitly tracked elsewhere or deferred to a later phase per the governing spec.
