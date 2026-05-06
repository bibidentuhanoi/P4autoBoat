# Maritime Phase 1 — Land Rebalance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a dedicated Kaggle notebook cloned from `ESPDet_Pico_Maritime_Kaggle.ipynb` that replaces the 12-strip upper-frustum curtain with a single upper-frustum bbox per call, framed as a Phase 1 measurement experiment. Per-landmass granularity: MODD2 → 1 box/image, MoDS → 1 box per landmass contour.

**Architecture:** Develop `sea_edge_to_upper_frustum` under TDD in a self-contained pytest file, then apply it to the cloned notebook via a one-shot Python patch script. The script reads the verified function source via `importlib.util` + `inspect.getsource` and uses stdlib `json` to edit notebook cells. Notebook edits: insert the function in the helpers cell, swap two call-site patterns (MODD2 2-line pair → 1-line call, MoDS 2-line pair inside the loop → 1-line call), prepend a Phase 1 title header, insert a Phase 0 precondition cell.

**Tech Stack:** Python 3.12, numpy 2.x, pytest 9.x, stdlib `json` + `inspect` + `importlib.util`.

**Spec:** `docs/superpowers/specs/2026-04-11-maritime-phase1-land-rebalance-design.md`

**IMPORTANT — git policy for this project:** `tools/esp-detection/` is a nested git repo; parent repo cannot stage files inside it. **Skip all `git add` / `git commit` steps.** Just produce files on disk. The user handles version-tracking separately.

---

## File Structure

**Create (on disk only, no git):**
- `tools/esp-detection/test_sea_edge_upper_frustum.py` — self-contained pytest file containing the function definition + 9 unit tests. Single source of truth for the function.
- `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` — Phase 1 notebook, cloned from baseline and patched.

**Create (throwaway in /tmp):**
- `/tmp/apply_phase1_patches.py` — one-shot patch script.

**Modify:** None.

---

## Task 1: TDD `sea_edge_to_upper_frustum`

**Files:**
- Create: `tools/esp-detection/test_sea_edge_upper_frustum.py`

- [ ] **Step 1.1: Write the test file with a stub and all tests (red phase)**

Write `/workspaces/BoatEspP4/tools/esp-detection/test_sea_edge_upper_frustum.py` with this EXACT content:

```python
"""
Unit tests for sea_edge_to_upper_frustum — Phase 1 replacement for
get_curtains + curtains_to_yolo in the ESPDet-Pico maritime dataset pipeline.

This file is self-contained: it defines the function under test AND the
tests in one place so it can be developed TDD-style. The function gets
copy-inlined into ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb by
/tmp/apply_phase1_patches.py using inspect.getsource on this module.

Semantics: each call emits one YOLO label for a single axis-aligned box
from (0, 0) down to (img_w, max_waterline_y) — the "upper frustum" /
above-horizon no-go region. One call per landmass: MODD2 passes its
sea_edge polyline once per image; process_mods() passes each contour
once inside its existing loop.
"""

import numpy as np
import pytest


def sea_edge_to_upper_frustum(points, img_w, img_h):
    """STUB — replace in Step 1.3. Raises so tests fail red."""
    raise NotImplementedError("Stub — implement in Step 1.3")


# --- Tests ----------------------------------------------------------------

def test_empty_input_returns_empty_list():
    """Empty point array → []"""
    assert sea_edge_to_upper_frustum(np.zeros((0, 2), dtype=np.float32), 224, 224) == []


def test_single_point_returns_empty_list():
    """Fewer than 2 valid points → []"""
    pts = np.array([[100, 100]], dtype=np.float32)
    assert sea_edge_to_upper_frustum(pts, 224, 224) == []


def test_all_nan_points_returns_empty_list():
    """All-NaN input drops to zero valid points → []"""
    pts = np.array([[np.nan, np.nan], [np.nan, np.nan]], dtype=np.float32)
    assert sea_edge_to_upper_frustum(pts, 224, 224) == []


def test_waterline_at_image_top_returns_empty():
    """If the deepest waterline is ≤ 2px from the top, no meaningful
    no-go region exists — skip the label (matches get_curtains' trivial
    strip filter)."""
    pts = np.array([[0, 0], [640, 1]], dtype=np.float32)
    assert sea_edge_to_upper_frustum(pts, 640, 480) == []


def test_modd2_polyline_shape_accepted():
    """MODD2-style (N, 2) polyline produces one valid YOLO label line."""
    # sea_edge samples across the image width; deepest y is 110
    pts = np.array([[10, 100], [50, 105], [100, 110], [150, 108]], dtype=np.float32)
    result = sea_edge_to_upper_frustum(pts, 640, 480)
    assert len(result) == 1
    parts = result[0].split()
    assert parts[0] == "1"
    assert len(parts) == 5
    for p in parts[1:]:
        assert 0.0 <= float(p) <= 1.0


def test_cv2_contour_shape_accepted():
    """cv2 findContours shape (N, 1, 2) is accepted identically."""
    pts = np.array([[[10, 100]], [[50, 105]], [[100, 110]], [[150, 108]]], dtype=np.float32)
    result = sea_edge_to_upper_frustum(pts, 640, 480)
    assert len(result) == 1
    assert result[0].split()[0] == "1"


def test_frustum_spans_full_width_down_to_max_y():
    """Box must span x=0..img_w and y=0..max(waterline_y). Max y=110,
    img_h=480 → normalized: cx=0.5, cy=55/480, w=1.0, h=110/480."""
    pts = np.array([[10, 100], [50, 105], [100, 110], [150, 108]], dtype=np.float32)
    result = sea_edge_to_upper_frustum(pts, 640, 480)
    assert len(result) == 1
    _, cx, cy, w_norm, h_norm = result[0].split()
    assert float(cx) == pytest.approx(0.5, abs=1e-5)
    assert float(w_norm) == pytest.approx(1.0, abs=1e-5)
    assert float(h_norm) == pytest.approx(110 / 480, abs=1e-3)
    assert float(cy) == pytest.approx((110 / 2) / 480, abs=1e-3)


def test_uses_max_y_not_min_y_conservative():
    """For a polyline with widely varying y, the frustum extends to
    the LARGEST y (deepest waterline). This is the conservative-safe
    choice — over-include no-go rather than under-include land."""
    pts = np.array([[0, 100], [100, 200], [200, 150], [300, 300]], dtype=np.float32)
    result = sea_edge_to_upper_frustum(pts, 640, 480)
    assert len(result) == 1
    _, _, _, _, h_norm = result[0].split()
    # Max y = 300, so h = 300/480
    assert float(h_norm) == pytest.approx(300 / 480, abs=1e-3)


def test_waterline_clamped_to_image_height():
    """If points have y > img_h - 1, they get clamped so the frustum
    fits within the image."""
    pts = np.array([[0, 100], [640, 999]], dtype=np.float32)  # img 640x480
    result = sea_edge_to_upper_frustum(pts, 640, 480)
    assert len(result) == 1
    _, _, _, _, h_norm = result[0].split()
    # Clamped max = 479, so h = 479/480
    assert float(h_norm) == pytest.approx(479 / 480, abs=1e-3)
```

- [ ] **Step 1.2: Run pytest to verify the RED phase**

```bash
cd /workspaces/BoatEspP4/tools/esp-detection
python3 -m pytest test_sea_edge_upper_frustum.py -v
```

Expected: 9 tests collected, 9 FAILED, all with `NotImplementedError: Stub — implement in Step 1.3`.

**If you get fewer than 9 failures or any passes, stop and report — something is wrong.**

- [ ] **Step 1.3: Replace the stub with the real implementation (GREEN phase)**

Use the Edit tool on `/workspaces/BoatEspP4/tools/esp-detection/test_sea_edge_upper_frustum.py`. Replace this exact block:

```python
def sea_edge_to_upper_frustum(points, img_w, img_h):
    """STUB — replace in Step 1.3. Raises so tests fail red."""
    raise NotImplementedError("Stub — implement in Step 1.3")
```

with this exact block:

```python
def sea_edge_to_upper_frustum(points, img_w, img_h):
    """Option A replacement for get_curtains + curtains_to_yolo: one single
    upper-frustum bbox per call covering (0, 0) → (img_w, max_waterline_y).
    Semantically identical to the union of the 12 curtain strips — class-1
    "land" still means "above-horizon no-go region."

    Accepts MODD2 sea_edge polylines (N, 2) and cv2 contour arrays (N, 1, 2).
    One call per landmass: MODD2 passes its whole polyline once per image;
    process_mods() passes each contour once inside its loop.

    Returns [] on empty/invalid input.
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

- [ ] **Step 1.4: Run pytest to verify the GREEN phase**

```bash
cd /workspaces/BoatEspP4/tools/esp-detection
python3 -m pytest test_sea_edge_upper_frustum.py -v
```

Expected: `9 passed in <time>`. **If any test fails, stop and report — do not modify tests to force them to pass.**

- [ ] **Step 1.5: NO git commit**

Per project policy (tools/esp-detection is a nested git repo), **do not run `git add` or `git commit`**. The file exists on disk at `tools/esp-detection/test_sea_edge_upper_frustum.py` and that's where it needs to be for the patch script in Task 3.

---

## Task 2: Clone the source notebook

**Files:**
- Create: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` (copy of source)

- [ ] **Step 2.1: Verify source exists and is valid JSON**

```bash
ls -la /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle.ipynb
python3 -m json.tool /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle.ipynb > /dev/null && echo "valid JSON"
```

Expected: file listing showing size ~32KB, then `valid JSON`.

- [ ] **Step 2.2: Copy to Phase 1 path**

```bash
cp /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle.ipynb \
   /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb
```

- [ ] **Step 2.3: Verify the copy is valid JSON**

```bash
python3 -m json.tool /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb > /dev/null && echo "valid JSON"
```

Expected: `valid JSON`

---

## Task 3: Apply notebook patches via throwaway script

**Files:**
- Create: `/tmp/apply_phase1_patches.py` (throwaway)
- Modify: `tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb`

- [ ] **Step 3.1: Write the patch script**

Write `/tmp/apply_phase1_patches.py`:

```python
"""One-shot patcher for the Phase 1 maritime notebook.

Reads the verified sea_edge_to_upper_frustum from test_sea_edge_upper_frustum.py
via importlib + inspect.getsource, then applies four edits to the cloned
Phase 1 notebook using stdlib json:

  1. Append sea_edge_to_upper_frustum definition to the cell that currently
     contains def get_curtains (the "Detection Label Helpers" cell)
  2. Replace the two-line `curtains = get_curtains(...) /
     labels.extend(curtains_to_yolo(...))` pair with a single-line
     `labels.extend(sea_edge_to_upper_frustum(...))` call in both
     process_modd2 (once) and process_mods (once, inside its contour loop)
  3. Prepend a Phase 1 framing header to the first markdown cell
  4. Insert a Phase 0 precondition check code cell before the training cell

Idempotent: re-running the script skips edits that have already been applied.
"""

import importlib.util
import inspect
import json
import re
from pathlib import Path

BASE = Path("/workspaces/BoatEspP4/tools/esp-detection")
NOTEBOOK = BASE / "ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb"
TEST_FILE = BASE / "test_sea_edge_upper_frustum.py"

# --- Extract function source from the verified test file ------------------

spec = importlib.util.spec_from_file_location("_tested", TEST_FILE)
assert spec is not None and spec.loader is not None, f"cannot load {TEST_FILE}"
tested = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tested)
fn_src = inspect.getsource(tested.sea_edge_to_upper_frustum)
print(f"[+] Extracted sea_edge_to_upper_frustum ({len(fn_src)} chars) from {TEST_FILE.name}")

# --- Load the notebook ----------------------------------------------------

nb = json.loads(NOTEBOOK.read_text())
cells = nb["cells"]

def cell_src_text(cell):
    return "".join(cell.get("source", []))

def set_cell_src(cell, text):
    cell["source"] = text.splitlines(keepends=True)

# --- Edit 1: Insert function in the Detection Label Helpers cell ---------

helpers_idx = None
for i, c in enumerate(cells):
    if c["cell_type"] == "code" and "def get_curtains" in cell_src_text(c):
        helpers_idx = i
        break
assert helpers_idx is not None, "helpers cell (def get_curtains) not found"

helpers_src = cell_src_text(cells[helpers_idx])
if "def sea_edge_to_upper_frustum" in helpers_src:
    print(f"[=] sea_edge_to_upper_frustum already present in cell {helpers_idx}")
else:
    new_helpers = helpers_src.rstrip() + "\n\n\n" + fn_src
    set_cell_src(cells[helpers_idx], new_helpers)
    print(f"[+] Inserted sea_edge_to_upper_frustum into cell {helpers_idx}")

# --- Edit 2: Replace the 2-line curtain pair with a 1-line upper-frustum -

# The pattern we're looking for:
#     curtains = get_curtains(<arg>, w, h)
#     labels.extend(curtains_to_yolo(curtains, w, h))
# Replaced with:
#     labels.extend(sea_edge_to_upper_frustum(<arg>, w, h))
#
# Regex handles optional leading whitespace (to preserve indentation) and
# any legal Python identifier/expression for <arg>.

CURTAIN_PATTERN = re.compile(
    r"^([ \t]*)curtains\s*=\s*get_curtains\((.+?),\s*w,\s*h\)\s*\n"
    r"[ \t]*labels\.extend\(curtains_to_yolo\(curtains,\s*w,\s*h\)\)",
    re.MULTILINE,
)

swapped_total = 0
for i, c in enumerate(cells):
    if c["cell_type"] != "code":
        continue
    src = cell_src_text(c)
    if "def process_modd2" not in src and "def process_mods" not in src:
        continue

    def _repl(m):
        indent = m.group(1)
        arg = m.group(2)
        return f"{indent}labels.extend(sea_edge_to_upper_frustum({arg}, w, h))"

    new_src, n = CURTAIN_PATTERN.subn(_repl, src)
    if n > 0:
        set_cell_src(c, new_src)
        print(f"[+] Swapped {n} curtain pattern(s) in cell {i}")
        swapped_total += n

print(f"[+] Total pattern swaps this run: {swapped_total}")
assert swapped_total in (0, 2), f"Expected 0 or 2 swaps, got {swapped_total}"

# --- Edit 3: Prepend Phase 1 framing to the title markdown cell ----------

title_idx = None
for i, c in enumerate(cells):
    if c["cell_type"] == "markdown":
        title_idx = i
        break
assert title_idx is not None, "no markdown cell found"

phase1_header = """# ESPDet-Pico Maritime — Phase 1: Land Rebalance Experiment

**Phase 1 goal:** measure whether consolidating the 12-strip upper-frustum
curtain into one upper-frustum box per landmass recovers obstacle detection
recall on ESPDet-Pico at 224x224. This is a **measurement experiment**,
not a claim of a fix — if obstacle mAP50 stays below 3%, the architectural
floor hypothesis is confirmed and Phase 2 planning begins.

**Changes vs the baseline maritime notebook:**
- `get_curtains` + `curtains_to_yolo` replaced by `sea_edge_to_upper_frustum`
  (one full-width upper-frustum bbox per call: (0, 0) → (img_w, max_waterline_y))
- MODD2: 1 land label per image; MoDS: 1 land label per landmass contour
  (1-3 typical). Aggregate ~8x rebalancing of land:obstacle gradient share.
- Phase 0 precondition check cell: counts training obstacles above the
  stride-8 minimum size and raises if < 5% to prevent wasted Kaggle sessions
  on architecturally-mismatched datasets.

Everything else (dataset download, training hyperparameters, QAT,
calibration, export) is identical to `ESPDet_Pico_Maritime_Kaggle.ipynb`.

See `docs/superpowers/specs/2026-04-11-maritime-phase1-land-rebalance-design.md`
for success criteria and Phase 2 pointers.

---

"""

existing_title = cell_src_text(cells[title_idx])
if "Phase 1: Land Rebalance" in existing_title:
    print(f"[=] Title cell {title_idx} already has Phase 1 header")
else:
    set_cell_src(cells[title_idx], phase1_header + existing_title)
    print(f"[+] Prepended Phase 1 header to title cell {title_idx}")

# --- Edit 4: Insert Phase 0 precondition cell before the training cell ---

train_idx = None
for i, c in enumerate(cells):
    if c["cell_type"] == "code" and "model.train(" in cell_src_text(c):
        train_idx = i
        break
assert train_idx is not None, "training cell (model.train(...)) not found"

phase0_marker = "Phase 0: Precondition check"
already_has_phase0 = any(phase0_marker in cell_src_text(c) for c in cells)

if already_has_phase0:
    print(f"[=] Phase 0 cell already present")
else:
    phase0_code = '''# Phase 0: Precondition check — are there obstacles large enough to survive 224x224 resize?
# At stride-8, the detector's effective receptive floor is ~25 training pixels.
# For a 1278-wide source resized to 224, that back-converts to ~143 source px.
# If most training obstacles are below that threshold, the retrain cannot learn
# them regardless of label rebalancing.
from pathlib import Path
import cv2 as _phase0_cv2

STRIDE_8_FLOOR_TRAIN_PX = 25
_counts = {"learnable": 0, "below_floor": 0, "total": 0}
for _lbl_path in Path(OUTPUT_DIR, "labels", "train").glob("*.txt"):
    _img_path = Path(OUTPUT_DIR, "images", "train") / (_lbl_path.stem + ".jpg")
    if not _img_path.exists():
        continue
    _img = _phase0_cv2.imread(str(_img_path))
    if _img is None:
        continue
    _h_src, _w_src = _img.shape[:2]
    _scale = 224 / max(_w_src, _h_src)
    for _line in _lbl_path.read_text().splitlines():
        _parts = _line.split()
        if len(_parts) != 5:
            continue
        _cls, _xc, _yc, _w, _h = _parts
        if int(_cls) != 0:  # obstacles only (class 0)
            continue
        _w_train = float(_w) * _w_src * _scale
        _h_train = float(_h) * _h_src * _scale
        _counts["total"] += 1
        if min(_w_train, _h_train) >= STRIDE_8_FLOOR_TRAIN_PX:
            _counts["learnable"] += 1
        else:
            _counts["below_floor"] += 1

_ratio = _counts["learnable"] / _counts["total"] if _counts["total"] else 0.0
print(f"[Phase 0] Obstacle counts: {_counts}")
print(f"[Phase 0] Learnable ratio: {_ratio:.1%}")
if _ratio < 0.05:
    raise RuntimeError(
        f"Learnable ratio {_ratio:.1%} < 5%. Dataset is architecturally "
        f"mismatched; stop here and go to Phase 2 planning. Rebalancing "
        f"land labels cannot fix sub-floor obstacles."
    )
elif _ratio < 0.20:
    print("[Phase 0] WARNING: borderline ratio. Expect weak signal from Phase 1.")
else:
    print("[Phase 0] OK — enough learnable obstacles for Phase 1 to be meaningful.")
'''
    phase0_cell = {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": phase0_code.splitlines(keepends=True),
    }
    cells.insert(train_idx, phase0_cell)
    print(f"[+] Inserted Phase 0 cell at position {train_idx}")

# --- Save -----------------------------------------------------------------

NOTEBOOK.write_text(json.dumps(nb, indent=1) + "\n")
print(f"[+] Wrote {NOTEBOOK}")
```

- [ ] **Step 3.2: Run the patch script**

```bash
cd /workspaces/BoatEspP4
python3 /tmp/apply_phase1_patches.py
```

Expected output includes:
```
[+] Extracted sea_edge_to_upper_frustum (~1000 chars) from test_sea_edge_upper_frustum.py
[+] Inserted sea_edge_to_upper_frustum into cell <N>
[+] Swapped 1 curtain pattern(s) in cell <M>   (process_modd2)
[+] Swapped 1 curtain pattern(s) in cell <K>   (process_mods)
[+] Total pattern swaps this run: 2
[+] Prepended Phase 1 header to title cell 0
[+] Inserted Phase 0 cell at position <P>
[+] Wrote /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb
```

The assertion `swapped_total in (0, 2)` must pass. If it fails with a different number, the notebook has an unexpected structure — stop and investigate.

- [ ] **Step 3.3: Verify the notebook is still valid JSON**

```bash
python3 -m json.tool /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb > /dev/null && echo "valid JSON"
```

Expected: `valid JSON`

- [ ] **Step 3.4: Grep-verify function definition, call-site swaps, and removed patterns**

```bash
cd /workspaces/BoatEspP4/tools/esp-detection
echo "sea_edge_to_upper_frustum occurrences (should be >= 3: def + 2 call sites):"
grep -o "sea_edge_to_upper_frustum" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb | wc -l
echo "get_curtains call occurrences (should be 0 — all swapped, only def remains):"
grep -o "= get_curtains(" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb | wc -l
echo "def get_curtains occurrences (should be 1 — old function left as reference):"
grep -o "def get_curtains" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb | wc -l
echo "curtains_to_yolo(curtains, call occurrences (should be 0):"
grep -o "curtains_to_yolo(curtains" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb | wc -l
echo "def curtains_to_yolo occurrences (should be 1 — old function left as reference):"
grep -o "def curtains_to_yolo" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb | wc -l
```

Expected:
- `sea_edge_to_upper_frustum` occurrences: **≥ 3** (1 def + 2 call sites)
- `= get_curtains(` call occurrences: **0**
- `def get_curtains` occurrences: **1**
- `curtains_to_yolo(curtains` call occurrences: **0**
- `def curtains_to_yolo` occurrences: **1**

- [ ] **Step 3.5: Grep-verify Phase 0 cell and title header**

```bash
cd /workspaces/BoatEspP4/tools/esp-detection
echo "Phase 0 marker (should be 1):"
grep -c "Phase 0: Precondition check" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb
echo "Phase 1 title header (should be >= 1):"
grep -c "Phase 1: Land Rebalance Experiment" ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb
```

Expected:
- Phase 0 marker: **1**
- Phase 1 title header: **≥ 1**

- [ ] **Step 3.6: Parse the notebook in Python as a final sanity check**

```bash
python3 -c "
import json
nb = json.loads(open('/workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb').read())
print(f'Cells: {len(nb[\"cells\"])}')
types = [c['cell_type'] for c in nb['cells']]
print(f'Markdown: {types.count(\"markdown\")}, Code: {types.count(\"code\")}')
"
```

Expected: non-zero cell count, mix of markdown and code cell types, no exceptions.

---

## Task 4: Final verification on disk

No git operations. Just confirm both output files exist and are in a correct state.

- [ ] **Step 4.1: List the created files**

```bash
ls -la /workspaces/BoatEspP4/tools/esp-detection/test_sea_edge_upper_frustum.py /workspaces/BoatEspP4/tools/esp-detection/ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb
```

Expected: both files exist.

- [ ] **Step 4.2: Re-run the tests one final time**

```bash
cd /workspaces/BoatEspP4/tools/esp-detection
python3 -m pytest test_sea_edge_upper_frustum.py -v
```

Expected: `9 passed`.

---

## Post-execution

After this plan is complete, the implementation is done locally. Kaggle-side work remains:

1. Upload `ESPDet_Pico_Maritime_Kaggle_Phase1.ipynb` to Kaggle.
2. Run all cells. Phase 0 gates the training — if learnable ratio < 5%, the RuntimeError halts the session.
3. Capture the per-class mAP table from the training cell's final output.
4. Compare against the baseline curtain run:
   - **obstacle mAP50 > 10%** + **land mAP50 ≥ 60%** → ship the retrained model
   - **obstacle mAP50 in [3%, 10%]** → ship labels, begin Phase 2 planning
   - **obstacle mAP50 < 3%** → do not ship model; architectural floor confirmed

The bench script (`/tmp/bench_maritime.py`) redesign is explicitly out of scope; if you re-run it on the new model, focus on the obstacle column only and expect land precision to look artificially low (strip-matching against a 1-box-per-landmass GT will undercount).

Phase 2 is a separate spec and plan when we get there.
