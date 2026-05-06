# ToF + BBox Fusion in Detect Modal — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Inside the existing detect-result modal (`main/dashboard.html`), render a per-bbox ToF heatmap (clipped to each detection rectangle) and a side panel listing class label + best-sigma distance per detection.

**Architecture:** All changes confined to `main/dashboard.html` (firmware unchanged). At Detect-click time, snapshot the latest WS-received `tof_a`/`tof_b` plus all projection params (AZ_OFFSET, EL_OFFSET, CX, CY, GRID_SCALE, FX, flip toggles, drag offset). When detections arrive via the existing 15s firmware cache, project all 64 ToF zones per sensor into snapshot pixel space, filter to zones whose centroid lies inside each bbox AND have valid `target_status`, auto-pick the sensor with more overlapping zones, pick the lowest-σ zone in the winning pool as the reported distance, render heatmap fills clipped per-bbox, and populate side-panel rows.

**Tech Stack:** Vanilla JS in single embedded HTML, Canvas 2D, no test framework. Verification is on-device: `idf.py build && idf.py flash`, then click Detect in browser at `http://192.168.1.201/`. Reference spec: `docs/superpowers/specs/2026-04-13-tof-bbox-fusion-design.md`.

**Key existing code to reuse:**
- `renderOverlay()` at `main/dashboard.html:536` — projection math sits inline here. Lines ~360-380 contain the camera-projection (FX, CX, CY) used by `projectZone`-style logic. Identify the per-zone polygon code and extract it.
- `drawHeatmap()` at `main/dashboard.html:660` — current full-frame heatmap renderer; its color-ramp logic must be reused for the new bbox-clipped variant.
- `snapCamToDetectCanvas()` at `main/dashboard.html:~954` — async function that fetches `/snapshot` and draws into `#detect-canvas`. Call site of new ToF capture.
- `renderDetections()` at `main/dashboard.html:~975` — currently draws bbox + label only. Will be extended to call new heatmap + side-panel renderers.
- `$('detect-btn').addEventListener('click', ...)` at `main/dashboard.html:~1014` — click handler. Will be extended to call new state-capture function before sending DetectCommand.
- Detect modal HTML at `main/dashboard.html:~236` — `#detect-backdrop` > `#detect-panel` > `<canvas id="detect-canvas">`. Side panel + legend get added here.

**Always after each task:** stop the firmware-event-monitor first (it holds the serial port) → `idf.py build && idf.py flash` → restart monitor → hard-refresh dashboard → verify behavior described in the task's verification step.

---

### Task 1: Refactor zone projection into a pure function

**Why first:** Both the existing `renderOverlay` and the new bbox pipeline need the same projection math. Doing this as a pure function keeps the new feature independent of dashboard rendering side effects.

**Files:**
- Modify: `main/dashboard.html` — extract the `(row, col) → 4-corner polygon in image pixel space` logic out of `renderOverlay` into a pure function `projectZone(row, col, sensorKey, params)` placed just above `renderOverlay`.

- [ ] **Step 1: Read current projection code.** Open `main/dashboard.html`, read lines 300–430 (constants + projection helpers) and lines 536–660 (`renderOverlay` body). Note exactly which inputs the per-zone polygon math reads (AZ_OFFSET, EL_OFFSET, CX, CY, GRID_SCALE, FX, ZONE_STEP, flipH/V/T per sensor).

- [ ] **Step 2: Add the pure function** above `renderOverlay`:

```js
// Pure: maps a ToF zone (row,col) on sensorKey ('A'|'B') into snapshot pixel space.
// `params` snapshots all projection state at a moment in time so the result is
// reproducible after live sliders/drags move on. Returns 4 corner points
// [{x,y}, {x,y}, {x,y}, {x,y}] in [0..800]×[0..640] coords (snapshot canvas units).
function projectZone(row, col, sensorKey, p) {
  const flipH = sensorKey === 'A' ? p.flipHA : p.flipHB;
  const flipV = sensorKey === 'A' ? p.flipVA : p.flipVB;
  const trans = sensorKey === 'A' ? p.transA  : p.transB;
  const r = trans ? col : row;
  const c = trans ? row : col;
  const rr = flipV ? 7 - r : r;
  const cc = flipH ? 7 - c : c;
  // For each of the four zone corners (rr..rr+1, cc..cc+1):
  const corners = [];
  for (const [dr, dc] of [[0,0],[0,1],[1,1],[1,0]]) {
    const az = ((cc + dc) - 4) * p.ZONE_STEP + p.AZ_OFFSET;
    const el = (4 - (rr + dr)) * p.ZONE_STEP + p.EL_OFFSET;
    const azR = az * Math.PI / 180;
    const elR = el * Math.PI / 180;
    const wx = Math.sin(azR) * Math.cos(elR);
    const wy = Math.sin(elR);
    const wz = Math.cos(azR) * Math.cos(elR);
    if (wz <= 0) { corners.push(null); continue; }
    const u = p.FX * (wx / wz) + p.CX + p.dragOffX;
    const v = -p.FX * (wy / wz) + p.CY + p.dragOffY;
    const x = p.CX + (u - p.CX) * p.GRID_SCALE;
    const y = p.CY + (v - p.CY) * p.GRID_SCALE;
    corners.push({ x, y });
  }
  return corners.every(c => c !== null) ? corners : null;
}
```

(Adjust constant names to match what's actually in the file — the engineer should confirm by reading lines 300–430 first. The structure above mirrors the existing inline math.)

- [ ] **Step 3: Replace inline projection in `renderOverlay`** with calls to `projectZone(row, col, sensorKey, currentParams)` where `currentParams` is built from the live globals (AZ_OFFSET, EL_OFFSET, CX, CY, GRID_SCALE, FX, ZONE_STEP, flips, drag offsets). The behavior must be IDENTICAL — no visual change in the live overlay.

- [ ] **Step 4: Verify live overlay unchanged.** Build and flash. Open dashboard. The 3D-Threshold overlay (default mode) and Heatmap mode (toggle via gear → Overlay → Heatmap) must look identical to before. If a zone moves by even one pixel, the refactor is wrong — fix before continuing.

- [ ] **Step 5: Commit.**

```bash
git add main/dashboard.html
git commit -m "refactor(dashboard): extract projectZone into a pure function for reuse"
```

---

### Task 2: Snapshot ToF + projection params at click time

**Why:** Make the snapshotted detect frame coherent: the camera frame, ToF data, and projection params all reflect the moment of the click — even if the user drags sliders or fresh ToF data arrives later.

**Files:**
- Modify: `main/dashboard.html` — add module-scope state vars near the existing `_detectInFlight` declarations (around line 949), and a `captureTofSnapshot()` helper. Call it from the detect-btn click handler before `await snapCamToDetectCanvas()`.

- [ ] **Step 1: Add state vars** near `let _detectInFlight = false;`:

```js
let _snapTofA  = null;   // deep clone of last lastTofA at click-time
let _snapTofB  = null;
let _snapProj  = null;   // {AZ_OFFSET, EL_OFFSET, CX, CY, GRID_SCALE, FX, ZONE_STEP,
                         //  flipHA, flipVA, transA, flipHB, flipVB, transB,
                         //  dragOffX, dragOffY}
```

- [ ] **Step 2: Add `captureTofSnapshot`** below `snapCamToDetectCanvas`. NOTE: `captureProjectionParams()` already exists in the file (added during Task 1) — just reuse it:

```js
function captureTofSnapshot() {
  // structuredClone is fine for plain JSON-shaped protobuf-decoded objects.
  _snapTofA = lastTofA ? structuredClone(lastTofA) : null;
  _snapTofB = lastTofB ? structuredClone(lastTofB) : null;
  _snapProj = captureProjectionParams();
}
```

(Confirm the actual variable names by reading the area around the existing `lastTofA` and `FLIP` definitions; substitute if different.)

- [ ] **Step 3: Call from click handler.** In `$('detect-btn').addEventListener('click', async () => { ... })` at ~line 1014, add immediately before `await snapCamToDetectCanvas();`:

```js
  captureTofSnapshot();
```

- [ ] **Step 4: Verify in DevTools console.** Build, flash, hard-refresh. Click Detect. In console: `_snapTofA`, `_snapTofB`, `_snapProj` should be populated; click again 5 seconds later (after camera has moved) and the same vars should reflect the NEW state — proving they snap fresh per click.

- [ ] **Step 5: Commit.**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): snapshot ToF + projection params at detect-click"
```

---

### Task 3: Add `projectZonesIntoBbox` + best-sigma aggregation

**Files:**
- Modify: `main/dashboard.html` — new function placed near `projectZone`.

- [ ] **Step 1: Add the function:**

```js
// For one ToF grid + bbox + projection params, returns:
//   { count, bestZone: {distance_mm, sigma_mm}|null, polygons: [{poly, distance_mm}, ...] }
// Filters to zones whose CENTROID lies inside bbox AND target_status ∈ {5,9} AND distance > 0.
// `bbox` is {x1, y1, x2, y2} in snapshot pixel coords.
// `sensorKey` is 'A' or 'B'.
function projectZonesIntoBbox(tof, bbox, proj, sensorKey) {
  if (!tof || !tof.valid || !tof.distances) return { count: 0, bestZone: null, polygons: [] };
  const polygons = [];
  let bestZone = null;
  // Iterate the 64 zones; for multi-target, pick the closest valid target per zone (j=0 since sorted)
  for (let row = 0; row < 8; row++) {
    for (let col = 0; col < 8; col++) {
      const idx = (row * 8 + col) * 4;  // 4 targets/zone in current proto
      const status = tof.targetStatus ? tof.targetStatus[idx] : 0;
      const dist   = tof.distances    ? tof.distances[idx]    : 0;
      const sigma  = tof.sigma        ? tof.sigma[idx]        : 0;
      if (dist <= 0 || (status !== 5 && status !== 9)) continue;
      const poly = projectZone(row, col, sensorKey, dist, proj);  // signature: (row, col, sensorKey, mm, params)
      if (!poly) continue;
      const cx = (poly[0].x + poly[1].x + poly[2].x + poly[3].x) / 4;
      const cy = (poly[0].y + poly[1].y + poly[2].y + poly[3].y) / 4;
      if (cx < bbox.x1 || cx > bbox.x2 || cy < bbox.y1 || cy > bbox.y2) continue;
      polygons.push({ poly, distance_mm: dist, sigma_mm: sigma });
      if (!bestZone || sigma < bestZone.sigma_mm) bestZone = { distance_mm: dist, sigma_mm: sigma };
    }
  }
  return { count: polygons.length, bestZone, polygons };
}
```

(The exact field names — `targetStatus` vs `target_status` — depend on protobufjs camelCase setting. Verify by checking how `lastTofA.distances` is accessed elsewhere in the file; match that style.)

- [ ] **Step 2: Add sensor auto-pick:**

```js
function pickSensorForBbox(bbox) {
  const a = projectZonesIntoBbox(_snapTofA, bbox, _snapProj, 'A');
  const b = projectZonesIntoBbox(_snapTofB, bbox, _snapProj, 'B');
  if (a.count === 0 && b.count === 0) return { sensor: null, ...a, polygons: [] };
  if (a.count >= b.count) return { sensor: 'A', ...a };
  if (b.count > a.count)  return { sensor: 'B', ...b };
  // tie: merge both pools, recompute bestZone
  const merged = [...a.polygons, ...b.polygons];
  let best = null;
  for (const p of merged) if (!best || p.sigma_mm < best.sigma_mm) best = { distance_mm: p.distance_mm, sigma_mm: p.sigma_mm };
  return { sensor: 'A+B', count: merged.length, bestZone: best, polygons: merged };
}
```

- [ ] **Step 3: Console-test.** Build, flash, hard-refresh. Click Detect (with cat in frame so we get a bbox). In DevTools console after the modal opens: paste `pickSensorForBbox({x1: 200, y1: 100, x2: 500, y2: 500})`. Expect a non-empty result with `count > 0`, a `bestZone` object, and `polygons` array. If `count === 0`, the bbox doesn't overlap the ToF FOV — try a wider bbox or aim the ToF at the test scene.

- [ ] **Step 4: Commit.**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): projectZonesIntoBbox + best-sigma sensor auto-pick"
```

---

### Task 4: CSS + HTML for sidepanel and legend

**Files:**
- Modify: `main/dashboard.html` — extend `#detect-panel` CSS to flex-row, add `#detect-sidepanel` + `#detect-legend` styles, insert the sidepanel and legend HTML next to `#detect-canvas` inside `#detect-panel`.

- [ ] **Step 1: CSS updates.** Find the `#detect-panel` rule (~line 102) and the related rules. Modify so the panel is flex-row (canvas left, sidepanel right):

```css
#detect-panel { background: var(--panel); border: 1px solid var(--border); border-radius: 6px; max-width: 95vw; max-height: 90vh; display: flex; flex-direction: column; overflow: hidden; }
#detect-body  { display: flex; flex-direction: row; align-items: stretch; }
#detect-canvas { display: block; max-width: 70vw; max-height: 70vh; background: #000; }
#detect-sidepanel { flex: 0 0 200px; padding: 8px 10px; border-left: 1px solid var(--border); overflow-y: auto; font-size: 11px; color: var(--text); }
#detect-sidepanel .sp-row { padding: 6px 0; border-bottom: 1px dashed var(--border); }
#detect-sidepanel .sp-row:last-child { border-bottom: none; }
#detect-sidepanel .sp-class { color: var(--accent); font-weight: bold; }
#detect-sidepanel .sp-dist  { color: var(--text); font-size: 14px; }
#detect-sidepanel .sp-meta  { color: var(--dim); font-size: 9px; letter-spacing: 1px; }
#detect-sidepanel .sp-empty { color: var(--warn); font-style: italic; }
#detect-legend { padding: 6px 10px; border-top: 1px solid var(--border); font-size: 9px; color: var(--dim); }
#detect-legend .bar { height: 8px; background: linear-gradient(to right, hsl(0,80%,45%), hsl(60,80%,45%), hsl(120,80%,45%), hsl(200,80%,45%)); border-radius: 2px; margin-top: 2px; }
#detect-legend .ticks { display: flex; justify-content: space-between; }
```

- [ ] **Step 2: HTML structure update.** Find the existing `<canvas id="detect-canvas">` block (~line 243). Wrap it and the new sidepanel in `<div id="detect-body">`:

```html
<div id="detect-body">
  <canvas id="detect-canvas" width="800" height="640"></canvas>
  <div id="detect-sidepanel">
    <div class="sp-empty">— no detections yet</div>
  </div>
</div>
<div id="detect-legend">
  <div class="ticks"><span>0 m</span><span>1 m</span><span>2 m</span><span>3 m</span><span>4+ m</span></div>
  <div class="bar"></div>
</div>
```

- [ ] **Step 3: Verify layout.** Build, flash, refresh, click Detect. Modal should now be wider, with empty sidepanel on right + legend bar at bottom. Snapshot frame and bbox overlay still render correctly in canvas.

- [ ] **Step 4: Commit.**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): detect modal sidepanel + legend layout"
```

---

### Task 5: Render bbox-clipped ToF zones — 3D-Threshold style (minimal)

**Design intent (user direction 2026-04-13):** Match the existing 3D-Threshold overlay's per-zone look — colored filled quad + stroke + sigma-weighted opacity — but **MINIMAL**: no crosshair, no per-zone distance label, no yellow blend dot, no magenta edge circle, no "CLOSEST" indicator. Just the filled quads with outline, clipped to the bbox.

**Color source:** Reuse the existing `dangerColor(mm, thresholdMM)` function from `drawThreshold` (already in the file, ~line 619). Same color ramp as the 3D-Threshold overlay so the user reads bbox colors with the existing mental model.

**Files:**
- Modify: `main/dashboard.html` — add `renderToFInBbox` near `projectZone`; modify `renderDetections` to invoke it per bbox. NO new color helper — reuse `dangerColor`.

- [ ] **Step 1: Update `projectZonesIntoBbox` (Task 3) to also return per-zone `sigma_mm`.** It already keeps sigma in the polygons array. Verify the `polygons` items are `{poly, distance_mm, sigma_mm}` — this is what Task 5 needs to compute opacity.

- [ ] **Step 2: Add `renderToFInBbox`:**

```js
// Renders ToF zones inside a bbox, 3D-Threshold style (minimal — no labels/markers).
// `pick.polygons` items are {poly, distance_mm, sigma_mm}.
// Reuses the existing dangerColor() helper so colors match the live overlay.
function renderToFInBbox(ctx, bbox, polygons) {
  if (!polygons.length) return;
  ctx.save();
  ctx.beginPath();
  ctx.rect(bbox.x1, bbox.y1, bbox.x2 - bbox.x1, bbox.y2 - bbox.y1);
  ctx.clip();
  ctx.setLineDash([]);
  for (const { poly, distance_mm, sigma_mm } of polygons) {
    if (distance_mm > thresholdMM) continue;  // honor the user's distance threshold
    const colors = dangerColor(distance_mm, thresholdMM);
    // Sigma-weighted opacity: low sigma = high confidence = more opaque.
    // Match drawThreshold's existing scheme (clamp sigma to [10, 200] mm).
    const sNorm = Math.max(0, Math.min(1, 1 - (sigma_mm - 10) / 190));
    ctx.globalAlpha = 0.4 + 0.5 * sNorm;  // [0.4 .. 0.9]
    ctx.fillStyle = colors.fill;
    ctx.beginPath();
    ctx.moveTo(poly[0].x, poly[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(poly[i].x, poly[i].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = colors.stroke;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
  ctx.globalAlpha = 1.0;
  ctx.restore();
}
```

(Verify `dangerColor`'s actual signature and `thresholdMM`'s scope by reading lines around line 619. If sigma-opacity formula differs in `drawThreshold`, copy the exact one used there to keep visual parity.)

- [ ] **Step 3: Wire into `renderDetections`.** Find `renderDetections(dets)` (~line 975). For each detection, BEFORE drawing the bbox stroke + class label, compute:

```js
const bbox = { x1: det.x1, y1: det.y1, x2: det.x2, y2: det.y2 };
const pick = pickSensorForBbox(bbox);
renderToFInBbox(ctx, bbox, pick.polygons);
```

Cache `{ det, pick }` per detection in a local array — Task 6 needs it.

- [ ] **Step 4: Verify on device.** Build, flash, refresh, point camera at cat, click Detect. The bbox should contain colored quads matching the 3D-Threshold style — same colors as the live overlay would show in that region. Quads must STAY INSIDE the bbox (clip working). No labels, no crosshairs, no extra markers — just colored quads with outlines.

- [ ] **Step 5: Commit.**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): clipped 3D-Threshold-style ToF zones inside detect bboxes"
```

---

### Task 6: Render side-panel rows

**Files:**
- Modify: `main/dashboard.html` — new `renderSidePanel(detsWithPicks)` function; modify `renderDetections` to call it after drawing all bboxes; modify `openDetectModal` zero-detection branch to also reset the sidepanel.

- [ ] **Step 1: Add the function:**

```js
function renderSidePanel(rows) {
  const sp = $('detect-sidepanel');
  if (!rows || rows.length === 0) {
    sp.innerHTML = '<div class="sp-empty">— no detections</div>';
    return;
  }
  sp.innerHTML = '';
  for (const { det, pick } of rows) {
    const row = document.createElement('div');
    row.className = 'sp-row';
    const dist = pick.bestZone
      ? `<div class="sp-dist">${(pick.bestZone.distance_mm / 1000).toFixed(2)} m  <span class="sp-meta">best-σ ±${pick.bestZone.sigma_mm}mm</span></div>
         <div class="sp-meta">${pick.count} zones · sensor ${pick.sensor}</div>`
      : `<div class="sp-empty">— no ToF coverage</div>`;
    row.innerHTML = `
      <div class="sp-class">cat${det.category} · ${(det.score * 100).toFixed(0)}%</div>
      ${dist}`;
    sp.appendChild(row);
  }
}
```

- [ ] **Step 2: Modify `renderDetections`** to collect `{det, pick}` rows during the bbox loop and call `renderSidePanel(rows)` at the end. Sort `rows` by `det.score` descending before calling.

- [ ] **Step 3: Modify `openDetectModal`** zero-detections branch to also call `renderSidePanel([])` so the panel resets between cycles.

- [ ] **Step 4: Verify on device.** Build, flash, refresh, click Detect. Side panel should show one row per detection: `cat0 · 62%` heading, `1.42 m best-σ ±N mm`, `12 zones · sensor A`. Repeat with camera aimed away from any ToF coverage area — row should show `— no ToF coverage`.

- [ ] **Step 5: Commit.**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): detect modal side-panel with class + best-σ distance per bbox"
```

---

### Task 7: End-to-end verification + memory update

**Files:**
- None modified. Just build, flash, test, document.

- [ ] **Step 1: End-to-end test.** Hard-refresh dashboard. Click Detect with cat in frame. Verify the modal contains:
  - Snapshot frame (existing).
  - Green bbox outline + `cat0 NN%` label (existing).
  - Heatmap fill inside the bbox, clipped to bbox edges (new).
  - Side panel with one row per detection: class, score, distance, zone count, sensor letter (new).
  - Legend bar at bottom of modal (new).

- [ ] **Step 2: Verify edge case — no ToF coverage.** Aim the camera so the cat is in the upper part of the frame where the ToF FOV may not reach. Detect. Side panel row should read `— no ToF coverage` and no heatmap should render for that bbox.

- [ ] **Step 3: Verify edge case — no detections.** Cover the camera. Detect. Modal should show snapshot + no bboxes + side panel saying `— no detections`. (After 15s timeout if firmware sends no result.)

- [ ] **Step 4: Update memory.** Append to `/root/.claude/projects/-workspaces-BoatEspP4/memory/last_known_good_baseline.md` the new commit hash + 1-line summary of the new feature. Remove the "ToF + bbox fusion" item from the open-polish list if it was there.

- [ ] **Step 5: Final commit.** If any memory or doc changes were made:

```bash
git add -A
git commit -m "docs(memory): record ToF+bbox fusion baseline + remove from open-items list"
```

---

## Self-Review Notes

- **Spec coverage:** All 8 design decisions in the spec map to tasks: #1 best-σ → Task 3; #2 heatmap+transparent → Task 5; #3 right sidepanel → Task 4; #4 ToF freshness → Task 2; #5 multi-bbox → Task 6 (loop); #6 no coverage → Tasks 5/6 (skip path); #7 sensor auto-pick → Task 3 (`pickSensorForBbox`); #8 reuse projection params → Task 2 (snapshot into `_snapProj`).
- **Refactor risk** flagged in spec → Task 1 dedicates a verification step requiring identical visual output of the live overlay before continuing.
- **No placeholders, no TBDs, no "similar to Task N".** Code blocks are self-contained.
- **Type/name consistency:** `bbox` shape = `{x1,y1,x2,y2}` consistently. `pick` shape = `{sensor, count, bestZone, polygons}` consistently. `bestZone` shape = `{distance_mm, sigma_mm}` consistently.

## Open Risk Carried From Spec

If `drawHeatmap` uses a non-trivial color formula (per-pixel interpolation, custom thresholds), Task 5 Step 1 must mirror it exactly. Visual match is the acceptance criterion; if ramps drift, the side panel will report a distance that LOOKS inconsistent with the heatmap color.
