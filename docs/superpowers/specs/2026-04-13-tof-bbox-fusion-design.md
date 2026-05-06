# ToF + BBox Fusion in Detect Modal — Design Spec

**Date:** 2026-04-13
**Scope:** `main/dashboard.html` only — no firmware/protobuf changes.
**Status:** Approved verbally; proceeding to implementation plan.

## Goal

Unite the three signals already flowing into the dashboard — camera frame, detection bboxes, ToF zone distances — into a single coherent view inside the existing detect-result modal. After clicking Detect and seeing the bbox overlay, the user should also see:

1. A heatmap of ToF distance readings inside the bbox region (only the zones that fall within each detection, not the full ToF frame).
2. A side panel listing each detection with class label, score, and a single distance estimate based on the ToF zones inside that bbox.

## Non-Goals

- No firmware change. Detection cache + WS protocol stay as committed in `ebdcd93`.
- No new ToF data acquisition. We use the latest ToF snapshot already received over WS at click-time.
- No new aggregation modes — exactly one (best-sigma single zone, matching the dashboard's "Best sigma" label which maps to the `closest` blendMode).

## Final Modal Layout

```
+---------------------------------------------+ +-------------------+
|                                             | | DETECTIONS        |
|   snapshot canvas (800×640, rotated 180°)   | |                   |
|   ├─ camera frame (existing)                | | • cat 0.62        |
|   ├─ ToF heatmap fill, alpha≈0.45,          | |   1.42 m  best-σ  |
|   │   clipped per-bbox via ctx.clip()       | |   12 zones · A    |
|   └─ bbox outline + class label (existing)  | |                   |
|                                             | | • cat 0.27        |
|                                             | |   — no ToF cov.   |
+---------------------------------------------+ +-------------------+
  caption: frame ≈ click-time (~500ms drift)
  legend:  ▮▮▮▮▮▮▮▮  0m – 4m
```

Modal grows wider to host the side panel (~180px). Canvas keeps its current `max-width: 86vw`; sidepanel is `flex: 0 0 180px`.

## Design Decisions

| # | Question | Decision |
|---|---|---|
| 1 | Distance aggregation | **Best-sigma**: single zone with lowest `sigma` among overlapping valid zones. Reported with 2-decimal meters. |
| 2 | ToF visual treatment inside bbox | **Heatmap fill** per-zone polygon, semi-transparent (`alpha≈0.45`), clipped to bbox path. Same color ramp as dashboard's existing `heatmap` mode. |
| 3 | Side panel layout | **Right of canvas**, 180px wide, vertical stack of detection rows. |
| 4 | ToF freshness | **Snapshot the latest WS-received `tof_a` and `tof_b` at click time** (parallels the `/snapshot` JPEG fetch). |
| 5 | Multi-bbox case | **One row per bbox** in the side panel, sorted by score desc. |
| 6 | Bbox outside ToF FOV | Side panel row reads `— no ToF coverage` in dim text; no heatmap drawn for that bbox. |
| 7 | ToF A vs B | **Auto-pick by overlap count** per bbox; if tied, merge both before picking the best-sigma zone. |
| 8 | Projection params | **Reuse current dashboard state**: `AZ_OFFSET`, `EL_OFFSET`, `CX`, `CY`, `GRID_SCALE`, `FX`, flip toggles, drag offset (snapshot at click time so post-click slider drags don't desync). |

## State Capture (at Detect click)

In the click handler, before sending `DetectCommand`, capture into module-level vars:

| Var | Source |
|---|---|
| `_snapTofA` | deep-clone `lastTofA` (latest received over WS) |
| `_snapTofB` | deep-clone `lastTofB` |
| `_snapProj` | object containing `{AZ_OFFSET, EL_OFFSET, CX, CY, GRID_SCALE, FX, flipHA, flipVA, transA, flipHB, flipVB, transB, dragOffX, dragOffY}` |

These join the existing `/snapshot` JPEG fetch so all three signals (frame, ToF, projection) are coherent at click-time.

## Pipeline (when detection arrives via cache delivery)

For each `Detection {x1,y1,x2,y2}`:

1. **Project all 64 zones** of `_snapTofA` and `_snapTofB` into snapshot pixel space using existing `projectZone` math seeded with `_snapProj`. Each zone yields a 4-corner polygon in `[0..800]×[0..640]` coords.
2. **Filter to overlapping zones**: zone is included if its centroid lies inside the bbox AND `target_status ∈ {5,9}` AND `distance > 0`. Keep `{distance_mm, sigma_mm, polygon}` for each kept zone.
3. **Auto-pick sensor** (decision #7): use whichever sensor (A or B) yielded more valid overlapping zones; ties merge both pools.
4. **Aggregate distance** (decision #1): `bestZone = min by sigma`. Reported distance = `bestZone.distance_mm / 1000`.
5. **Render heatmap** (decision #2): `ctx.save(); ctx.beginPath(); rectClip(bbox); ctx.clip();` then for each overlapping zone, fill its polygon with the heatmap color for `bestZone.distance_mm` (use existing `distanceToColor` style ramp), `globalAlpha = 0.45`. `ctx.restore()` between bboxes.
6. **Side panel row** (decision #5/6): emit one row per bbox. If `count > 0`: `class · score · distance · "N zones · sensor"`. Else: `class · score · "— no ToF coverage"`.

## Files Touched

- `main/dashboard.html` only:
  - **CSS:** `#detect-panel` → `flex-direction: row` for canvas+sidepanel; new `#detect-sidepanel` styles; legend gradient.
  - **HTML:** add `<div id="detect-sidepanel">` + `<div id="detect-legend">` next to the existing `#detect-canvas`.
  - **State vars:** `_snapTofA`, `_snapTofB`, `_snapProj` at module scope.
  - **Functions:**
    - `captureTofSnapshot()` — copies latest ToF + projection params at click time.
    - `projectZonesIntoBbox(tofGrid, bbox, proj, sensorKey)` → `{count, bestZone, polygons}`.
    - `pickSensorForBbox(bbox)` — runs the above for A and B, returns winner per decision #7.
    - `renderToFHeatmap(ctx, polygons, color)` — per-bbox clipped fill.
    - `renderSidePanel(detections)` — populates `#detect-sidepanel`.
  - **Modify:** `snapCamToDetectCanvas` → also call `captureTofSnapshot`. `renderDetections` → call `pickSensorForBbox`, `renderToFHeatmap`, `renderSidePanel`.

## Edge Cases & Acceptance

| Case | Behavior |
|---|---|
| Detect fires before any ToF data ever received | `_snapTofA/B` are null → all bboxes show `no ToF coverage`. |
| ToF received but `valid: false` | Treated as no coverage. |
| 0 detections | Modal opens as today (no bboxes, no heatmap). Side panel shows `— no detections`. |
| Detection bbox fully outside FOV (e.g. far edge of image) | `count=0` → `no ToF coverage`. |
| Multiple bboxes, one with coverage and one without | Mixed: heatmap for the covered one, dim row for the other. |
| User drags AZ/EL slider after click | Heatmap stays aligned (we snapshotted projection params). |

## Open Risk

The `projectZone` math currently lives inline in `renderOverlay`. We'll need to refactor it into a pure function `projectZone(row, col, params) → polygon` so both renderOverlay and the new modal pipeline use it. Verify no behavior change in the live overlay after refactor.

## Out of Scope (later)

- Per-zone distance label inside the heatmap (consider after seeing the heatmap).
- Sensor blending strategies beyond best-sigma.
- Fixing the broken `snap-btn` / `snap-overlay-btn` (separate task — they need `/snapshot` fetch like the detect modal).
