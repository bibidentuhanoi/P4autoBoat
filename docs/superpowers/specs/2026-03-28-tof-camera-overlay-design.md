# ToF–Camera Overlay Design

**Date**: 2026-03-28
**Status**: Approved

## Summary

Add a semi-transparent ToF depth overlay on top of the MJPEG camera stream in the dashboard (`dashboard.html`). Each VL53L5CX 8×8 zone is projected onto camera pixels using distance-dependent parallax correction from the known physical mount geometry. Pure client-side JS — zero firmware changes.

## Hardware Geometry

```
              BOW (forward / +Z)
                 ^
                 |
     hull-L    CAM (0,0,0)    hull-R
                 |
          7.27mm aft (-Z)
                 |
    ToF-A -------+------- ToF-B
    (port)              (starboard)
   ←24.60mm→       ←24.60mm→
    ↙ 5° out          5° out ↘

    Both RX apertures face inboard (toward camera center).
    TX apertures face outboard (toward hulls).
```

### Fixed Parameters

| Parameter | Value |
|---|---|
| Camera resolution | 800 × 640 px |
| Camera HFoV | 72° (standard OV5647 lens) |
| Camera VFoV | ~60° (from aspect ratio) |
| Focal length (px) | `400 / tan(36°) ≈ 550.6` |
| Principal point | `(400, 320)` — image center |
| ToF-A offset | `(-24.6, 0, -7.27)` mm relative to camera |
| ToF-B offset | `(+24.6, 0, -7.27)` mm relative to camera |
| ToF-A yaw | -5° (port outward) |
| ToF-B yaw | +5° (starboard outward) |
| ToF FoV | 45° × 45° square |
| Zone angular size | 5.625° (45° / 8), linearly spaced |
| ToF range | 0–4000 mm |
| Data rate | 5 Hz (WebSocket protobuf snapshots) |

## Projection Math

### Coordinate System

- Camera at origin `(0, 0, 0)`, looking along `+Z` (forward/bow)
- `+X` = starboard (right), `-X` = port (left)
- `+Y` = down, `-Y` = up (standard image convention)

### Zone Ray Computation

Each ToF zone `(row, col)` maps to an angular direction in the sensor's local frame. The VL53L5CX uses an f-theta lens, so zones are linearly spaced in angle:

```
az = (col - 3.5) × 5.625°    // horizontal angle from sensor boresight
el = (row - 3.5) × 5.625°    // vertical angle (positive = down)
```

The ray direction in sensor-local frame:
```
dx = tan(az)
dy = tan(el)
dz = 1.0
normalize to unit vector: (dx, dy, dz) / ||(dx, dy, dz)||
```

For a zone reading distance `d` mm, the 3D point in sensor-local frame:
```
P_sensor = d × (dx, dy, dz)_normalized
```

### Sensor-to-Camera Transform

Rotate by sensor yaw, then translate by physical offset:

**ToF-A** (`yaw_a = -5°`, offset `(-24.6, 0, -7.27)` mm):
```
X_cam = cos(yaw_a) × X_sensor - sin(yaw_a) × Z_sensor + (-24.6)
Y_cam = Y_sensor
Z_cam = sin(yaw_a) × X_sensor + cos(yaw_a) × Z_sensor + (-7.27)
```

**ToF-B** (`yaw_b = +5°`, offset `(+24.6, 0, -7.27)` mm):
```
X_cam = cos(yaw_b) × X_sensor - sin(yaw_b) × Z_sensor + (+24.6)
Y_cam = Y_sensor
Z_cam = sin(yaw_b) × X_sensor + cos(yaw_b) × Z_sensor + (-7.27)
```

### Pinhole Projection

Project 3D camera-frame point to pixel coordinates:
```
f = 550.6    // focal length in pixels
cx = 400     // principal point x
cy = 320     // principal point y

px = f × (X_cam / Z_cam) + cx
py = f × (Y_cam / Z_cam) + cy
```

### Per-Zone Rendering

For each zone, project all 4 angular corners (not just center) to get the bounding quadrilateral on the camera image. Each corner is computed at the zone's measured distance.

## Zone Ordering Calibration

The mapping of array index to physical zone position depends on sensor orientation and possible ULD API transposition. This will be determined empirically:

1. Place an object at a known position (e.g., directly ahead, slightly port)
2. Observe which zone indices light up in the raw 8×8 grid
3. Set flip flags: `FLIP_H_A`, `FLIP_V_A`, `FLIP_H_B`, `FLIP_V_B` (boolean JS constants)

Both sensors are mirrored (A is flipped relative to B), so each needs independent flip flags.

## Rendering

### Visual Design

- **Heatmap**: blue (4000 mm, far) → red (0 mm, close). Reuses existing `distToColor()` function.
- **Opacity**: 0.35 alpha — camera image visible beneath.
- **Overlap zone**: where ToF-A and ToF-B zones overlap (~35° center), both rectangles draw independently. No blending or fusion. Overlap appears slightly more opaque.
- **No labels or numbers** on the overlay. The existing separate 8×8 heatmap canvases remain for detailed reading.

### Edge Cases

| Condition | Behavior |
|---|---|
| Zone reads 0 mm (no target) | Don't draw — leave transparent |
| Zone reads > 4000 mm | Clamp color to 4000 mm, project at 4000 mm |
| Zone projects outside camera frame | Clip to canvas bounds |
| No ToF data yet | Overlay stays empty/clear |
| Camera not streaming | Overlay still renders on black background |

## Dashboard Integration

### DOM Changes

```html
<!-- Wrap existing cam-img in a relative container -->
<div id="cam-container" style="position:relative; width:100%; flex:1; min-height:0;">
  <img id="cam-img" src="" alt="No stream" style="width:100%; height:100%; object-fit:contain;" />
  <canvas id="tof-overlay" style="position:absolute; top:0; left:0; width:100%; height:100%; pointer-events:none;"></canvas>
</div>
```

### Canvas Sizing

Use `ResizeObserver` on `#cam-img` to keep `#tof-overlay` canvas dimensions synced with the displayed image size. The projection math outputs native pixel coordinates (800×640) which are then scaled to the canvas display size.

Note: `object-fit: contain` may letterbox the image. The overlay canvas must account for letterboxing offsets to align correctly.

### Toggle

A checkbox or small button in the dashboard header to show/hide the overlay. Default: visible.

### Data Flow

```
WebSocket onmessage
  → protobuf decode (existing)
  → update 8×8 heatmap canvases (existing)
  → NEW: renderOverlay(tofA, tofB)
```

No new endpoints, no new WebSocket messages, no firmware changes.

### JS Structure

All new code added to `dashboard.html` `<script>` block:

1. **Geometry constants** — offsets, yaw angles, FoV, focal length, flip flags
2. **`projectZone(sensor, row, col, distance)`** — returns 4 pixel coordinates (corners)
3. **`renderOverlay(tofA, tofB)`** — clears canvas, iterates all zones, draws filled quadrilaterals
4. **`ResizeObserver` handler** — updates canvas size and letterbox offset
5. **Toggle state** — boolean, wired to checkbox

### Performance

- 128 zones × 4 corners = 512 pinhole projections per frame at 5 Hz
- Pure arithmetic + canvas `fillRect` — trivially fast in JS
- No DOM manipulation, no compositing tricks

## What This Design Does NOT Include

- Zone ordering auto-detection (empirical calibration only)
- Lens distortion correction (negligible for standard OV5647 lens at this precision)
- Fusion or averaging in the overlap zone
- Any firmware changes
- SD card recording of overlay
- YOLO or object detection integration

## References

- ST VL53L5CX datasheet — 45° × 45° FoV, 8×8 zone grid, f-theta lens
- ST AN5896 — camera-to-ToF registration (homography approach with depth-dependent parallax)
- VL53L5CX ULD API — zone data ordering may be transposed vs datasheet
