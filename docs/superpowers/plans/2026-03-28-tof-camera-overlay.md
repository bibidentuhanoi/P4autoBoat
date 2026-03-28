# ToF–Camera Overlay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a distance-dependent ToF depth overlay on the MJPEG camera stream in the dashboard.

**Architecture:** Pure client-side JS in `dashboard.html`. A `<canvas>` is positioned over the camera `<img>`. Each WebSocket snapshot triggers projection of 128 ToF zones (2 sensors × 64 zones) onto camera pixel coordinates using pinhole model + physical mount offsets. No firmware changes.

**Tech Stack:** Vanilla JS, Canvas 2D API, existing protobuf WebSocket

**Spec:** `docs/superpowers/specs/2026-03-28-tof-camera-overlay-design.md`

---

## File Map

All changes are in a single file:

- **Modify:** `dashboard.html`
  - DOM: wrap `#cam-img` in container, add overlay `<canvas>`, add toggle checkbox
  - CSS: overlay positioning, toggle styling
  - JS: geometry constants, projection math, overlay renderer, ResizeObserver, toggle wiring

---

### Task 1: DOM — Wrap Camera Image and Add Overlay Canvas

**Files:**
- Modify: `dashboard.html:73-82` (CSS) and `dashboard.html:136-140` (HTML)

- [ ] **Step 1: Add CSS for the overlay container and canvas**

In the `<style>` block, after the `#cam-img` rules (line 82), add:

```css
  /* ToF overlay */
  #cam-container {
    position: relative;
    width: 100%;
    flex: 1;
    min-height: 0;
  }
  #cam-container #cam-img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    border-radius: 4px;
    background: #000;
  }
  #tof-overlay {
    position: absolute;
    top: 0; left: 0;
    width: 100%; height: 100%;
    pointer-events: none;
  }
```

- [ ] **Step 2: Remove the standalone `#cam-img` CSS rule**

Remove these lines (75-82) — keep line 74 (`#camera-card` grid rule) intact:

```css
  #cam-img {
    width: 100%;
    flex: 1;
    object-fit: contain;
    border-radius: 4px;
    background: #000;
    min-height: 0;
  }
```

- [ ] **Step 3: Wrap `#cam-img` in container and add overlay canvas**

Replace the camera card body (lines 137-140):

```html
  <!-- Camera -->
  <div class="card" id="camera-card">
    <div class="card-title">Camera — MJPEG Stream</div>
    <img id="cam-img" src="" alt="No stream" />
  </div>
```

With:

```html
  <!-- Camera -->
  <div class="card" id="camera-card">
    <div class="card-title">Camera — MJPEG Stream</div>
    <div id="cam-container">
      <img id="cam-img" src="" alt="No stream" />
      <canvas id="tof-overlay"></canvas>
    </div>
  </div>
```

- [ ] **Step 4: Add toggle checkbox in header**

After the `<span id="msg-rate" ...></span>` line (line 131), add:

```html
  <label id="overlay-toggle" style="color:var(--dim); font-size:11px; cursor:pointer; user-select:none;">
    <input type="checkbox" id="overlay-cb" checked style="margin-right:4px;" />ToF overlay
  </label>
```

- [ ] **Step 5: Verify by opening dashboard in browser**

Open `dashboard.html` in a browser. The camera card should still render. The overlay canvas should be invisible (empty). The "ToF overlay" checkbox should appear in the header.

- [ ] **Step 6: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): add ToF overlay canvas and toggle checkbox"
```

---

### Task 2: Overlay Color Function (4000mm Scale)

**Files:**
- Modify: `dashboard.html` (JS `<script>` block)

- [ ] **Step 1: Add overlay color function**

After the existing `distToColor` function (after line 378), add:

```javascript
// ── ToF overlay color (4000mm range) ────────────────────────
const OVERLAY_MAX_MM = 4000;

function overlayColor(mm) {
  if (!mm || mm <= 0) return null;  // no target — skip zone
  const t = Math.min(mm, OVERLAY_MAX_MM) / OVERLAY_MAX_MM;
  const stops = [
    [255, 0,   0  ],   // close = red
    [255, 180, 0  ],   // orange
    [0,   255, 80 ],   // green
    [0,   200, 255],   // cyan
    [0,   60,  200],   // far = blue
  ];
  const scaled = t * (stops.length - 1);
  const i = Math.floor(scaled);
  const f = scaled - i;
  const a = stops[Math.min(i, stops.length - 1)];
  const b = stops[Math.min(i + 1, stops.length - 1)];
  return [
    Math.round(a[0] + (b[0] - a[0]) * f),
    Math.round(a[1] + (b[1] - a[1]) * f),
    Math.round(a[2] + (b[2] - a[2]) * f),
  ];
}
```

- [ ] **Step 2: Quick console test**

In browser dev console, run:
```javascript
overlayColor(0)      // → null
overlayColor(200)    // → reddish [255, ~18, 0]
overlayColor(2000)   // → greenish
overlayColor(4000)   // → blue [0, 60, 200]
```

- [ ] **Step 3: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): add overlayColor() with 4000mm range"
```

---

### Task 3: Geometry Constants and Projection Math

**Files:**
- Modify: `dashboard.html` (JS `<script>` block)

- [ ] **Step 1: Add geometry constants**

After the `overlayColor` function, add:

```javascript
// ── ToF overlay — geometry & projection ─────────────────────
const TOF_OVERLAY = {
  // Camera intrinsics (800x640 @ 72° HFoV)
  camW: 800,
  camH: 640,
  focalPx: 400 / Math.tan(36 * Math.PI / 180),  // ≈ 550.6
  cx: 400,
  cy: 320,

  // VL53L5CX zone grid
  zones: 8,
  zoneDeg: 5.625,  // 45° / 8, linearly spaced (f-theta)

  // Zone ordering flip flags (calibrate empirically)
  flipHA: false,  // flip horizontal for sensor A
  flipVA: false,  // flip vertical for sensor A
  flipHB: false,  // flip horizontal for sensor B
  flipVB: false,  // flip vertical for sensor B

  // Sensor A (port) — offset in mm, yaw in radians
  sensorA: {
    tx: -24.6,  ty: 0,  tz: -7.27,
    yaw: -5 * Math.PI / 180,
  },
  // Sensor B (starboard)
  sensorB: {
    tx: 24.6,  ty: 0,  tz: -7.27,
    yaw: 5 * Math.PI / 180,
  },

  alpha: 0.35,  // overlay opacity
};
```

- [ ] **Step 2: Add zone corner projection function**

```javascript
/**
 * Project a single 3D point from sensor frame to camera pixel.
 * @param {object} sensor - TOF_OVERLAY.sensorA or .sensorB
 * @param {number} az - horizontal angle in radians (sensor-local)
 * @param {number} el - vertical angle in radians (sensor-local, positive=down)
 * @param {number} d  - distance in mm
 * @returns {[number, number]} pixel [px, py] in native 800x640 coords, or null if behind camera
 */
function tofProject(sensor, az, el, d) {
  // Ray direction in sensor-local frame
  const dx = Math.tan(az);
  const dy = Math.tan(el);
  const dz = 1.0;
  const len = Math.sqrt(dx * dx + dy * dy + dz * dz);

  // 3D point in sensor frame
  const sx = d * dx / len;
  const sy = d * dy / len;
  const sz = d * dz / len;

  // Rotate by yaw + translate to camera frame
  const cosY = Math.cos(sensor.yaw);
  const sinY = Math.sin(sensor.yaw);
  const cx = cosY * sx - sinY * sz + sensor.tx;
  const cy = sy + sensor.ty;
  const cz = sinY * sx + cosY * sz + sensor.tz;

  // Behind camera — don't render
  if (cz <= 0) return null;

  // Pinhole projection
  const f = TOF_OVERLAY.focalPx;
  const px = f * (cx / cz) + TOF_OVERLAY.cx;
  const py = f * (cy / cz) + TOF_OVERLAY.cy;
  return [px, py];
}
```

- [ ] **Step 3: Add function to get the 4 corners of a zone**

```javascript
/**
 * Get the 4 projected pixel corners for a ToF zone.
 * @param {object} sensor - TOF_OVERLAY.sensorA or .sensorB
 * @param {number} row - zone row 0-7
 * @param {number} col - zone col 0-7
 * @param {number} d - distance reading in mm
 * @param {boolean} flipH - flip horizontal
 * @param {boolean} flipV - flip vertical
 * @returns {Array|null} [[x0,y0],[x1,y1],[x2,y2],[x3,y3]] or null
 */
function projectZoneCorners(sensor, row, col, d, flipH, flipV) {
  if (!d || d <= 0) return null;
  d = Math.min(d, OVERLAY_MAX_MM);  // clamp projection distance per spec

  const zd = TOF_OVERLAY.zoneDeg * Math.PI / 180;
  const c = flipH ? (7 - col) : col;
  const r = flipV ? (7 - row) : row;

  // Angular bounds of this zone
  const azMin = (c - 4) * zd;
  const azMax = (c - 3) * zd;  // (c+1 - 4) * zd
  const elMin = (r - 4) * zd;
  const elMax = (r - 3) * zd;

  // Project all 4 corners at the zone's measured distance
  const tl = tofProject(sensor, azMin, elMin, d);
  const tr = tofProject(sensor, azMax, elMin, d);
  const br = tofProject(sensor, azMax, elMax, d);
  const bl = tofProject(sensor, azMin, elMax, d);

  if (!tl || !tr || !br || !bl) return null;
  return [tl, tr, br, bl];
}
```

- [ ] **Step 4: Console verification of projection**

In browser dev console:
```javascript
// Sensor A, center zone (3,3), 1000mm distance — should project near image center-left
projectZoneCorners(TOF_OVERLAY.sensorA, 3, 3, 1000, false, false);
// Should return 4 corner pixel coords roughly around (350-380, 300-340)

// Sensor B, center zone (3,4), 1000mm — should project near image center-right
projectZoneCorners(TOF_OVERLAY.sensorB, 3, 4, 1000, false, false);
// Should return 4 corner pixel coords roughly around (420-450, 300-340)
```

- [ ] **Step 5: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): add ToF projection math with geometry constants"
```

---

### Task 4: Canvas Sizing with Letterbox Handling

**Files:**
- Modify: `dashboard.html` (JS `<script>` block)

- [ ] **Step 1: Add ResizeObserver and letterbox calculation**

After the projection functions, add:

```javascript
// ── ToF overlay — canvas sizing ─────────────────────────────
let overlayCtx = null;
let overlayScale = 1;
let overlayOffsetX = 0;
let overlayOffsetY = 0;

function updateOverlaySize() {
  const canvas = $('tof-overlay');
  const img = $('cam-img');
  const container = $('cam-container');
  const cw = container.clientWidth;
  const ch = container.clientHeight;

  canvas.width = cw;
  canvas.height = ch;

  // Compute letterbox offsets for object-fit: contain
  const imgAspect = TOF_OVERLAY.camW / TOF_OVERLAY.camH;  // 800/640 = 1.25
  const containerAspect = cw / ch;

  let drawW, drawH;
  if (containerAspect > imgAspect) {
    // Pillarboxed (bars on sides)
    drawH = ch;
    drawW = ch * imgAspect;
  } else {
    // Letterboxed (bars on top/bottom)
    drawW = cw;
    drawH = cw / imgAspect;
  }

  overlayScale = drawW / TOF_OVERLAY.camW;
  overlayOffsetX = (cw - drawW) / 2;
  overlayOffsetY = (ch - drawH) / 2;

  overlayCtx = canvas.getContext('2d');
}

// Observe container size changes
const resizeObs = new ResizeObserver(() => updateOverlaySize());
resizeObs.observe($('cam-container'));
updateOverlaySize();
```

- [ ] **Step 2: Verify sizing**

Resize the browser window. Check in dev console:
```javascript
overlayScale      // should be a positive number
overlayOffsetX    // should be >= 0
overlayOffsetY    // should be >= 0
$('tof-overlay').width   // should match container width
$('tof-overlay').height  // should match container height
```

- [ ] **Step 3: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): add ResizeObserver with letterbox offset calc"
```

---

### Task 5: Overlay Renderer

**Files:**
- Modify: `dashboard.html` (JS `<script>` block)

- [ ] **Step 1: Add the renderOverlay function**

After the canvas sizing code, add:

```javascript
// ── ToF overlay — renderer ──────────────────────────────────
let overlayEnabled = true;

function renderOverlay(tofA, tofB) {
  if (!overlayCtx) return;
  const canvas = $('tof-overlay');
  overlayCtx.clearRect(0, 0, canvas.width, canvas.height);

  if (!overlayEnabled) return;

  overlayCtx.globalAlpha = TOF_OVERLAY.alpha;

  function drawSensor(tofGrid, sensor, flipH, flipV) {
    if (!tofGrid || !tofGrid.valid || !tofGrid.distances || tofGrid.distances.length < 64) return;

    for (let row = 0; row < 8; row++) {
      for (let col = 0; col < 8; col++) {
        const mm = tofGrid.distances[row * 8 + col];
        const color = overlayColor(mm);
        if (!color) continue;  // no target

        const corners = projectZoneCorners(sensor, row, col, mm, flipH, flipV);
        if (!corners) continue;

        // Transform native px → display px (with letterbox offset)
        overlayCtx.fillStyle = `rgb(${color[0]},${color[1]},${color[2]})`;
        overlayCtx.beginPath();
        overlayCtx.moveTo(
          corners[0][0] * overlayScale + overlayOffsetX,
          corners[0][1] * overlayScale + overlayOffsetY
        );
        for (let i = 1; i < 4; i++) {
          overlayCtx.lineTo(
            corners[i][0] * overlayScale + overlayOffsetX,
            corners[i][1] * overlayScale + overlayOffsetY
          );
        }
        overlayCtx.closePath();
        overlayCtx.fill();
      }
    }
  }

  drawSensor(tofA, TOF_OVERLAY.sensorA, TOF_OVERLAY.flipHA, TOF_OVERLAY.flipVA);
  drawSensor(tofB, TOF_OVERLAY.sensorB, TOF_OVERLAY.flipHB, TOF_OVERLAY.flipVB);

  overlayCtx.globalAlpha = 1.0;
}
```

- [ ] **Step 2: Wire toggle checkbox**

After `renderOverlay`, add:

```javascript
$('overlay-cb').addEventListener('change', (e) => {
  overlayEnabled = e.target.checked;
  if (!overlayEnabled && overlayCtx) {
    overlayCtx.clearRect(0, 0, $('tof-overlay').width, $('tof-overlay').height);
  }
});
```

- [ ] **Step 3: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): add renderOverlay() and toggle wiring"
```

---

### Task 6: Hook into WebSocket Data Flow

**Files:**
- Modify: `dashboard.html:268-269` (inside `ws.onmessage`)

- [ ] **Step 1: Add renderOverlay call to WebSocket handler**

In the `ws.onmessage` handler, after the existing ToF heatmap draw calls (lines 268-269):

```javascript
        if (s.tofA) drawTof('tof-a', s.tofA);
        if (s.tofB) drawTof('tof-b', s.tofB);
```

Add immediately after:

```javascript
        renderOverlay(s.tofA, s.tofB);
```

- [ ] **Step 2: Verify end-to-end**

Open dashboard connected to the ESP32. With ToF sensors active:
- Colored semi-transparent rectangles should appear over the camera image
- Rectangles should update at ~5Hz with each WebSocket snapshot
- Unchecking the "ToF overlay" checkbox should hide the overlay
- Checking it again should restore it on the next snapshot

- [ ] **Step 3: Commit**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): wire renderOverlay into WebSocket onmessage"
```

---

### Task 7: Empirical Zone Ordering Calibration

This task is done with the physical hardware. No code to write upfront — just the procedure.

- [ ] **Step 1: Place a known object**

Put an object (hand, box) directly in front of the camera, slightly to the **port** (left) side, at ~500mm range.

- [ ] **Step 2: Observe which ToF-A zones light up**

Look at the overlay on the camera stream. The red/orange zones should appear where the object is. If they appear mirrored or rotated:
- Object is port but overlay shows starboard → set `TOF_OVERLAY.flipHA = true`
- Object is high but overlay shows low → set `TOF_OVERLAY.flipVA = true`

- [ ] **Step 3: Repeat for ToF-B**

Move the object to the **starboard** (right) side. Adjust `flipHB` and `flipVB` the same way.

- [ ] **Step 4: Commit the calibrated flip flags**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): calibrate ToF zone ordering flip flags"
```

---

### Task 8: Final Cleanup and Verification

- [ ] **Step 1: Test all edge cases**

- Disconnect WebSocket → overlay should stay empty
- No ToF data (sensors off) → overlay empty, camera still streams
- Object at close range (~200mm) → zones should visibly shift outward from parallax
- Object at far range (~2000mm) → zones should cluster near center
- Toggle checkbox on/off → overlay appears/disappears
- Resize browser window → overlay stays aligned with camera image

- [ ] **Step 2: Final commit if any tweaks needed**

```bash
git add -f dashboard.html
git commit -m "feat(dashboard): ToF camera overlay — final tweaks"
```
