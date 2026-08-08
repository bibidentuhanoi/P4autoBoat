# Dashboard Camera-ToF Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add a temporary browser-only workflow that calibrates upright camera intrinsics, solves each ToF sensor's physical placement, and previews a sparse camera-aligned ToF heatmap.

**Architecture:** The only production change is main/dashboard.html because it is embedded directly by main/CMakeLists.txt. A marked DOM-free CalibrationCore block in the dashboard owns statistics, geometry, mapping, fitting, and projection; Node tests extract and execute that exact shipped code. The dashboard UI owns the rail swap, OpenCV ChArUco handling, in-memory session, snapshot capture, and overlay.

**Tech Stack:** Embedded HTML/CSS/vanilla JavaScript, on-demand OpenCV.js, existing protobufjs/WebSocket telemetry and snapshot endpoint, Node 24 built-in node:test, Playwright 1.62.

## Global Constraints

- Modify production behavior only in main/dashboard.html. Do not edit firmware, protobuf, CMake, endpoints, ESP-NOW, camera, ToF, ML, NVS, or storage code.
- Keep every existing WebSocket/API call, motor/winch/steer behavior, telemetry, ToF card, settings, detection modal, MJPEG stream, and snapshot route intact.
- Use existing human-upright 800 x 640 browser coordinates. The current CSS image rotation and one 180-degree snapshot-canvas rotation are the only rotations.
- Keep calibration session-only. Do not use NVS, localStorage, sessionStorage, IndexedDB, cookies, export files, or firmware parameters.
- The current Operations content must hide but remain alive when Calibration is open. Calibration must not send motor, winch, steer, raw-steer, arm, or servo-power commands.
- Wall target: 7 x 5 squares, 35 mm square, 26 mm marker, DICT_4X4 family, 280 x 195 mm PDF. Foreground target: 5 x 3, 35 mm, 26 mm, DICT_4X4 family, start ID 20, 200 x 120 mm PDF.
- Require 10 useful camera captures and 10 accepted wall poses. Solve A and B independently using eight fit poses and two held-out validation poses.
- The compass and IMU only warn about motion. ChArUco supplies wall pose, actual yaw, and distance.
- Status 5 has full fitting weight, status 9 half weight, all other statuses are excluded. A capture needs 32 stable zones per sensor.
- Render sparse measured ToF cells only. Never claim to produce a dense depth map.

---

## File Structure

| File | Responsibility |
| --- | --- |
| main/dashboard.html | Production CalibrationCore, right-rail swap, OpenCV flow, session state, sampling, solver orchestration, preview, and validation. |
| tests/extract-dashboard-calibration-core.mjs | Extracts marked CalibrationCore source from dashboard.html and evaluates it inside Node VM. |
| tests/fixtures/dashboard-calibration-fixtures.mjs | Deterministic walls, known transforms, valid grids, and rejected-zone inputs. |
| tests/dashboard-calibration-core.test.mjs | Node tests for mapping, reduction, fit, trust verdict, foreground metric, and static dashboard contract. |
| tests/dashboard-calibration-ui.spec.mjs | Playwright browser fixture with mocked WebSocket and snapshot response. |
| playwright.config.mjs | Test-only Chromium project and static-server lifecycle for the browser smoke test. |

Do not create external production JavaScript because the present firmware embeds dashboard.html only. Put the pure CalibrationCore just after the current projection helpers and before UI/event code. Mark it literally with CALIBRATION_CORE_BEGIN and CALIBRATION_CORE_END comments.

## Shared Interfaces

~~~js
const CalibrationCore = {
  median(values),
  medianAbsoluteDeviation(values, center),
  enumerateZoneMappings(),
  mapZone(row, col, mapping),
  makeZoneRay(row, col, mapping, horizontalFovDeg = 45, verticalFovDeg = 45),
  reduceStableGrid(samples),
  wallPlaneFromPose(rvec, tvec, cv),
  transformPoint(point, transform),
  pointPlaneResidual(point, plane),
  solveSensorTransform({ captures, sensorKey, initialTransform }),
  evaluateSensorSolution({ solution, captures }),
  cameraQuality({ sampleCount, rmsPx }),
  projectZoneToImage({ row, col, distanceMm, sensorTransform, intrinsics, mapping }),
  foregroundOverlapScore({ zonePolygons, foregroundPolygon }),
};

const SensorTransform = {
  translationMm: [Number, Number, Number],
  rotationDeg: { yaw: Number, pitch: Number, roll: Number },
  rangeBiasMm: Number,
  mapping: String,
};
~~~

A WallCapture has id, snapshotTimestampMs, plane with normal and d, cameraPose with rvec and tvec, and independently reduced tofA/tofB zone collections. All lengths are millimetres, UI rotations are degrees, and display pixels are upright 800 x 640.

### Task 1: Add Extractable Calibration Core and Test Harness

**Files:**
- Create: tests/extract-dashboard-calibration-core.mjs
- Create: tests/fixtures/dashboard-calibration-fixtures.mjs
- Create: tests/dashboard-calibration-core.test.mjs
- Modify: main/dashboard.html after captureProjectionParams and before UI event code.

**Interfaces:**
- Consumes: dashboard source only. It has no DOM, Canvas, WebSocket, OpenCV, or firmware dependency.
- Produces: global CalibrationCore, core markers, fixture data.

- [ ] **Step 1: Write failing extractor, mapping, and stable-grid tests**

Create the strict extractor:

~~~js
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';

export function loadCalibrationCore() {
  const html = fs.readFileSync(path.resolve('main/dashboard.html'), 'utf8');
  const begin = '// CALIBRATION_CORE_BEGIN';
  const end = '// CALIBRATION_CORE_END';
  const start = html.indexOf(begin);
  const finish = html.indexOf(end);
  if (start < 0 || finish < 0 || finish <= start ||
      html.indexOf(begin, start + begin.length) !== -1 ||
      html.indexOf(end, finish + end.length) !== -1) {
    throw new Error('dashboard calibration core markers are invalid');
  }
  const context = vm.createContext({ console, Math, Number, Array, Object, Map, Set });
  const source = html.slice(start + begin.length, finish);
  vm.runInContext(source + '; globalThis.__core = CalibrationCore;', context);
  return context.__core;
}
~~~

Fixture mapping names:

~~~js
export const EXPECTED_MAPPINGS = [
  'identity', 'flipH', 'flipV', 'rotate180',
  'transpose', 'transposeFlipH', 'transposeFlipV', 'transposeRotate180',
];
~~~

First tests:

~~~js
test('core exports eight unique 8x8 zone mappings', () => {
  const core = loadCalibrationCore();
  const mappings = core.enumerateZoneMappings();
  assert.deepEqual(mappings.map(m => m.name), EXPECTED_MAPPINGS);
  for (const mapping of mappings) {
    const mapped = new Set();
    for (let row = 0; row < 8; row++) for (let col = 0; col < 8; col++) {
      mapped.add(core.mapZone(row, col, mapping).join(','));
    }
    assert.equal(mapped.size, 64, mapping.name);
  }
});

test('stable-grid reduction weights statuses and rejects unstable values', () => {
  const core = loadCalibrationCore();
  const samples = Array.from({ length: 8 }, (_, n) => ({
    timestampUs: n + 1, distances: Array(64).fill(1000),
    sigma: Array(64).fill(8), targetStatus: Array(64).fill(5),
    nbTargetDetected: Array(64).fill(1),
  }));
  samples[0].targetStatus[1] = 9;
  samples[0].distances[2] = 100;
  samples[1].distances[2] = 1900;
  samples[2].distances[2] = 100;
  const reduced = core.reduceStableGrid(samples);
  assert.equal(reduced.zones[0].distanceMm, 1000);
  assert.equal(reduced.zones[1].statusWeight, 0.5);
  assert.equal(reduced.zones[2].stable, false);
});
~~~

- [ ] **Step 2: Run test to verify failure**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: FAIL with dashboard calibration core markers are invalid.

- [ ] **Step 3: Implement pure core**

Insert exactly one IIFE with the literal markers. Define all eight physical mapping functions explicitly:

~~~js
// CALIBRATION_CORE_BEGIN
const CalibrationCore = (() => {
  const MAPPINGS = [
    ['identity', (r, c) => [r, c]],
    ['flipH', (r, c) => [r, 7 - c]],
    ['flipV', (r, c) => [7 - r, c]],
    ['rotate180', (r, c) => [7 - r, 7 - c]],
    ['transpose', (r, c) => [c, r]],
    ['transposeFlipH', (r, c) => [c, 7 - r]],
    ['transposeFlipV', (r, c) => [7 - c, r]],
    ['transposeRotate180', (r, c) => [7 - c, 7 - r]],
  ];
  const median = values => {
    const sorted = values.filter(Number.isFinite).toSorted((a, b) => a - b);
    if (!sorted.length) return NaN;
    const middle = sorted.length >> 1;
    return sorted.length & 1 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2;
  };
  const medianAbsoluteDeviation = (values, center = median(values)) =>
    median(values.map(value => Math.abs(value - center)));
  // Define every shared interface member.
  return { median, medianAbsoluteDeviation, enumerateZoneMappings, mapZone };
})();
// CALIBRATION_CORE_END
~~~

reduceStableGrid selects each sample's nearest positive target for a zone, uses status weight 1 for status 5 and 0.5 for 9, then retains it only if MAD is at most max(20 mm, twice median sigma). Return zones with row, col, distanceMm, sigmaMm, statusWeight, stable, targetCount and return stableCount/sampleCount.

- [ ] **Step 4: Run test**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS. Do not use DOM stubs; the core must remain pure.

- [ ] **Step 5: Commit**

~~~bash
git add main/dashboard.html tests/extract-dashboard-calibration-core.mjs \
  tests/fixtures/dashboard-calibration-fixtures.mjs tests/dashboard-calibration-core.test.mjs
git commit -m "feat: add dashboard calibration core"
~~~

### Task 2: Add Operations/Calibration Right-Rail State

**Files:**
- Modify: main/dashboard.html CSS grid near lines 68-100, grid markup near 360-455, final init/event code.
- Test: tests/dashboard-calibration-core.test.mjs

**Interfaces:**
- Consumes: camera card, IMU, ToF, drive, and winch cards.
- Produces: setDashboardRail(mode), resetCalibrationSession(), calibrationSession, operations-rail, calibration-rail, calibration-btn, calibration-close, calibration-stage, calibration-status, calibration-content.

- [ ] **Step 1: Write failing UI/API contract test**

~~~js
test('dashboard retains camera, telemetry, controls, and endpoints', () => {
  const html = readFileSync('main/dashboard.html', 'utf8');
  for (const token of [
    'id="cam-img"', 'id="tof-overlay"', 'id="motor-card"', 'id="winch-card"',
    'id="tof-a"', 'id="tof-b"', 'id="calibration-btn"', 'id="operations-rail"',
    'id="calibration-rail"', '/snapshot', 'startStream()', 'connectWS()',
  ]) assert.ok(html.includes(token), token);
  assert.doesNotMatch(html, /localStorage|indexedDB|NVS|sessionStorage/);
});
~~~

- [ ] **Step 2: Run test to verify failure**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: FAIL because rail IDs do not exist.

- [ ] **Step 3: Implement the complete rail swap**

Wrap current imu-card, tof-card, motor-card, and winch-card without changing their internal IDs or listeners:

~~~html
<div id="operations-rail" class="right-rail" aria-label="Operations">
  <!-- existing IMU, ToF, drive, and winch cards -->
</div>
<aside id="calibration-rail" class="right-rail" aria-label="Calibration" hidden>
  <div class="calibration-head">
    <div><span class="rail-kicker">Bench calibration</span><h2>Camera + ToF</h2></div>
    <button id="calibration-close" class="icon-button" title="Return to operations"
      aria-label="Return to operations">×</button>
  </div>
  <ol id="calibration-stage" class="calibration-steps"></ol>
  <div id="calibration-status" role="status"></div>
  <div id="calibration-content"></div>
</aside>
~~~

Both rails occupy the old right-column cell. Preserve camera-card grid-column 1 / 3 and grid-row 1 / 5. At mobile widths stack the visible rail after camera. Add Calibration to the current camera-title button group.

~~~js
const calibrationSession = {
  mode: 'operations', stage: 'camera', camera: null, cameraSamples: [],
  wallCaptures: [], solutions: { A: null, B: null }, preview: 'existing',
  foreground: null, opencv: null,
};

function setDashboardRail(mode) {
  calibrationSession.mode = mode;
  $('operations-rail').hidden = mode !== 'operations';
  $('calibration-rail').hidden = mode !== 'calibration';
  $('calibration-btn').setAttribute('aria-pressed', String(mode === 'calibration'));
  renderCalibrationRail();
}

function resetCalibrationSession() {
  for (const sample of calibrationSession.cameraSamples) sample.bitmap?.close?.();
  for (const capture of calibrationSession.wallCaptures) capture.bitmap?.close?.();
  Object.assign(calibrationSession, {
    mode: calibrationSession.mode, stage: 'camera', camera: null, cameraSamples: [],
    wallCaptures: [], solutions: { A: null, B: null }, preview: 'existing', foreground: null,
  });
  renderCalibrationRail();
}
~~~

No calibration handler may call ws.send, motor/winch/steer command functions, ARM, or servo power.

- [ ] **Step 4: Run contract test and visual smoke check**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS.

Start npx --yes serve . -l 8000. Inspect desktop and mobile: the camera remains large and only the full right rail swaps.

- [ ] **Step 5: Commit**

~~~bash
git add main/dashboard.html tests/dashboard-calibration-core.test.mjs
git commit -m "feat: add dashboard calibration rail"
~~~

### Task 3: Implement Upright Snapshot, ChArUco Preflight, and Camera Intrinsics

**Files:**
- Modify: main/dashboard.html calibration helpers next to existing snapCamToDetectCanvas near lines 1440-1475.
- Test: tests/dashboard-calibration-core.test.mjs

**Interfaces:**
- Consumes: calibrationSession, current ESP host fallback logic, snapshot endpoint, CalibrationCore.
- Produces: loadOpenCvForCalibration(), fetchUprightCalibrationSnapshot(), preflightWallBoard(bitmap), captureCameraCalibrationSample(), solveCameraIntrinsics(samples), calibrationSession.camera.

- [ ] **Step 1: Write failing camera quality/projection tests**

~~~js
test('upright projection uses 800x640 display coordinates', () => {
  const core = loadCalibrationCore();
  const pixel = core.projectZoneToImage({
    row: 3, col: 3, distanceMm: 1000,
    sensorTransform: { translationMm: [0, 0, 0],
      rotationDeg: { yaw: 0, pitch: 0, roll: 0 }, rangeBiasMm: 0, mapping: 'identity' },
    intrinsics: { fx: 600, fy: 600, cx: 400, cy: 320, distortion: [0, 0, 0, 0, 0] },
    mapping: 'identity',
  });
  assert.ok(pixel.x > 350 && pixel.x < 450);
  assert.ok(pixel.y > 270 && pixel.y < 370);
});

test('camera quality needs ten views and RMS at most one pixel', () => {
  const core = loadCalibrationCore();
  assert.equal(core.cameraQuality({ sampleCount: 9, rmsPx: 0.2 }).state, 'unstable');
  assert.equal(core.cameraQuality({ sampleCount: 10, rmsPx: 1.01 }).state, 'unstable');
  assert.equal(core.cameraQuality({ sampleCount: 10, rmsPx: 0.65 }).state, 'ready');
});
~~~

- [ ] **Step 2: Run test to verify failure**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: FAIL because projectZoneToImage and cameraQuality are missing.

- [ ] **Step 3: Implement browser camera workflow**

projectZoneToImage uses normalized nominal ray, solved yaw/pitch/roll, translation, distortion-aware intrinsics, and returns x, y, inFront in upright pixel coordinates. It does not use old FX, CX, CY, GRID_SCALE, drag offsets, SENSOR_A, or SENSOR_B.

Load OpenCV only after opening Calibration. Assign window.Module.onRuntimeInitialized before adding the script for https://docs.opencv.org/4.10.0/opencv.js. Reject after 15 seconds or when cv.aruco is unavailable.

Fetch a fresh same-origin snapshot using the current host choice and cache no-store. Draw the bitmap into 800 x 640 with exactly:

~~~js
ctx.translate(800, 640);
ctx.rotate(Math.PI);
ctx.drawImage(bitmap, 0, 0, 800, 640);
~~~

Never draw the live MJPEG image into a calibration canvas.

Use:

~~~js
const WALL_BOARD = { squaresX: 7, squaresY: 5, squareMm: 35, markerMm: 26, startId: 0 };
const FOREGROUND_BOARD = { squaresX: 5, squaresY: 3, squareMm: 35, markerMm: 26, startId: 20 };
const DICTIONARY_CANDIDATES = ['DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000'];
~~~

Detect markers and interpolate ChArUco for every candidate. Pick the smallest topology-consistent candidate, report decoded IDs, and block when layouts disagree, no candidate works, or fewer than 12 ChArUco corners appear. Choosing the smallest compatible dictionary resolves DICT_4X4 family overlap for lower-numbered markers.

Each manual capture fetches new snapshot, rejects fewer than 12 corners, Laplacian-variance blur score below 80 on the grayscale 800 x 640 canvas, or centroid duplicate within 8 percent of image width and height. It stores corners, IDs, bitmap, and coverage location. At 10 samples run calibrateCameraCharuco for 800 x 640, retain fx/fy/cx/cy, five distortion values, poses, and RMS. Ready is 10+ samples and RMS <= 1.0 px. Release every OpenCV Mat/vector/board/dictionary after use.

- [ ] **Step 4: Test**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS.

In browser with live or saved wall-board snapshot, preflight must show candidate, IDs, corners, and upright annotation; it must not use MJPEG canvas capture.

- [ ] **Step 5: Commit**

~~~bash
git add main/dashboard.html tests/dashboard-calibration-core.test.mjs
git commit -m "feat: add Charuco camera calibration"
~~~

### Task 4: Implement Stable Wall Capture and Independent ToF Extrinsic Solves

**Files:**
- Modify: main/dashboard.html CalibrationCore and calibration rail.
- Modify: tests/fixtures/dashboard-calibration-fixtures.mjs
- Modify: tests/dashboard-calibration-core.test.mjs

**Interfaces:**
- Consumes: ready calibrationSession.camera, decoded lastTofA/lastTofB, SensorSnapshot timestampUs, upright wall poses.
- Produces: collectStableTofCapture(), captureWallPose(), solveSensorTransform(), evaluateSensorSolution(), wallCaptures, independent A/B solutions.

- [ ] **Step 1: Write synthetic fixtures and failing solver tests**

~~~js
export const SYNTHETIC_A = {
  translationMm: [-23, 4, 8],
  rotationDeg: { yaw: -5.4, pitch: 0.7, roll: -0.4 },
  rangeBiasMm: 11, mapping: 'identity',
};
export const SYNTHETIC_B = {
  translationMm: [24, 3, 9],
  rotationDeg: { yaw: 4.7, pitch: -0.5, roll: 0.6 },
  rangeBiasMm: -7, mapping: 'flipV',
};
~~~

Generate ten distinct camera-frame wall planes, covering different distances, yaw signs, and small pitch changes, by intersecting inverse-transformed zone rays with each plane. Test recovery for both sensor transforms and mapping, then test a deliberately corrupt solution gives unstable due to heldout error.

~~~js
test('independent solve recovers transform and map', () => {
  const core = loadCalibrationCore();
  const solution = core.solveSensorTransform({
    captures: makeSyntheticCaptures('A'), sensorKey: 'A', initialTransform: SYNTHETIC_A,
  });
  assert.equal(solution.mapping, 'identity');
  assert.ok(Math.abs(solution.translationMm[0] - SYNTHETIC_A.translationMm[0]) <= 2);
  assert.ok(Math.abs(solution.rotationDeg.yaw - SYNTHETIC_A.rotationDeg.yaw) <= 0.25);
  assert.ok(Math.abs(solution.rangeBiasMm - SYNTHETIC_A.rangeBiasMm) <= 3);
});
~~~

- [ ] **Step 2: Run test to verify failure**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: FAIL because solver functions still throw.

- [ ] **Step 3: Implement sampling and robust solve**

In the existing message.sensors branch, record timestamp identity and observe, without changing the normal decoder/rendering path:

~~~js
lastSensorTimestampUs = Number(s.timestampUs ?? 0);
calibrationSampler?.observe({
  timestampUs: lastSensorTimestampUs, tofA: lastTofA, tofB: lastTofB,
});
~~~

Sampler counts a new observation only when timestamp changes; if absent/unchanged, use deterministic fingerprint of distance, sigma, status, target-count arrays so cached values are not counted twice.

Manual Capture starts 2-5 second collection. Save eight unique observations. Reduce A/B independently, require 32 stable zones each, then fetch fresh upright snapshot and wall board. Reject whole pose if any part fails. Show ChArUco-derived actual yaw/distance only; consume neither compass nor tape input.

Geometry is:

~~~js
const ray = makeZoneRay(row, col, mapping, 45, 45);
const rangeMm = zone.distanceMm + transform.rangeBiasMm;
const tofPoint = ray.map(value => value * rangeMm);
const cameraPoint = transformPoint(tofPoint, transform);
const residualMm = pointPlaneResidual(cameraPoint, capture.plane);
~~~

wallPlaneFromPose uses Rodrigues: normal equals R times [0,0,1], d equals negative dot(normal,tvec).

Solve A and B separately. Enumerate all eight mappings. For each, run multi-start Nelder-Mead over tx,ty,tz,yaw,pitch,roll,rangeBias using current SENSOR_A/SENSOR_B translation and tilt only as starting geometry. Use yaw starts -10,-5,0,5,10 degrees around nominal; simplex scales 10,10,10,2,2,2,10; reflection 1, expansion 2, contraction 0.5, shrink 0.5, max 300 iterations. Cost is 30 mm Huber loss weighted by statusWeight divided by max(sigmaMm,1) squared. Keep lowest robust loss; do not alter existing overlay constants.

Fit eight captures; select two heldouts maximizing yaw-sign and distance separation. Leave-one-fit-out repeats give spread. Ready requires heldout median <= max(30, twice median accepted sigma), yaw spread <= 0.5 degree, translation spread <= 10 mm. Return state, medianHeldoutErrorMm, yawSpreadDeg, translationSpreadMm, and reasons.

- [ ] **Step 4: Run solver tests**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS for both transforms, mapping, rejected zones, and bad heldout verdict with no browser dependencies.

- [ ] **Step 5: Commit**

~~~bash
git add main/dashboard.html tests/fixtures/dashboard-calibration-fixtures.mjs \
  tests/dashboard-calibration-core.test.mjs
git commit -m "feat: solve dashboard ToF extrinsics"
~~~

### Task 5: Render Calculated Sparse Depth and Foreground Validation

**Files:**
- Modify: main/dashboard.html existing renderOverlay region, calibration rail, preview events.
- Modify: tests/dashboard-calibration-core.test.mjs
- Create: tests/dashboard-calibration-ui.spec.mjs
- Create: playwright.config.mjs

**Interfaces:**
- Consumes: existing tof-overlay, current ToFs, ready intrinsics and A/B solutions, projectZoneToImage.
- Produces: renderCalculatedCalibrationOverlay(), setCalibrationPreview(mode), captureForegroundValidation(), calibrationSession.foreground.

- [ ] **Step 1: Write failing projection/foreground tests**

~~~js
test('calculated projection returns finite upright pixels', () => {
  const core = loadCalibrationCore();
  const projected = core.projectZoneToImage({
    row: 0, col: 0, distanceMm: 1200, sensorTransform: SYNTHETIC_A,
    intrinsics: { fx: 620, fy: 620, cx: 400, cy: 320, distortion: [0, 0, 0, 0, 0] },
    mapping: SYNTHETIC_A.mapping,
  });
  assert.equal(typeof projected.inFront, 'boolean');
  assert.ok(Number.isFinite(projected.x));
  assert.ok(Number.isFinite(projected.y));
});

test('foreground overlap remains informational', () => {
  const core = loadCalibrationCore();
  const score = core.foregroundOverlapScore({ zonePolygons: [], foregroundPolygon: [] });
  assert.equal(score.state, 'informational');
  assert.equal(score.acceptanceOverride, false);
});
~~~

- [ ] **Step 2: Run test to verify failure**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: FAIL because foregroundOverlapScore is not defined.

- [ ] **Step 3: Implement preview and validation**

Leave current renderOverlay untouched for Existing preview. Add segmented Existing and Calculated controls. Calculated stays disabled until camera and both sensor results are Ready.

For valid status 5/9 zones, project centre plus four half-zone corners with solved map, transform, and bias. Skip behind-camera or fewer-than-three-finite polygons. Draw low-opacity distance-colored quads with subtle sensor A/B outline. Leave unmeasured pixels transparent and never blur/interpolate/fill dense depth.

Foreground uses 5 x 3 start-ID-20 board on rigid backing in front of wall board. Capture stable grids plus fresh upright snapshot, detect both boards, calculate foreground polygon, compute total zone intersection area divided by foreground area, and display percent plus covered zones with informational only badge. It cannot change Ready/Unstable.

Show read-only A/B mapping, translation, yaw/pitch/roll, range bias, stable zones, loss, heldout error, spread, and reasons. Calibration controls are capture, reset, and existing/calculated only. Do not add manual calibration sliders, saving, or export.

- [ ] **Step 4: Run unit and browser smoke checks**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS.

Create playwright.config.mjs with one Chromium project, testDir tests, and a reusable local server:

~~~js
import { defineConfig, devices } from '@playwright/test';
export default defineConfig({
  testDir: './tests',
  use: { ...devices['Desktop Chrome'], baseURL: 'http://127.0.0.1:8000' },
  projects: [{ name: 'chromium', use: { ...devices['Desktop Chrome'] } }],
  webServer: { command: 'npx --yes serve . -l 8000', url: 'http://127.0.0.1:8000', reuseExistingServer: true },
});
~~~

Install the browser once with npx playwright install chromium. Create Playwright fixture that stubs WebSocket and snapshot, then run:

~~~bash
npx playwright test tests/dashboard-calibration-ui.spec.mjs --project=chromium
npx playwright test tests/dashboard-calibration-ui.spec.mjs --project=chromium --headed
~~~

Expected: desktop/mobile retain dominant camera, no nested cards, full rail returns to Operations, calculated cells use upright coordinates.

- [ ] **Step 5: Commit**

~~~bash
git add main/dashboard.html tests/dashboard-calibration-core.test.mjs \
  tests/dashboard-calibration-ui.spec.mjs playwright.config.mjs
git commit -m "feat: preview calibrated ToF depth"
~~~

### Task 6: Run Full Regression and Physical Bench Acceptance

**Files:**
- Modify: tests/dashboard-calibration-core.test.mjs only for demonstrated missing edge cases.
- Modify: main/dashboard.html only for demonstrated faults.

**Interfaces:**
- Consumes: all prior interfaces.
- Produces: verified dashboard-only calibration with unchanged transport behavior.

- [ ] **Step 1: Add final static contract test**

~~~js
test('calibration stays session-only and transport APIs remain', () => {
  const html = readFileSync('main/dashboard.html', 'utf8');
  assert.doesNotMatch(html, /localStorage|sessionStorage|indexedDB|JSON\.stringify\([^)]*calibration/i);
  assert.match(html, /startStream\(\);/);
  assert.match(html, /connectWS\(\);/);
  assert.match(html, /sendMotorCommand\(\)/);
  assert.match(html, /sendWinchCommand\(speed\)/);
  assert.match(html, /sendSteerCommand\(\)/);
});
~~~

- [ ] **Step 2: Run automated gate**

Run: node --test tests/dashboard-calibration-core.test.mjs

Expected: PASS for mapping, stability, camera quality, synthetic recovery, trust verdict, foreground score, and static contracts.

Run: git diff --check

Expected: no whitespace errors.

Run: git diff -- main/dashboard.html main/CMakeLists.txt main/http_server.c

Expected: production changes only in dashboard.html; CMake and HTTP server are untouched.

- [ ] **Step 3: Physical camera stage**

1. Fix the 280 x 195 mm wall board flat against matte wall/panel; prevent room corners entering ToF fields.
2. Open Calibration and complete preflight.
3. Capture 10 useful images at varied board position, distance, yaw, pitch, roll, and image location.
4. Confirm RMS <= 1.0 px and broad coverage; repeat weak samples.

Expected: Ready camera intrinsics in upright frame, no compass/tape input.

- [ ] **Step 4: Physical wall/ToF stage**

1. Capture stationary poses near yaw -20,-10,0,+10,+20 at one close and one farther wall distance.
2. For every pose wait for eight unique updates plus fresh snapshot.
3. Accept only detected board and 32+ stable zones for both sensors.
4. Verify eight fit poses and two meaningfully different heldouts.
5. Inspect independent Ready A/B results before Calculated preview.

Expected: two independently calculated physical transforms and no firmware changes.

- [ ] **Step 5: Foreground and Operations acceptance**

1. Place the 200 x 120 mm board on rigid backing in front of wall board.
2. Capture validation and compare Existing versus Calculated. Confirm each ToF field lands in the correct foreground image region.
3. Treat overlap as diagnostic only.
4. Return to Operations and verify telemetry, mini ToF grids, controls, Detect, Snap, and Snap+Overlay.
5. Reload and verify calibration values clear while live dashboard reconnects.

Expected: no persisted calibration and no automatic motor command on rail entry/exit.

- [ ] **Step 6: Commit demonstrated final fixes only**

~~~bash
git add main/dashboard.html tests/dashboard-calibration-core.test.mjs \
  tests/dashboard-calibration-ui.spec.mjs
git commit -m "test: cover dashboard calibration workflow"
~~~

Do not commit .superpowers, boatMainReimagine.FCStd, screenshots, browser binaries, or firmware files.

## Plan Self-Review

### Spec coverage

- Dashboard-only/session-only/no API change: Tasks 2-6.
- Camera stays large and full right rail swaps: Task 2.
- Upright snapshot and ChArUco camera calibration: Task 3.
- 10 camera captures, 10 wall poses, stable sampling, no compass/tape: Tasks 3,4,6.
- Independent transforms, uncertain grid orientation, trust criteria: Tasks 1 and 4.
- Sparse overlay and informational foreground score: Task 5.
- Desktop/mobile and preservation regression: Tasks 5 and 6.

### Placeholder scan

Every task has exact files, named interfaces, a failing test, verification command and outcome, implementation algorithm, and commit boundary.

### Type consistency

WallCapture, SensorTransform, calibrationSession, CalibrationCore, lastTofA, lastTofB, timestampUs, distanceMm, sigmaMm, statusWeight, and mapping retain their meanings across tasks. Geometry remains millimetres and display coordinates remain upright 800 x 640.
