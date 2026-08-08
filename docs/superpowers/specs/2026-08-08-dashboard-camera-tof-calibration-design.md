# Dashboard Camera-ToF Calibration Design

Date: 2026-08-08
Status: Approved through interactive brainstorming

## Purpose

Add a temporary, dashboard-only bench workflow that:

1. Calibrates the upside-down physical camera in the existing human-upright browser coordinate system.
2. Independently estimates the physical placement of ToF A and ToF B relative to the camera.
3. Previews a camera-aligned ToF depth heatmap using the calculated geometry.
4. Validates the alignment against held-out wall poses and a foreground object.

The first milestone is a visual bench proof. It does not persist calibration or use it for control.

## Scope Boundaries

All feature work is confined to `main/dashboard.html` and dashboard test/support files.

The design does not change:

- Firmware tasks, drivers, or scheduling.
- `boat.proto` or any WebSocket message.
- The MJPEG stream or `/snapshot` API.
- Motor, winch, arm, detection, or telemetry API calls.
- ESP-NOW behavior.
- NVS.
- Internal obstacle avoidance.
- PicoDet training or inference.
- Existing overlay constants.

The calculated calibration exists only in JavaScript memory. Reloading the dashboard restores the existing behavior and discards every captured image and result.

## Approved User Workflow

The camera remains the dominant dashboard surface. The complete right rail switches between two modes:

```text
Operations rail <-> Calibration rail
```

Operations retains the existing telemetry, controls, and ToF displays. Calibration hides that rail without destroying its DOM state, changing event handlers, or sending a motor command. Returning to Operations reveals the unchanged controls and telemetry.

The calibration flow is:

```text
Board preflight
  -> ten useful camera captures
  -> temporary camera intrinsic solution
  -> ten useful wall/ToF poses
  -> independent ToF A and ToF B solutions
  -> existing/calculated overlay comparison
  -> informational foreground-object validation
```

The Capture buttons are manual. After a wall-pose Capture, the dashboard automatically waits for stable ToF updates and then fetches a fresh `/snapshot`.

## Physical Targets

### Camera and wall board

- OpenCV board size: `squaresX=7`, `squaresY=5`.
- Active grid: 245 x 175 mm.
- PDF: 280 x 195 mm landscape.
- Nominal square length: 35 mm.
- Nominal marker length: 26 mm.
- Dictionary family shown by the generator: `DICT_4X4`.

The user measures the printed square length and enters that measured value once. Printer scaling is not assumed to be exact.

The dashboard tests OpenCV's predefined `DICT_4X4_50`, `_100`, `_250`, and `_1000` variants against the first clear image. A candidate is accepted only when it consistently decodes the expected board geometry and IDs. Ambiguous or incomplete detection blocks camera calibration and reports the detected IDs.

The board is attached flush to a large, flat, matte wall or panel. The wall, not the paper, fills the ToF fields of view.

### Foreground validation target

- OpenCV board size: `squaresX=5`, `squaresY=3`.
- Active grid: 175 x 105 mm.
- PDF: 200 x 120 mm.
- Square length: 35 mm.
- Marker length: 26 mm.
- Dictionary family: the same detected `DICT_4X4` variant as the wall board.
- Starting marker ID: 20.

The foreground print is attached to a larger rigid matte backing so it occupies multiple ToF zones. The dashboard also supports an unmarked foreground object for visual-only testing.

Foreground overlap is informational. It never acts as a hard success gate.

## Coordinate Convention

The physical camera is upside down, and the existing dashboard already presents it upright:

- The live `#cam-img` uses CSS `rotate(180deg)`.
- The existing detection snapshot path rotates `/snapshot` by 180 degrees on a canvas.

Calibration reuses the detection snapshot convention. It does not add another rotation.

All temporary camera intrinsics, ChArUco corners, wall planes, ToF projections, and overlay pixels use the upright 800 x 640 browser coordinate system. A calibrated pixel therefore draws directly on the unrotated overlay canvas above the visually upright camera.

## Browser Components

### Calibration mode controller

Owns the Operations/Calibration rail switch. It only changes dashboard presentation. It must not send motor commands or recreate existing API event handlers.

### OpenCV loader and board preflight

Loads an ArUco/ChArUco-capable OpenCV.js build from the internet. If loading fails or the build lacks the required module, Calibration shows an error and Operations remains functional.

Board preflight detects the exact dictionary variant, verifies dimensions and marker IDs, and locks the board definition for the temporary session.

### Snapshot normalizer

Fetches the existing `/snapshot` route with cache disabled, creates an image bitmap, and applies the existing 180-degree canvas rotation. MJPEG remains the live human view and is not read into a calibration canvas.

### ToF stability sampler

Observes the existing `lastTofA` and `lastTofB` WebSocket values without changing the telemetry decoder. For a wall capture it:

1. Collects at least eight distinct updates over a two-to-five-second stationary interval. It de-duplicates updates by the existing telemetry timestamp when available, with a full-grid fingerprint as the fallback, so repeated reads of the cached `lastTofA` and `lastTofB` values do not count as new samples.
2. Keeps distance, sigma, target status, and target count per zone.
3. Uses status 5 at full weight, status 9 at reduced weight, and excludes other statuses.
4. Requires at least 32 usable zones per sensor.
5. Computes a per-zone median and median absolute deviation.
6. Marks a zone unstable when its deviation exceeds `max(20 mm, 2 * median sigma)`.
7. Rejects the capture when fewer than 32 zones remain stable in either sensor.

IMU changes are displayed as a motion warning but do not supply calibration yaw or distance. ChArUco supplies wall-relative pose.

Because firmware is unchanged, camera and ToF are not hardware-synchronized. The stationary interval and immediate `/snapshot` are explicit limitations of this bench-only design.

### Temporary session store

Stores camera samples, wall captures, candidate parameters, validation results, and bitmaps in memory. It does not use Local Storage, IndexedDB, JSON export, cookies, or NVS. Reset Session and page reload both release image resources and return to the existing overlay.

### Solver worker

Runs camera/ToF fitting outside the main UI thread. Progress updates are read-only. Candidate parameters are not applied to the visible overlay until a complete result exists.

### Overlay renderer

Provides an atomic Existing/Calculated toggle. Existing always means the checked-in dashboard constants. Calculated uses the temporary camera and ToF solutions. Optimization never gradually moves the live overlay.

## Stage 1: Camera Calibration

The user manually positions the large board and presses Capture. Each capture:

1. Fetches and normalizes `/snapshot`.
2. Detects markers and ChArUco corners.
3. Rejects an incomplete, blurry, duplicate, or poorly distributed observation.
4. Adds the accepted observation to a coverage display.

Solve becomes available after ten useful captures, not merely ten button presses. The set must include image-center, edge, and corner coverage plus meaningful distance and orientation changes.

OpenCV calculates `fx`, `fy`, `cx`, `cy`, lens distortion, and per-view reprojection error in upright 800 x 640 coordinates. An RMS reprojection error above 1.0 pixel or weak coverage labels the result Unstable and recommends additional captures. The user may still inspect it, but it is not presented as a successful camera solution.

## Stage 2: Wall and ToF Capture

The initial ten-pose target is:

```text
Closer distance: approximately -20, -10, 0, +10, +20 degrees
Farther distance: approximately -20, -10, 0, +10, +20 degrees
```

These are positioning guides only. The compass is optional and never enters the solver. ChArUco measures the actual wall distance, yaw, pitch, and roll after each capture.

The boat may be translated, lifted, rotated, or moved closer to the wall between captures. The camera/ToF enclosure must remain rigid, and the boat must be stationary during collection.

Each accepted wall capture contains:

- A fresh upright snapshot.
- Detected board corners and IDs.
- The wall plane in camera coordinates.
- Summarized ToF A and ToF B pads.
- Per-zone sigma, status, stability, and weight.
- Capture timing and informational IMU motion range.

Captures showing a second plane, abrupt room-corner discontinuity, insufficient board detection, or insufficient stable zones are rejected with one clear reason.

## Stage 3: Independent ToF Solves

ToF A and ToF B are fitted independently. Each fit starts from the corresponding CAD pose but does not assume the CAD's nominal outward angle is physically exact.

The unknown parameters per sensor are:

```text
translation: tx, ty, tz
rotation:    yaw, pitch, roll
range:       one sensor-wide bias
```

The first version does not fit independent angular or distance corrections for all 64 zones. Per-zone corrections would add enough freedom to hide a wrong physical transform.

### Zone rays and discrete mapping

The solver builds nominal 8 x 8 zone-center rays and four boundary rays per zone from the VL53L5CX square field of view. It tests all eight square-grid orientation mappings: four rotations, with and without reflection. The mapping with the best held-out wall prediction is selected. A mapping that wins only on fitting poses but fails validation is rejected.

### Wall residual

For wall pose `k`, zone `i`, measured range `rho`, nominal zone ray `r_i`, candidate rotation `R`, and translation `t`:

```text
point_camera = R * (rho * r_i) + t
residual     = distance from point_camera to ChArUco wall plane k
```

The solver minimizes robust, sigma-weighted residuals across accepted zones and fitting poses. Status 5 has full weight, status 9 has reduced weight, and isolated outliers are limited by a robust loss.

CAD is the starting point and plausibility reference, not ground truth. Broad physical bounds prevent numerical runaway. A solution that reaches a bound is labeled Unstable rather than silently accepted.

### Fit and holdout split

With ten accepted wall poses, eight are used for fitting and two are reserved for validation. Holdouts are chosen to include opposite yaw signs and different distances when possible.

The worker repeats the fit over multiple resampled capture subsets. The result reports parameter spread rather than only the single lowest-error solution.

## Trust and Result States

Each camera and ToF result has one of three states:

```text
Ready      enough useful data to solve
Stable     solution passes coverage, repeatability, and holdout checks
Unstable   preview is allowed, but the result is not represented as trustworthy
```

The Calibration rail reports, separately for A and B:

- Calculated yaw, pitch, roll, and translation.
- Calculated range bias.
- Selected zone mapping.
- Valid/stable zone percentage.
- Fit and held-out plane error.
- Parameter variation across repeated fits.
- Difference from CAD.
- A/B agreement in their overlapping camera region.

Initial stability guidance is:

- Camera RMS reprojection error at or below 1.0 pixel.
- Median held-out plane error at or below `max(30 mm, 2 * median accepted sigma)`.
- Repeated-fit yaw spread at or below 0.5 degrees.
- Repeated-fit translation spread at or below 10 mm.
- At least 32 stable zones per sensor per accepted capture.

These values are visible dashboard constants rather than hidden assumptions. The bench result and foreground test determine whether they are appropriate for this hardware.

## Preview and Depth Heatmap

The Calculated overlay projects every valid ToF zone independently:

1. Convert the measured radial range and calibrated zone ray into a ToF-frame point.
2. Apply that sensor's calculated rotation and translation.
3. Project the point and zone boundary rays with the calculated camera intrinsics and distortion.
4. Draw a camera-aligned quadrilateral colored by camera-forward depth.

Zones outside the camera remain available only in the measured pad view. Invalid zones are transparent and explicitly unknown. Visual interpolation may smooth the display, but measured zones remain visible and interpolation is never presented as measured depth.

In A/B overlap:

- Similar valid depths are displayed as agreement.
- A valid result from only one sensor is displayed with reduced confidence.
- Conflicting depths are displayed as conflict, not averaged into false certainty.

The heatmap is a sparse camera-aligned visualization, not a dense per-pixel depth map.

## Foreground Validation

After wall fitting, the user places a closer rigid object in front of the background wall and moves it through left-only, central overlap, right-only, upper, and lower regions.

The expected behavior is:

- Background zones remain at wall distance.
- Zones seeing the foreground object jump to a nearer distance.
- The corresponding projected quadrilaterals visually overlap the object.
- ToF A and B behave correctly in both separate and shared areas.

If the small ChArUco target is detected, the dashboard computes an informational overlap score. An unmarked object is supported for visual judgment. Neither score nor visual result changes existing dashboard constants.

## Dashboard Presentation

The camera stays large in both modes.

Operations rail contains the existing telemetry, mini ToF pads, and controls. Calibration rail contains:

```text
1. Board preflight and camera captures
2. Camera solution
3. Wall pose and stability countdown
4. ToF A/B capture and solve status
5. Existing/Calculated preview toggle
6. Foreground validation
7. Reset Session
```

Detailed measured, predicted, and error pads remain in the rail. The camera shows only useful visual evidence: board corners, projected ToF polygons, detected foreground target, and concise overlay state.

Errors are local to Calibration mode and give one actionable reason at a time. A Calibration failure never disables Operations.

## Testing Strategy

### Synthetic solver tests

Generate known camera intrinsics, wall poses, independent ToF transforms, range noise, invalid zones, flipped zone maps, and outliers. Verify recovery of the original transform and correct rejection of ambiguous or insufficient data.

Synthetic tests cover:

- Rotation sign and coordinate convention.
- The existing 180-degree browser orientation.
- All zone-map candidates.
- Independent A/B fitting.
- Robust handling of status, sigma, outliers, and missing zones.
- Fit/holdout separation.
- Unstable-result detection.

### Dashboard regression tests

Verify that:

- Existing WebSocket/protobuf handling is unchanged.
- Existing camera, motor, winch, arm, and detection request shapes are unchanged.
- Entering Calibration sends no motor request.
- Returning to Operations preserves control and telemetry state.
- OpenCV/CDN failure affects only Calibration.
- Existing is always the default overlay after reset or reload.
- Object URLs, image bitmaps, workers, and session memory are released on reset.

### Physical bench acceptance

The first proof uses ten useful camera captures and ten useful wall poses. If coverage or stability is insufficient, the dashboard requests additional captures instead of treating the count as proof.

Physical acceptance requires:

1. A usable camera solution in the upright display frame.
2. Independent A/B solutions that remain similar under resampling.
3. Reasonable held-out wall predictions.
4. A visibly improved Calculated overlay relative to Existing.
5. Foreground depth zones that follow the target through each relevant camera/ToF region.

The user makes the final visual judgment. No result is persisted or sent to firmware.

## Explicit Non-Goals

- Dense monocular depth estimation.
- Pixel-perfect object boundaries from 8 x 8 ToF grids.
- Hardware-synchronized image and ToF capture.
- Raw unfiltered ToF acquisition.
- Saving calibration to NVS, files, or browser storage.
- Using calibration for motor control or obstacle avoidance.
- Replacing the existing dashboard overlay constants.
- Modifying camera rotation in firmware.
