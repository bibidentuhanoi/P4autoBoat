# Waypoint mission, step 1: live map + a cleaner control page

**Date:** 2026-09-04
**Branch / worktree:** `feat/waypoint-mission` at `.claude/worktrees/waypoint-mission`
**Scope:** `tools/espnow_drive.py` (the laptop control page) and its tests.
No firmware change. No change to what the boat sends.

## Why

The control page already receives the boat's GPS position 20 times a second
(field telemetry: `lat`, `lon`, `gps_valid`, speed, course, satellites, HDOP,
plus the IMU heading). It only ever showed those as numbers in a card. The next
piece of work on this branch is waypoint missions, which need a real map with
the boat drawn on it. This step builds that map and tidies the page so the map
has room, without removing any card or any data row.

Two things get fixed on the way because they were found while reading the page:

1. **STOP left the Left/Right sliders where they were.** With "Link L+R"
   unticked, pressing STOP reset only the linked throttle and the rudder. The
   boat was stopped, but the per-motor sliders kept showing the old value, and
   the next nudge jumped that motor straight back to it. The same gap existed on
   DISARM, on a fresh control session, and when the tab was hidden.
2. **A stray `</div>`** after the rudder test card, and page/docstring text that
   still claimed "MotorStatus isn't decoded here" although it has been for
   weeks.

## Layout

Wide screen (laptop, 1180 px and up): three columns.

```
+----------------------+--------------------------------+----------------------+
| Connect              | MAP                            | Telemetry            |
| Drive (ARM, sliders, |   menu: layer / follow /       | Motor (confirmed)    |
|   winch, STOP)       |         centre / clear trail   | Sensors (boot check) |
| > Throttle mismatch  |   readout: lat/lon speed ...   | > Bridge (S3)        |
| > Rudder test        |   [ tiles, boat arrow, trail ] |                      |
+----------------------+--------------------------------+----------------------+
| footer                                                                       |
+------------------------------------------------------------------------------+
```

- Left column = things you do. Right column = things the boat tells you.
  The map sits between them and stays on screen (`position: sticky`) while
  either column scrolls.
- The two test cards fold (`<details>`), collapsed by default. Sensors and
  Bridge fold too, open by default. Each card remembers its state in
  `localStorage`. Connect, Drive, Telemetry and Motor never fold: STOP and the
  live truth must always be visible.
- 780 to 1180 px: two columns, controls and status stacked on the left, map on
  the right spanning both. Under 780 px: one column, controls, then map (70 vh,
  not sticky), then status.
- Card markup inside each card is byte-for-byte unchanged. Only the wrappers,
  the four `details`/`summary` conversions and the footer text change.

## Map

- **Library:** Leaflet 1.9.4 from unpkg, loaded by the page at runtime (not in
  `<head>`) with subresource-integrity hashes, so a machine with no internet
  still gets the rest of the page instantly and the map box says why it is
  empty. The "Centre" button doubles as retry.
- **Layers ("function menu"):** Google Hybrid (default), Google Satellite,
  Google Streets, OpenStreetMap. The Google layers use the public
  `mt{0-3}.google.com/vt` tile endpoints, which need no API key. That is common
  in hobby tools but is not covered by Google's terms; OpenStreetMap is the
  clean fallback and is one click away. Choice is remembered.
- **Boat marker:** an arrow rotated to the IMU heading. Dimmed when telemetry
  is stale. It stays at the last known position when the fix is lost; it never
  jumps to 0,0.
- **Trail:** a polyline of past fixes. First fix zooms to 17. "Follow boat" is
  on by default and switches itself off when you drag the map; "Centre" turns
  it back on. "Clear trail" clears both the page and the tool's own copy.
- **Readout strip** above the map repeats the key GPS numbers (lat/lon to six
  places, speed in m/s and km/h, course, heading, sats/HDOP, trail length) so
  they are next to the picture. The Telemetry card keeps its rows too.
- Last known position is remembered in `localStorage` so the page opens on the
  right lake next time, at zoom 16, with a note that it is old.

## Trail lives in the Python process

`GpsTrack` (new, pure, tested) keeps up to 5000 fixes that moved at least
1.5 m from the previous kept one, drops the (0,0) "null island" and any
non-finite or out-of-range value. `BoatLink` owns one and feeds it from both
telemetry decoders (field struct and legacy protobuf). Why in Python and not
only in the page: a reload or a second browser tab gets the trail back with one
request, and a future mission planner will live on this side too.

- `GET /api/track` -> `{points: [[lat, lon], ...], max_points, min_move_m}`
- `POST /api/track/clear` -> `{ok: true}`
- `status()` gains one integer, `gps_track_points`, so the page can notice a
  clear or a restart and re-fetch. The points themselves are never in the 20 Hz
  status push.
- The page applies the same 1.5 m / 5000 rule locally between fetches; a test
  pins the two constants to each other.

Links built by the tests with `__new__` may not have a track; recording is then
simply skipped (`getattr`), never an error.

## The slider fix

One helper, `zeroDriveUI()`, sets all four sliders (throttle, left, right,
rudder) and their labels to 0 and resets the heartbeat state, keeping the
operator's link/unlink choice. It is called wherever the boat is put at zero:
connect, a new control session, STOP, DISARM, tab hidden, and calibration start
and stop (during a calibration the tool sends only the keepalive, so the
operator's throttle must not be silently resumed when it ends).

## Error handling

- Leaflet fails to load or times out (20 s): note in the map box, page
  unaffected, retry via Centre.
- Tiles fail (offline): one-line note, cleared when tiles load again.
- `localStorage` missing or throwing: ignored, defaults used.
- Bad GPS values (NaN, out of range, 0/0): not drawn, not recorded.

## Testing

- `GpsTrack` unit tests (filter, cap, clear, bad input).
- Telemetry decode tests: both frame types record a fix; `gps_valid=0` does
  not; a link without a track does not crash.
- HTTP tests for the two new routes and the status field.
- Page structure tests: all card ids and rows still present, balanced
  `<div>`/`<details>`, map ids, Leaflet URL + hashes, constants agree.
- Node (vm) tests running the real page JS against a fake DOM and a fake
  Leaflet: first fix moves the marker and zooms, a repeat position adds no trail
  point, no-fix does not move the marker, stale dims it; and STOP in per-motor
  mode zeroes every slider and the next heartbeat carries zeros.
- Whole suite must stay green (baseline 533 passed on this commit once the
  git-ignored `sdkconfig` is copied into the worktree).
