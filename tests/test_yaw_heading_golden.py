"""The working P and auto-trim must not change (Kiet, 2026-09-25).

The heading hold gained two input fields for the mission (mission_owned,
target_heading_deg).  With them false/0 -- every existing caller -- the new
yaw_heading_control.c must give exactly the outputs, and keep exactly the
state, of the version the boat was proven with (c8a4c5e), bit for bit, over:
  * ~200 000 seeded random inputs in episodes: P on/off, driving or not,
    stale gyro, invalid heading, steering and recapture, throttle below the
    floor, headings across north, dt jitter, NaN/inf, resets;
  * every recorded lake row with the controller's own inputs, when dataout/
    is present (skipped otherwise; the random stream always runs).
"""
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASELINE = "c8a4c5e"


def _git_show(rev_path):
    return subprocess.run(["git", "-C", str(ROOT), "show", rev_path],
                          capture_output=True, text=True)


def _build(src_dir, out):
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(src_dir),
        str(src_dir / "yaw_heading_control.c"),
        str(ROOT / "tests" / "yaw_heading_golden_driver.c"),
        "-lm", "-o", str(out),
    ], check=True)


def _fmt(v):
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    if math.isnan(v):
        return "nan"
    if math.isinf(v):
        return "inf" if v > 0 else "-inf"
    return repr(float(v))


def _line(dt, yaw, hdg, valid, thr, ff, steer, en, drv, fresh, base, reset):
    return " ".join(_fmt(x) for x in (dt, yaw, hdg, valid, thr, ff, steer, en, drv, fresh, base, reset))


def random_stream(seed=20260925, target_lines=200_000):
    rng = random.Random(seed)
    lines = []
    while len(lines) < target_lines:
        n = rng.randint(50, 400)
        yaw = rng.uniform(-10, 10)
        hdg = rng.uniform(0, 360)
        thr = rng.choice([0.0, 0.1, 0.149, 0.15, 0.2, 0.3, 0.4, 0.45, 0.6, 1.0, rng.uniform(0, 1)])
        ff = rng.choice([0.0, 0.21, -0.3, 0.56, rng.uniform(-1.2, 1.2)])
        en, drv = rng.random() < 0.85, rng.random() < 0.9
        steer = 0.0
        for i in range(n):
            yaw += (-yaw / 20.0) + rng.gauss(0, 1.5)
            hdg = (hdg - yaw * 0.02) % 360.0
            if rng.random() < 0.01:
                hdg = (hdg + rng.choice([-179, -90, 90, 179, 359])) % 360.0
            if rng.random() < 0.02:
                steer = rng.choice([0.0, 0.0, 0.01, 0.019, 0.021, -0.5, 0.5, 1.0, -1.0])
            if rng.random() < 0.01:
                thr = rng.choice([0.0, 0.14, 0.15, 0.4, rng.uniform(0, 1)])
            if rng.random() < 0.005:
                en = not en
            if rng.random() < 0.005:
                drv = not drv
            dt = 0.02 + rng.uniform(-0.005, 0.005)
            r = rng.random()
            if r < 0.01:
                dt = rng.choice([0.0, -0.01, 0.1, 0.12, 0.3, 1.0])
            valid = rng.random() > 0.02
            fresh = rng.random() > 0.01
            base = rng.random() < 0.003
            reset = rng.random() < 0.002
            y, h, t, f, s = yaw, hdg, thr, ff, steer
            bad = rng.random()
            if bad < 0.001:
                y = float("nan")
            elif bad < 0.002:
                h = float("nan")
            elif bad < 0.0025:
                t = float("inf")
            elif bad < 0.003:
                f = float("nan")
            elif bad < 0.0035:
                s = float("nan")
            elif bad < 0.004:
                h = rng.choice([-720.5, 1080.25, -0.0, 360.0])
            lines.append(_line(dt, y, h, valid, t, f, s, en, drv, fresh, base, reset))
    return lines


def recorded_stream():
    """The controller's own inputs from every lake row that recorded them."""
    import csv
    import io
    base = next((p / "dataout" for p in (ROOT, *ROOT.parents) if (p / "dataout").is_dir()), None)
    if base is None:
        return []
    lines = []
    for run in sorted(base.iterdir()):
        f = run / "samples.csv"
        if not (run.is_dir() and f.is_file() and run.name.startswith(("STRAIGHT_", "YAW_", "LAKE_ID_"))):
            continue
        text = "".join(l for l in f.open(newline="") if not l.startswith("#"))
        rows = list(csv.DictReader(io.StringIO(text)))
        if not rows or "yaw_dps" not in rows[0] or "boat_assist_motor_p" not in rows[0]:
            continue
        last_t = None
        for i, r in enumerate(rows):
            try:
                t = float(r["elapsed_s"]); yaw = float(r["yaw_dps"]); hdg = float(r["heading_deg"])
                thr = float(r.get("throttle_set") or 0.0)
                ff = float(r.get("bench_learn_c_last") or 0.21)
                L = float(r.get("cmd_left") or 0.0); R = float(r.get("cmd_right") or 0.0)
            except (TypeError, ValueError):
                continue
            dt = 0.05 if last_t is None else max(0.001, t - last_t)
            last_t = t
            en = str(r.get("boat_assist_motor_p")).lower() in ("1", "true")
            steer = 0.5 * (L - R)
            lines.append(_line(dt, yaw, hdg, True, thr, ff, steer, en, thr > 0.0, True, False, i == 0))
    return lines


@pytest.fixture(scope="module")
def builds():
    old_c = _git_show(f"{BASELINE}:main/yaw_heading_control.c")
    old_h = _git_show(f"{BASELINE}:main/yaw_heading_control.h")
    if old_c.returncode or old_h.returncode:
        pytest.skip(f"{BASELINE} is not in this clone's history")
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / "old").mkdir()
        (td / "old" / "yaw_heading_control.c").write_text(old_c.stdout)
        (td / "old" / "yaw_heading_control.h").write_text(old_h.stdout)
        _build(td / "old", td / "old_bin")
        _build(ROOT / "main", td / "new_bin")
        yield td / "old_bin", td / "new_bin"


def _compare(builds, lines):
    old_bin, new_bin = builds
    feed = "\n".join(lines) + "\n"
    old = subprocess.run([str(old_bin)], input=feed, capture_output=True, text=True, check=True).stdout
    new = subprocess.run([str(new_bin)], input=feed, capture_output=True, text=True, check=True).stdout
    old_l, new_l = old.splitlines(), new.splitlines()
    assert len(old_l) == len(lines), "the driver stopped reading early"
    assert len(new_l) == len(old_l)
    first = next((i for i, (a, b) in enumerate(zip(old_l, new_l)) if a != b), None)
    assert first is None, "input %d: %s\n old %s\n new %s" % (first, lines[first], old_l[first], new_l[first])


def test_the_heading_hold_is_bit_identical_on_random_inputs(builds):
    lines = random_stream()
    assert len(lines) >= 200_000
    _compare(builds, lines)


def test_the_heading_hold_is_bit_identical_on_recorded_lake_inputs(builds):
    lines = recorded_stream()
    if len(lines) < 1000:
        pytest.skip("no recorded lake runs (dataout/) here")
    _compare(builds, lines)


def test_the_golden_test_can_fail(builds):
    """The comparison must notice a one-ulp difference: a harness that always
    matches proves nothing."""
    old_bin, _ = builds
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / "mut").mkdir()
        src = (ROOT / "main" / "yaw_heading_control.c").read_text()
        needle = "out.p_term = cfg->rate_kp * out.rate_error_dps;"
        assert needle in src
        (td / "mut" / "yaw_heading_control.c").write_text(
            src.replace(needle, "out.p_term = cfg->rate_kp * out.rate_error_dps * 1.0000001f;"))
        (td / "mut" / "yaw_heading_control.h").write_text((ROOT / "main" / "yaw_heading_control.h").read_text())
        _build(td / "mut", td / "mut_bin")
        with pytest.raises(AssertionError):
            _compare((old_bin, td / "mut_bin"), random_stream(target_lines=20_000))
