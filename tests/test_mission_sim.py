"""The out-and-back mission simulator (spec section 9.2).

First its boat must match the lake: tools/mission_sim/model_ref.json holds the
numbers tools/mission_sim/model_from_dataout.py extracted from the 2026-09-20/21
runs (the current physical boat), and the simulator's own plant has to
reproduce them -- speeds, speed-up, glide, straight running with Motor P on and
off, and every recorded turn / yaw pulse replayed from its recorded commands.

Then the REAL mission, AUTO owner, heading hold and mixer fly it: every
scenario of the fault catalogue must end as expected, the nominal envelope must
finish truly inside the zone, and nothing may spin or declare a false arrival.
"""
import json
import math
import os
from pathlib import Path
import statistics
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "tools" / "mission_sim"
REF = json.loads((SIM_DIR / "model_ref.json").read_text())
SOURCES = ["mission.c", "nav_geo.c", "course_error.c", "path_follow.c", "planner.c", "auto_drive.c",
           "esc_trim.c", "yaw_heading_control.c"]


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    out = tmp_path_factory.mktemp("mission_sim") / "mission_sim"
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "main"), str(SIM_DIR / "mission_sim.c"),
        *[str(ROOT / "main" / s) for s in SOURCES],
        "-lm", "-o", str(out),
    ], check=True)
    return out


def run(sim, *args):
    return subprocess.run([str(sim), *args], capture_output=True, text=True)


@pytest.fixture(scope="module")
def plant(sim):
    res = run(sim, "--plant", "--runs", "60")
    assert res.returncode == 0, res.stderr
    return [json.loads(line) for line in res.stdout.splitlines() if line.startswith("{")]


def pct(xs, q):
    xs = sorted(xs)
    k = (len(xs) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def col(rows, key):
    return [r[key] for r in rows if r.get(key) is not None and math.isfinite(r[key])]


# ---- the reference is the current boat ------------------------------------

def test_the_reference_is_the_current_boat():
    meta = REF["meta"]
    assert meta["days"] == ["2026-09-20", "2026-09-21"]
    assert len(meta["runs"]) >= 30
    assert len(REF["p_on"]) >= 10 and len(REF["p_off"]) >= 3
    assert len([s for s in REF["segments"] if not s["truncated"]]) >= 12
    assert len(REF["speed"]) >= 15 and len(REF["glide"]) >= 8


# ---- the plant reproduces the lake -----------------------------------------

@pytest.mark.parametrize("throttle,key", [(0.20, "v20"), (0.40, "v40"), (0.45, "v45")])
def test_speed_matches_the_lake(plant, throttle, key):
    rec = statistics.median(s["v"] for s in REF["speed"] if abs(s["throttle"] - throttle) < 1e-6)
    model = col(plant, key)
    assert abs(statistics.median(model) - rec) <= 0.15 * rec, (rec, statistics.median(model))
    assert pct(model, 5) <= rec <= pct(model, 95), (rec, pct(model, 5), pct(model, 95))


def test_speed_up_matches_the_lake(plant):
    rec = statistics.median(s["t90"] for s in REF["speed"]
                            if abs(s["throttle"] - 0.40) < 1e-6 and math.isfinite(s["t90"]))
    model = col(plant, "t90_40")
    assert pct(model, 5) - 1.0 <= rec <= pct(model, 95) + 1.0, (rec, pct(model, 5), pct(model, 95))


def test_glide_matches_the_lake(plant):
    halves = [g["half_life_s"] for g in REF["glide"] if g["half_life_s"] is not None]
    rec_half = statistics.median(halves)
    model_half = col(plant, "half_40")
    assert pct(model_half, 5) - 0.5 <= rec_half <= pct(model_half, 95) + 0.5, (rec_half, model_half)
    # distance per unit speed at the cut: the GPS reads the cut speed late,
    # so compare the ratio, not the distance
    rec_ratio = statistics.median(g["dist_3s"] / g["v0"] for g in REF["glide"])
    model_ratio = statistics.median(p["glide3_40"] / p["v40"] for p in plant)
    assert abs(model_ratio - rec_ratio) <= 0.35 * rec_ratio, (rec_ratio, model_ratio)


def test_straight_with_motor_p_on_matches_the_lake(plant):
    rec_yaw = statistics.median(p["yaw_std"] for p in REF["p_on"])
    rec_c = statistics.median(p["c_std"] for p in REF["p_on"])
    for key_yaw, key_c in (("pon40_yaw_std", "pon40_c_std"), ("pon20_yaw_std", "pon20_c_std")):
        assert pct(col(plant, key_yaw), 10) <= rec_yaw <= pct(col(plant, key_yaw), 90), (key_yaw, rec_yaw)
        assert pct(col(plant, key_c), 10) <= rec_c <= pct(col(plant, key_c), 90), (key_c, rec_c)
    # the heading held within a couple of degrees, on the lake and in the model
    assert pct([abs(p["hdg_change"]) for p in REF["p_on"]], 90) <= 2.5
    assert pct([abs(x) for x in col(plant, "pon40_hdg")], 90) <= 2.5
    # the straight-running c the P found: the model covers the lake's spread
    rec_c_mean = [p["c_mean"] for p in REF["p_on"]]
    model_c_mean = col(plant, "pon40_c_mean")
    assert pct(model_c_mean, 5) <= statistics.median(rec_c_mean) <= pct(model_c_mean, 95)


def test_straight_with_motor_p_off_wanders_like_the_lake(plant):
    rec = statistics.median(p["yaw_std"] for p in REF["p_off"])
    model = col(plant, "poff40_yaw_std")
    assert pct(model, 10) <= rec <= pct(model, 90), (rec, pct(model, 10), pct(model, 90))


def test_recorded_turns_and_pulses_replay(sim, tmp_path):
    """Every recorded turn / yaw pulse, driven by its own recorded commands
    (re-timed by the command age), must come out of the model at the recorded
    yaw rate: mean over the last second within max(3 deg/s, 25 %)."""
    segs = [s for s in REF["segments"] if not s["truncated"]]
    lines = []
    for i, s in enumerate(segs):
        lines.append("S,%d,%.4f,%.3f" % (i, s["c_pre"], s["yaw_pre"]))
        rows = sorted(zip((t - a for t, a in zip(s["t"], s["age"])), s["left"], s["right"]), key=lambda r: r[0])
        lines += ["P,%.3f,%.4f,%.4f" % (max(t, 0.0), l, r) for t, l, r in rows]
    path = tmp_path / "replay.csv"
    path.write_text("\n".join(lines) + "\n")
    res = run(sim, "--replay", str(path), "--runs", "40")
    assert res.returncode == 0, res.stderr
    model = {}
    for line in res.stdout.splitlines():
        i, b, t, y = line.split(",")
        model.setdefault(int(i), {}).setdefault(int(b), []).append((float(t), float(y)))
    good, rel, report = 0, [], []
    for i, s in enumerate(segs):
        end = s["t"][-1]
        rec = statistics.fmean(y for t, y in zip(s["t"], s["yaw"]) if t > end - 1.0)
        per_boat = []
        for series in model[i].values():
            t_end = max(t for t, _ in series)
            per_boat.append(statistics.fmean(y for t, y in series if t > t_end - 1.0))
        med = statistics.median(per_boat)
        ok = abs(rec - med) <= max(3.0, 0.25 * abs(rec))
        good += ok
        rel.append((med - rec) / max(abs(rec), 3.0))
        report.append("%s %s recorded %+.1f model %+.1f %s" % (s["run"], s["phase"], rec, med, "ok" if ok else "MISS"))
    assert good >= 0.8 * len(segs), "\n".join(report)
    assert abs(statistics.median(rel)) <= 0.15, "\n".join(report)


# ---- the mission on that boat -----------------------------------------------

def test_every_catalogue_scenario_ends_as_expected(sim):
    res = run(sim, "--catalogue", "--runs", "40")
    assert res.returncode == 0 and "CATALOGUE PASSED" in res.stdout, res.stdout + res.stderr


def test_the_catalogue_covers_the_faults(sim):
    names = run(sim, "--list").stdout.split()
    assert len(names) == len(set(names)) >= 60
    for must in ("nominal", "radio_gone", "stop_mid", "manual_nudge", "disarm_mid", "rail_cut",
                 "calibration_start", "gps_dropout_3s", "gps_frozen", "gps_2d", "imu_frozen",
                 "autonomy_stall", "jet_dies", "current_faster", "left_turn", "dry_run",
                 "out_only", "out_turn", "compass_70", "old_compass", "windy_balance",
                 "home_gps_jumpy", "restart_after_stop", "start_spam"):
        assert must in names, must


def test_nominal_missions_finish_truly_inside_the_zone(sim):
    res = run(sim, "--runs", "300")
    assert res.returncode == 0, res.stderr
    s = json.loads(res.stdout.strip().splitlines()[-1])
    assert s["in_zone"] >= 0.95, s
    assert s["spins"] == 0 and s["false_arrivals"] == 0 and s["aborted"] == 0, s


def test_where_drift_starts_to_fail_is_known_and_safe(sim):
    res = run(sim, "--sweep", "drift", "--runs", "60")
    assert res.returncode == 0, res.stderr
    rows = {}
    for line in res.stdout.splitlines():
        parts = line.split()
        if parts and parts[0].endswith("m/s"):
            v = float(parts[0][:-3])
            rows[v] = {"in_zone": float(parts[1].rstrip("%")), "spins": int(parts[6]), "false": int(parts[7])}
    assert rows[0.10]["in_zone"] >= 95.0, res.stdout
    assert rows[0.15]["in_zone"] >= 90.0, res.stdout
    for v, r in rows.items():            # beyond the envelope it may not arrive -- but never spins
        assert r["spins"] == 0 and r["false"] == 0, (v, res.stdout)
