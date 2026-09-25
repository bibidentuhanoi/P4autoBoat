#!/usr/bin/env python3
"""Extract the mission simulator's reference numbers from the lake runs.

Reads dataout/<RUN>/summary.json + samples.csv (recorded by tools/espnow_drive.py),
keeps only the runs of the chosen days -- by default 2026-09-20 and 2026-09-21,
the current physical boat -- and writes tools/mission_sim/model_ref.json:

  speed      steady GPS speed of each straight run (median of its last 60 %)
             and the time to 90 % of it
  glide      after motors off: speed at the cut, distance in 3 s, half-life
  p_on       straight with Motor P ON: gyro yaw std, applied-c std and mean,
             heading change, GPS course minus heading
  p_off      straight with Motor P OFF: gyro yaw std, heading change
  segments   every turn / yaw pulse: the P-held straight before it (c_pre,
             yaw_pre) and its rows (t, left, right, yaw, command age)
  gps_still  the still boat before each run: GPS spread and speed

tests/test_mission_sim.py checks the simulator's boat model against this file.
Re-run it when new lake data should change the model:

    .venv/bin/python tools/mission_sim/model_from_dataout.py
    .venv/bin/python tools/mission_sim/model_from_dataout.py --dataout ../../dataout --days 2026-09-20,2026-09-21
"""
import argparse
import csv
import datetime
import io
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = Path(__file__).resolve().parent / "model_ref.json"
DEFAULT_DAYS = "2026-09-20,2026-09-21"
FAMILIES = ("STRAIGHT_", "YAW_", "LAKE_ID_")
MIN_STRAIGHT_S = 15.0      # shorter straights were cut by a failsafe or STOP
SETTLE_S = 5.0             # speed-up at the start of a straight
MAX_GPS_SPEED = 2.0        # the boat cannot do this: a GPS glitch
MAX_FIX_STEP_M = 5.0       # consecutive fixes this far apart: a GPS glitch


def default_dataout():
    """dataout/ lives in the main checkout; a worktree sits two levels below it."""
    for base in (ROOT, *ROOT.parents):
        if (base / "dataout").is_dir():
            return base / "dataout"
    return ROOT / "dataout"


def num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return float("nan")
    return v


def truthy(x):
    return str(x).strip().lower() in ("1", "true", "yes")


def load_rows(run_dir):
    with open(run_dir / "samples.csv", newline="") as f:
        text = "".join(line for line in f if not line.startswith("#"))
    return list(csv.DictReader(io.StringIO(text)))


def local_day(t_utc, offset_h):
    try:
        t = datetime.datetime.fromisoformat(str(t_utc).replace("Z", "+00:00"))
    except ValueError:
        return None
    return (t + datetime.timedelta(hours=offset_h)).strftime("%Y-%m-%d")


def circ_mean_std(deg):
    if not deg:
        return float("nan"), float("nan")
    s = sum(math.sin(math.radians(d)) for d in deg) / len(deg)
    c = sum(math.cos(math.radians(d)) for d in deg) / len(deg)
    r = min(1.0, math.hypot(s, c))
    mean = math.degrees(math.atan2(s, c))
    std = math.degrees(math.sqrt(-2.0 * math.log(r))) if r > 1e-9 else 180.0
    return mean, std


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def local_en(lat, lon, lat0, lon0):
    return ((lon - lon0) * 111320.0 * math.cos(math.radians(lat0)), (lat - lat0) * 110540.0)


def fixes(rows):
    """Rows that carry a new GPS fix, with glitches removed."""
    out = []
    for r in rows:
        if not truthy(r.get("gps_values_changed")):
            continue
        sp, lat, lon = num(r.get("speed_mps")), num(r.get("lat")), num(r.get("lon"))
        if not (math.isfinite(sp) and math.isfinite(lat) and math.isfinite(lon)):
            continue
        if sp > MAX_GPS_SPEED or lat == 0.0:
            continue
        out.append(r)
    return out


def glitchy(fx):
    if len(fx) < 2:
        return False
    lat0, lon0 = num(fx[0]["lat"]), num(fx[0]["lon"])
    prev = None
    for r in fx:
        e, n = local_en(num(r["lat"]), num(r["lon"]), lat0, lon0)
        if prev is not None and math.hypot(e - prev[0], n - prev[1]) > MAX_FIX_STEP_M:
            return True
        prev = (e, n)
    return False


def c_of(r):
    L, R = num(r.get("boat_applied_left_cmd")), num(r.get("boat_applied_right_cmd"))
    if not (math.isfinite(L) and math.isfinite(R)) or L + R <= 0.05:
        return float("nan")
    return (R - L) / (R + L)


def std(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return statistics.pstdev(xs) if len(xs) >= 2 else float("nan")


def mean(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return statistics.fmean(xs) if xs else float("nan")


def straight_metrics(name, summary, rows, out, excluded):
    st = [r for r in rows if r.get("phase") == "straight"]
    if not st:
        return
    dur = num(st[-1].get("phase_elapsed_s"))
    if not math.isfinite(dur) or dur < MIN_STRAIGHT_S:
        excluded.append({"run": name, "part": "straight", "why": "shorter than %.0f s" % MIN_STRAIGHT_S})
        return
    T = float(summary["settings"]["throttle"])
    body = [r for r in st if num(r.get("phase_elapsed_s")) >= SETTLE_S]
    p_frac = sum(truthy(r.get("boat_assist_motor_p")) for r in body) / max(1, len(body))
    yaw = [num(r.get("yaw_dps")) for r in body]
    head = [num(r.get("heading_deg")) for r in body]
    t = [num(r.get("phase_elapsed_s")) for r in body]
    first = [h for h, tt in zip(head, t) if tt < SETTLE_S + 2.0 and math.isfinite(h)]
    last = [h for h, tt in zip(head, t) if tt > t[-1] - 2.0 and math.isfinite(h)]
    hdg_change = wrap180(circ_mean_std(last)[0] - circ_mean_std(first)[0]) if first and last else float("nan")
    fx = fixes(st)
    gps_ok = len(fx) >= 20 and not glitchy(fx)
    if not gps_ok:
        excluded.append({"run": name, "part": "straight GPS", "why": "too few fixes or a position glitch"})
    if gps_ok:
        ft = [num(r["phase_elapsed_s"]) for r in fx]
        fv = [num(r["speed_mps"]) for r in fx]
        steady = statistics.median([v for v, tt in zip(fv, ft) if tt >= 0.4 * ft[-1]])
        t90 = float("nan")
        for i in range(len(fv)):
            window = fv[max(0, i - 2):i + 3]
            if statistics.fmean(window) >= 0.9 * steady:
                t90 = ft[i]
                break
        out["speed"].append({"run": name, "throttle": T, "p_on": p_frac > 0.5, "v": round(steady, 4),
                             "t90": round(t90, 2), "duration_s": round(dur, 1)})
    if p_frac >= 0.9:
        cs = [c_of(r) for r in body]
        beta = [wrap180(num(r["course_deg"]) - num(r["heading_deg"])) for r in fx
                if num(r["speed_mps"]) >= 0.2 and num(r.get("phase_elapsed_s")) >= SETTLE_S] if gps_ok else []
        bm, bs = circ_mean_std(beta)
        out["p_on"].append({"run": name, "throttle": T, "yaw_std": round(std(yaw), 3),
                            "c_std": round(std(cs), 4), "c_mean": round(mean(cs), 4),
                            "hdg_change": round(hdg_change, 2), "heading": round(circ_mean_std(head)[0] % 360.0, 1),
                            "beta_mean": round(bm, 1) if beta else None, "beta_std": round(bs, 1) if beta else None,
                            "duration_s": round(dur, 1)})
    elif p_frac <= 0.1:
        out["p_off"].append({"run": name, "throttle": T, "yaw_std": round(std(yaw), 3),
                             "hdg_change": round(hdg_change, 2), "c_mean": round(mean(c_of(r) for r in body), 4),
                             "duration_s": round(dur, 1)})


def glide_metrics(name, rows, out):
    stop = [r for r in rows if r.get("phase") == "stop"]
    if len(stop) < 20:
        return
    i0 = rows.index(stop[0])
    if i0 == 0:
        return
    before = rows[i0 - 1].get("phase")
    fx = fixes(stop)
    if len(fx) < 20 or glitchy(fx):
        return
    t = [num(r["phase_elapsed_s"]) for r in fx]
    v = [num(r["speed_mps"]) for r in fx]
    early = [vv for vv, tt in zip(v, t) if tt < 0.4]
    if not early:
        return
    v0 = statistics.median(early)
    if v0 < 0.15:
        return                          # it was barely moving: no glide to measure
    lat0, lon0 = num(fx[0]["lat"]), num(fx[0]["lon"])
    k3 = min(range(len(t)), key=lambda k: abs(t[k] - 3.0))
    e, n = local_en(num(fx[k3]["lat"]), num(fx[k3]["lon"]), lat0, lon0)
    half = None
    for i in range(1, len(v)):
        if statistics.fmean(v[max(0, i - 1):i + 2]) <= 0.5 * v0:
            half = t[i]
            break
    out["glide"].append({"run": name, "after": before, "v0": round(v0, 4), "dist_3s": round(math.hypot(e, n), 3),
                         "half_life_s": round(half, 2) if half is not None else None})


def segment_metrics(name, summary, rows, out, excluded):
    planned = {p[0]: float(p[1]) for p in summary.get("settings", {}).get("phases", [])}
    for phase in ("turn_a", "turn_b"):
        idx = [i for i, r in enumerate(rows) if r.get("phase") == phase]
        if len(idx) < 10:
            continue
        i0 = idx[0]
        pre = [r for r in rows[max(0, i0 - 40):i0]
               if r.get("phase") in ("straight", "recover_a") and truthy(r.get("boat_assist_motor_p"))]
        if len(pre) < 20:
            excluded.append({"run": name, "part": phase, "why": "no P-held straight before it"})
            continue
        c_pre = mean(c_of(r) for r in pre)
        yaw_pre = mean(num(r.get("yaw_dps")) for r in pre)
        if not math.isfinite(c_pre) or c_pre <= -0.9:
            excluded.append({"run": name, "part": phase, "why": "the boat was not held straight before it (c %.2f)" % c_pre})
            continue
        t0 = num(rows[i0]["elapsed_s"])
        seg = {"run": name, "phase": phase, "throttle": float(summary["settings"]["throttle"]),
               "half_difference": summary.get(phase, {}).get("motor_half_difference"),
               "c_pre": round(c_pre, 4), "yaw_pre": round(yaw_pre, 3), "t": [], "left": [], "right": [], "yaw": [], "age": []}
        for i in idx:
            r = rows[i]
            vals = [num(r.get("elapsed_s")) - t0, num(r.get("boat_applied_left_cmd")),
                    num(r.get("boat_applied_right_cmd")), num(r.get("yaw_dps")), num(r.get("motor_status_age_s"))]
            if not all(math.isfinite(x) for x in vals[:4]):
                continue
            seg["t"].append(round(vals[0], 3)); seg["left"].append(round(vals[1], 4))
            seg["right"].append(round(vals[2], 4)); seg["yaw"].append(round(vals[3], 3))
            seg["age"].append(round(vals[4], 3) if math.isfinite(vals[4]) else 0.05)
        span = seg["t"][-1] if seg["t"] else 0.0
        seg["truncated"] = bool(summary.get("abort_phase") == phase or span < 0.9 * planned.get(phase, 0.0))
        out["segments"].append(seg)


def still_metrics(name, rows, out):
    pre = [r for r in rows if r.get("phase") == "precheck"]
    fx = fixes(pre)
    if len(fx) < 8:
        return
    lat0, lon0 = num(fx[0]["lat"]), num(fx[0]["lon"])
    pts = [local_en(num(r["lat"]), num(r["lon"]), lat0, lon0) for r in fx]
    me = statistics.fmean(p[0] for p in pts)
    mn = statistics.fmean(p[1] for p in pts)
    spread = max(math.hypot(p[0] - me, p[1] - mn) for p in pts)
    out["gps_still"].append({"run": name, "spread_m": round(spread, 3),
                             "speed": round(statistics.median(num(r["speed_mps"]) for r in fx), 3),
                             "sats": int(statistics.median(num(r.get("satellites")) for r in fx
                                                           if math.isfinite(num(r.get("satellites"))))
                                         if any(math.isfinite(num(r.get("satellites"))) for r in fx) else 0)})


def extract(dataout, days, offset_h):
    out = {"meta": {"days": days, "utc_offset_h": offset_h, "source": "dataout/<run>/{summary.json,samples.csv}",
                    "generated_by": "tools/mission_sim/model_from_dataout.py", "runs": [], "excluded": []},
           "speed": [], "glide": [], "p_on": [], "p_off": [], "segments": [], "gps_still": []}
    for run_dir in sorted(p for p in Path(dataout).iterdir() if p.is_dir() and p.name.startswith(FAMILIES)):
        try:
            summary = json.loads((run_dir / "summary.json").read_text())
        except (OSError, ValueError):
            continue
        if local_day(summary.get("t_utc_end"), offset_h) not in days:
            continue
        if not summary.get("sample_count"):
            out["meta"]["excluded"].append({"run": run_dir.name, "part": "run", "why": "no samples"})
            continue
        rows = load_rows(run_dir)
        out["meta"]["runs"].append(run_dir.name)
        straight_metrics(run_dir.name, summary, rows, out, out["meta"]["excluded"])
        glide_metrics(run_dir.name, rows, out)
        segment_metrics(run_dir.name, summary, rows, out, out["meta"]["excluded"])
        still_metrics(run_dir.name, rows, out)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dataout", type=Path, default=default_dataout())
    ap.add_argument("--days", default=DEFAULT_DAYS, help="comma-separated local dates (YYYY-MM-DD)")
    ap.add_argument("--utc-offset-h", type=float, default=7.0, help="local time = UTC + this (Vietnam: 7)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    if not args.dataout.is_dir():
        raise SystemExit("no dataout folder at %s" % args.dataout)
    days = [d.strip() for d in args.days.split(",") if d.strip()]
    ref = extract(args.dataout, days, args.utc_offset_h)
    if not ref["meta"]["runs"]:
        raise SystemExit("no runs from %s in %s" % (", ".join(days), args.dataout))
    args.out.write_text(json.dumps(ref, indent=1, sort_keys=True) + "\n")
    print("%d runs -> %s: %d speeds, %d glides, %d P-ON straights, %d P-OFF straights, %d turn segments, "
          "%d still checks, %d exclusions" % (len(ref["meta"]["runs"]), args.out, len(ref["speed"]), len(ref["glide"]),
                                              len(ref["p_on"]), len(ref["p_off"]), len(ref["segments"]),
                                              len(ref["gps_still"]), len(ref["meta"]["excluded"])))


if __name__ == "__main__":
    main()
