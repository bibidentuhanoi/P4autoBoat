#!/usr/bin/env python3
"""Plot one GPS path and a timed GPS table for every dataout run on a UTC day.

The plots show recorded fixes without smoothing.  A run without GPS still gets
an image so that missing position data cannot be mistaken for a stationary boat.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PHASE_COLORS = {
    "precheck": "#8994a3",
    "straight": "#2474b5",
    "turn_a": "#e87928",
    "recover_a": "#218c70",
    "turn_b": "#b34b9b",
    "recover_b": "#27a5a5",
    "stop": "#515b68",
    "base": "#2474b5",
}


def number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(line for line in handle if not line.startswith("#")))


def gps_fix(row: dict[str, str]) -> tuple[float, float] | None:
    if row.get("gps_valid") != "1":
        return None
    lat, lon = number(row.get("lat")), number(row.get("lon"))
    if lat is None or lon is None or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return None
    if lat == 0 and lon == 0:
        return None
    return lat, lon


def local_xy(fix: tuple[float, float], origin: tuple[float, float]) -> tuple[float, float]:
    lat, lon = fix
    lat0, lon0 = origin
    radius = 6_371_000.0
    north = math.radians(lat - lat0) * radius
    east = math.radians(lon - lon0) * radius * math.cos(math.radians(lat0))
    return east, north


def phase_label(phase: str, summary: dict) -> str:
    if phase in ("turn_a", "turn_b"):
        side = (summary.get(phase) or {}).get("side")
        if side:
            return f"{phase} {side.lower()}"
    return phase.replace("_", " ")


def first_phase_indices(rows: list[dict[str, str]]) -> list[int]:
    return [i for i, row in enumerate(rows) if i == 0 or row.get("phase") != rows[i - 1].get("phase")]


def table_indices(rows: list[dict[str, str]]) -> list[int]:
    if not rows:
        return []
    chosen = set(first_phase_indices(rows)) | {len(rows) - 1}
    elapsed = [(i, number(row.get("elapsed_s"))) for i, row in enumerate(rows)]
    elapsed = [(i, t) for i, t in elapsed if t is not None]
    if elapsed:
        for target in range(0, int(max(t for _, t in elapsed)) + 1, 5):
            chosen.add(min(elapsed, key=lambda pair: abs(pair[1] - target))[0])
    return sorted(chosen)


def utc_time(row: dict[str, str]) -> str:
    value = row.get("t_utc", "")
    if "T" in value:
        return value.split("T", 1)[1][:12]
    return "—"


def plot_run(name: str, rows: list[dict[str, str]], summary: dict, output: Path,
             source: Path) -> dict:
    fixes = [(i, fix) for i, row in enumerate(rows) if (fix := gps_fix(row))]
    elapsed = [number(row.get("elapsed_s")) for row in rows]
    valid_times = [t for t in elapsed if t is not None]
    duration = max(valid_times) - min(valid_times) if valid_times else 0.0
    hdops = [number(rows[i].get("hdop")) for i, _ in fixes]
    hdops = [value for value in hdops if value is not None]
    sats = [number(rows[i].get("satellites")) for i, _ in fixes]
    sats = [value for value in sats if value is not None]
    speeds = [number(rows[i].get("speed_mps")) for i, _ in fixes]
    speeds = [value for value in speeds if value is not None]
    hdop_median = float(np.median(hdops)) if hdops else None
    origin = fixes[0][1] if fixes else None
    xy = {i: local_xy(fix, origin) for i, fix in fixes} if origin else {}
    positions = list(xy.values())
    net_m = math.dist(positions[0], positions[-1]) if len(positions) >= 2 else None

    fig = plt.figure(figsize=(17.5, 10.5), dpi=150, facecolor="#fbfcff")
    grid = fig.add_gridspec(2, 2, width_ratios=(1.03, 1.32),
                           height_ratios=(3.4, 1.05), left=0.055, right=0.985,
                           top=0.855, bottom=0.075, wspace=0.23, hspace=0.28)
    ax_path = fig.add_subplot(grid[0, 0])
    ax_time = fig.add_subplot(grid[1, 0])
    ax_table = fig.add_subplot(grid[:, 1])
    ax_table.axis("off")

    status = summary.get("status", "standalone BASE")
    end_utc = summary.get("t_utc_end", "")
    if not end_utc and rows:
        end_utc = rows[-1].get("t_utc", "")
    title = f"{name}  |  {status.upper()}"
    fig.suptitle(title, x=0.055, y=0.985, ha="left", fontsize=20, fontweight="bold")
    origin_note = "local path origin = first valid GPS fix" if fixes else "GPS path unavailable"
    fig.text(0.055, 0.948,
             f"UTC day {end_utc[:10] or 'unknown'}  •  raw WGS84 fixes  •  "
             f"{origin_note}  •  elapsed {duration:.1f} s",
             fontsize=10.5, color="#48586a", ha="left")
    quality = ("NO GPS FIX" if not fixes else
               "LOW GPS CONFIDENCE" if hdop_median is not None and hdop_median >= 3 else
               "GPS CAUTION" if hdop_median is not None and hdop_median >= 2 else
               "GPS FIX AVAILABLE")
    net_text = "—" if net_m is None else f"{net_m:.2f} m"
    gps_meta = (f"{quality}   |   {len(fixes)}/{len(rows)} rows with position   |   "
                f"median HDOP {hdop_median:.2f}   |   "
                f"mean GPS speed {np.mean(speeds):.2f} m/s   |   "
                f"start→end {net_text}") if fixes and speeds and hdop_median is not None else (
                f"{quality}   |   {len(fixes)}/{len(rows)} rows with position   |   "
                f"start→end {net_text}")
    fig.text(0.055, 0.917, gps_meta, fontsize=10.5,
             color="#a14a22" if quality != "GPS FIX AVAILABLE" else "#28664d", ha="left")
    reason = summary.get("reason")
    if reason:
        fig.text(0.055, 0.890, f"Run note: {str(reason)[:130]}", fontsize=9.3,
                 color="#8a3340", ha="left")

    ax_path.set_title("GPS path · raw positions", loc="left", fontsize=13, pad=10)
    ax_path.set_xlabel("East from first fix (m)")
    ax_path.set_ylabel("North from first fix (m)")
    ax_path.grid(alpha=0.27)
    ax_path.set_aspect("equal", adjustable="box")
    if fixes:
        # Connect only consecutive valid positions within a phase and without a
        # large telemetry gap; otherwise a missing fix looks like a real track.
        used_phases = set()
        for (i, _), (j, _) in zip(fixes, fixes[1:]):
            row_i, row_j = rows[i], rows[j]
            phase = row_j.get("phase", "base")
            ti, tj = elapsed[i], elapsed[j]
            if (phase != row_i.get("phase", "base") or ti is None or tj is None
                    or tj - ti > 0.5):
                continue
            x0, y0 = xy[i]
            x1, y1 = xy[j]
            ax_path.plot((x0, x1), (y0, y1), color=PHASE_COLORS.get(phase, "#627487"),
                         lw=2.0, alpha=0.85, label=phase_label(phase, summary)
                         if phase not in used_phases else None)
            used_phases.add(phase)
        start_i, end_i = fixes[0][0], fixes[-1][0]
        ax_path.scatter(*xy[start_i], s=115, marker="o", color="#21a560",
                        edgecolors="white", linewidths=1.2, zorder=6, label="first GPS fix")
        ax_path.scatter(*xy[end_i], s=145, marker="X", color="#192a3d",
                        edgecolors="white", linewidths=0.8, zorder=6, label="last GPS fix")
        for i in first_phase_indices(rows):
            phase = rows[i].get("phase", "base")
            next_fix = next((j for j, _ in fixes if j >= i and rows[j].get("phase", "base") == phase), None)
            if next_fix is None or next_fix == start_i:
                continue
            x, y = xy[next_fix]
            ax_path.scatter(x, y, s=47, marker="D", color=PHASE_COLORS.get(phase, "#627487"),
                            edgecolors="white", linewidths=0.7, zorder=7)
            ax_path.annotate(f"{phase_label(phase, summary)}\n{elapsed[next_fix]:.0f}s",
                             (x, y), xytext=(6, 7), textcoords="offset points",
                             fontsize=7.5, color="#203047",
                             bbox={"boxstyle": "round,pad=0.16", "fc": "white", "ec": "none", "alpha": 0.78})
        xs, ys = zip(*positions)
        x_mid, y_mid = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        span = max(max(xs) - min(xs), max(ys) - min(ys), 1.5) * 1.24
        ax_path.set_xlim(x_mid - span / 2, x_mid + span / 2)
        ax_path.set_ylim(y_mid - span / 2, y_mid + span / 2)
        ax_path.legend(loc="best", fontsize=7.6, framealpha=0.9, ncol=2)
    else:
        ax_path.text(0.5, 0.54, "NO GPS PATH AVAILABLE", transform=ax_path.transAxes,
                     ha="center", va="center", fontsize=16, fontweight="bold", color="#a14a22")
        ax_path.text(0.5, 0.43, "No valid latitude/longitude was recorded for this run.",
                     transform=ax_path.transAxes, ha="center", va="center", fontsize=9.5)
        ax_path.set_xlim(-1, 1)
        ax_path.set_ylim(-1, 1)

    ax_time.set_title("Heading change and GPS speed over time", loc="left", fontsize=11)
    heading = [(i, elapsed[i], number(row.get("heading_deg"))) for i, row in enumerate(rows)]
    heading = [(i, t, h) for i, t, h in heading if t is not None and h is not None]
    if heading:
        t = np.array([v[1] for v in heading])
        h = np.rad2deg(np.unwrap(np.deg2rad([v[2] for v in heading])))
        ax_time.plot(t, h - h[0], color="#33465d", lw=1.55, label="heading Δ")
    else:
        ax_time.text(0.5, 0.5, "No heading samples", transform=ax_time.transAxes,
                     ha="center", va="center", color="#69788a")
    ax_time.axhline(0, color="#8994a3", lw=0.7)
    ax_time.set_xlabel("Elapsed time (s)")
    ax_time.set_ylabel("Heading Δ (degrees)")
    ax_time.grid(alpha=0.22)
    gps_speed = [(elapsed[i], number(row.get("speed_mps"))) for i, row in enumerate(rows)
                 if gps_fix(row) is not None and elapsed[i] is not None]
    gps_speed = [(t, v) for t, v in gps_speed if v is not None]
    if gps_speed:
        ax_speed = ax_time.twinx()
        ax_speed.plot([t for t, _ in gps_speed], [v for _, v in gps_speed],
                      color="#31a07c", lw=1.0, alpha=0.73, label="GPS speed")
        ax_speed.set_ylabel("GPS speed (m/s)", color="#238363")
        ax_speed.tick_params(axis="y", labelcolor="#238363")

    ax_table.set_title("Timed GPS fixes · phase starts, each 5 s, and final sample",
                       loc="left", fontsize=12, pad=10)
    selected = table_indices(rows)
    table_rows = []
    for i in selected:
        row = rows[i]
        fix = gps_fix(row)
        speed = number(row.get("speed_mps")) if fix else None
        hdop = number(row.get("hdop")) if fix else None
        sat = number(row.get("satellites")) if fix else None
        t = elapsed[i]
        table_rows.append([
            "—" if t is None else f"{t:.1f}", utc_time(row),
            phase_label(row.get("phase", "base"), summary),
            "—" if fix is None else f"{fix[0]:.7f}",
            "—" if fix is None else f"{fix[1]:.7f}",
            "—" if speed is None else f"{speed:.2f}",
            "—" if hdop is None else f"{hdop:.2f}",
            "—" if sat is None else f"{sat:.0f}",
        ])
    if not table_rows:
        table_rows = [["—", "—", "no samples", "—", "—", "—", "—", "—"]]
    columns = ["t+ s", "UTC hh:mm:ss", "Phase", "Lat °", "Lon °", "Speed", "HDOP", "Sats"]
    table_height = min(0.82, 0.065 * (len(table_rows) + 1))
    table = ax_table.table(cellText=table_rows, colLabels=columns,
                           colWidths=[0.07, 0.13, 0.16, 0.16, 0.16, 0.10, 0.10, 0.07],
                           cellLoc="center", loc="upper center",
                           bbox=[0, 0.94 - table_height, 1, table_height])
    table.auto_set_font_size(False)
    table.set_fontsize(7.7 if len(table_rows) <= 19 else 7.1)
    for (ri, ci), cell in table.get_celld().items():
        cell.set_edgecolor("#dbe2ea")
        cell.set_linewidth(0.48)
        if ri == 0:
            cell.set_facecolor("#263e58")
            cell.get_text().set_color("white")
            cell.get_text().set_weight("bold")
        elif ri % 2 == 0:
            cell.set_facecolor("#f0f4f8")
        else:
            cell.set_facecolor("white")
    source_label = f"{source.parent.name}/{source.name}"
    ax_table.text(0, 0.095, f"Source: {source_label}", transform=ax_table.transAxes,
                  ha="left", fontsize=8.2, color="#526377")
    ax_table.text(0, 0.062,
                  "Coordinates are raw. GPS jitter, drift and low speed make short distances and turn radius uncertain.",
                  transform=ax_table.transAxes, ha="left", fontsize=8.2, color="#526377")
    ax_table.text(0, 0.029,
                  "A missing fix is shown as —. The CSV contains every recorded sample; this table shows timed checkpoints.",
                  transform=ax_table.transAxes, ha="left", fontsize=8.2, color="#526377")
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    return {"name": name, "file": output.name, "status": status, "end_utc": end_utc,
            "samples": len(rows), "gps_fixes": len(fixes), "median_hdop": hdop_median,
            "net_displacement_m": net_m, "source": str(source)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--date", default=datetime.now(timezone.utc).date().isoformat(),
                        help="UTC date YYYY-MM-DD")
    parser.add_argument("--dataout", type=Path,
                        default=Path(__file__).resolve().parents[3] / "dataout")
    parser.add_argument("--output", type=Path, help="directory for PNG images and index")
    args = parser.parse_args()
    datetime.strptime(args.date, "%Y-%m-%d")
    dataout = args.dataout.resolve()
    output = (args.output or dataout / f"path_plots_{args.date}").resolve()
    output.mkdir(parents=True, exist_ok=True)

    runs: list[tuple[str, Path, Path | None, dict]] = []
    for summary_path in dataout.glob("*/summary.json"):
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not str(summary.get("t_utc_end", "")).startswith(args.date):
            continue
        sample_path = summary_path.parent / "samples.csv"
        if sample_path.exists():
            runs.append((summary_path.parent.name, sample_path, summary_path, summary))
    for base_path in dataout.glob("BASE_*.csv"):
        modified_day = datetime.fromtimestamp(base_path.stat().st_mtime, timezone.utc).date().isoformat()
        if modified_day == args.date:
            recorded_at = datetime.fromtimestamp(base_path.stat().st_mtime, timezone.utc).isoformat()
            runs.append((base_path.stem, base_path, None,
                         {"status": "standalone BASE", "t_utc_end": recorded_at}))
    runs.sort(key=lambda item: (item[3].get("t_utc_end", ""), item[0]))

    manifest = []
    for name, sample_path, _, summary in runs:
        rows = csv_rows(sample_path)
        if sample_path.name.startswith("BASE_"):
            for row in rows:
                row.setdefault("phase", "base")
        target = output / f"{name}.png"
        manifest.append(plot_run(name, rows, summary, target, sample_path))
        print(f"{target.name}: {manifest[-1]['gps_fixes']}/{manifest[-1]['samples']} GPS rows")

    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    cards = "\n".join(
        f'<article><a href="{html.escape(item["file"])}"><img loading="lazy" '
        f'src="{html.escape(item["file"])}" alt="{html.escape(item["name"])} GPS path"></a>'
        f'<div><a href="{html.escape(item["file"])}">{html.escape(item["name"])}'
        f'</a> · {html.escape(str(item["status"]))} · '
        f'{item["gps_fixes"]}/{item["samples"]} GPS rows</div></article>'
        for item in manifest
    )
    index = ("<!doctype html><html lang=\"en\"><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             f"<title>GPS paths {html.escape(args.date)}</title>"
             "<style>body{font:16px system-ui,sans-serif;background:#edf2f7;color:#203047;"
             "margin:2rem}h1{margin-bottom:.2rem}p{color:#536275}main{display:grid;"
             "grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:1rem}article{"
             "background:white;padding:.7rem;border-radius:10px;box-shadow:0 2px 9px #263e581c}"
             "img{width:100%;height:auto;display:block}article div{padding:.5rem .3rem}"
             "a{color:#145988}</style>"
             f"<h1>GPS paths · {html.escape(args.date)} UTC</h1>"
             f"<p>{len(manifest)} test images. Raw GPS paths; use each image's timed table and "
             "quality note before interpreting metre-scale motion.</p><main>"
             f"{cards}</main></html>")
    (output / "index.html").write_text(index, encoding="utf-8")
    archive = output.parent / f"{output.name}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6) as zipped:
        for path in [output / "index.html", output / "manifest.json", *sorted(output.glob("*.png"))]:
            zipped.write(path, arcname=f"{output.name}/{path.name}")
    print(f"Created {len(manifest)} images in {output}")
    print(f"Download bundle: {archive}")


if __name__ == "__main__":
    main()
