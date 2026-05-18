#!/usr/bin/env python3
"""Extract overlay projection constants from dashboard.html into JSON."""
import re, json, sys

def extract(html_path):
    with open(html_path, 'r') as f:
        html = f.read()

    def find_const(pattern, cast=float):
        m = re.search(pattern, html)
        return cast(m.group(1)) if m else None

    params = {
        "cam": {"w": 800, "h": 640},
        "fx": find_const(r"const\s+FX\s*=\s*([0-9.]+)"),
        "cx": find_const(r"const\s+CX\s*=\s*([0-9.]+)"),
        "cy": find_const(r"const\s+CY\s*=\s*([0-9.]+)"),
        "az_offset": find_const(r"let\s+AZ_OFFSET\s*=\s*([0-9.\-]+)"),
        "el_offset": find_const(r"let\s+EL_OFFSET\s*=\s*([0-9.\-]+)"),
        "sensor_tz": find_const(r"const\s+SENSOR_TZ\s*=\s*([0-9.]+)"),
        "zone_step": find_const(r"const\s+ZONE_STEP\s*=\s*([0-9.]+)"),
        "cam_scale": find_const(r"let\s+camScale\s*=\s*([0-9.]+)"),
    }

    m_a = re.search(r"SENSOR_A\s*=\s*\{\s*x:\s*([0-9.\-]+),\s*tilt:\s*([0-9.\-]+)", html)
    m_b = re.search(r"SENSOR_B\s*=\s*\{\s*x:\s*([0-9.\-]+),\s*tilt:\s*([0-9.\-]+)", html)
    params["sensor_a"] = {"x": float(m_a.group(1)), "tilt": float(m_a.group(2))} if m_a else None
    params["sensor_b"] = {"x": float(m_b.group(1)), "tilt": float(m_b.group(2))} if m_b else None

    params["flip"] = {
        "ha": "flipHA: true" in html,
        "va": "flipVA: true" in html,
        "ta": "transA: true" in html,
        "hb": "flipHB: true" in html,
        "vb": "flipVB: true" in html,
        "tb": "transB: true" in html,
    }

    params["drag"] = {"x": 0, "y": 0}

    return params

if __name__ == "__main__":
    html_path = sys.argv[1] if len(sys.argv) > 1 else "main/dashboard.html"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "overlay_params.json"

    params = extract(html_path)
    with open(out_path, 'w') as f:
        json.dump(params, f, indent=2)

    print(f"Extracted overlay params from {html_path} → {out_path}")
    print(json.dumps(params, indent=2))
