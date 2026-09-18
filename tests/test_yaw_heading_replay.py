import csv
import ctypes
import os
from pathlib import Path
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT.parents[1] / "dataout" / "THEPTESTV3" / "T40_B_13.CSV"


class Config(ctypes.Structure):
    _fields_ = [
        ("yaw_tau_s", ctypes.c_float),
        ("rate_kp", ctypes.c_float),
        ("rate_ki", ctypes.c_float),
        ("heading_tau_s", ctypes.c_float),
        ("heading_kp", ctypes.c_float),
        ("max_yaw_target_dps", ctypes.c_float),
        ("min_throttle", ctypes.c_float),
        ("steering_deadband", ctypes.c_float),
        ("recapture_delay_s", ctypes.c_float),
    ]


class Input(ctypes.Structure):
    _fields_ = [
        ("dt_s", ctypes.c_float),
        ("yaw_rate_dps", ctypes.c_float),
        ("heading_deg", ctypes.c_float),
        ("throttle", ctypes.c_float),
        ("feedforward_c", ctypes.c_float),
        ("steering", ctypes.c_float),
        ("enabled", ctypes.c_bool),
        ("driving", ctypes.c_bool),
        ("gyro_fresh", ctypes.c_bool),
        ("heading_valid", ctypes.c_bool),
        ("base_capture_now", ctypes.c_bool),
    ]


class Output(ctypes.Structure):
    _fields_ = [
        ("active", ctypes.c_bool),
        ("heading_hold", ctypes.c_bool),
        ("saturated", ctypes.c_bool),
        ("heading_target_deg", ctypes.c_float),
        ("heading_error_deg", ctypes.c_float),
        ("yaw_target_dps", ctypes.c_float),
        ("yaw_filt_dps", ctypes.c_float),
        ("rate_error_dps", ctypes.c_float),
        ("p_term", ctypes.c_float),
        ("i_term", ctypes.c_float),
        ("dynamic_c", ctypes.c_float),
        ("effective_c", ctypes.c_float),
        ("c_limit", ctypes.c_float),
    ]


class Controller(ctypes.Structure):
    _fields_ = [
        ("initialized", ctypes.c_bool),
        ("heading_hold", ctypes.c_bool),
        ("steering_suspended", ctypes.c_bool),
        ("yaw_filt", ctypes.c_float),
        ("heading_target", ctypes.c_float),
        ("heading_error_filt", ctypes.c_float),
        ("integral", ctypes.c_float),
        ("recapture_elapsed_s", ctypes.c_float),
    ]


def _build_library(path):
    subprocess.run([
        os.environ.get("CC", "cc"),
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
        "-I", str(ROOT / "main"),
        str(ROOT / "main" / "yaw_heading_control.c"),
        "-lm", "-o", str(path),
    ], check=True)
    lib = ctypes.CDLL(str(path))
    lib.yaw_heading_control_init.argtypes = [ctypes.POINTER(Controller)]
    lib.yaw_heading_control_update.argtypes = [
        ctypes.POINTER(Controller), ctypes.POINTER(Config), ctypes.POINTER(Input)
    ]
    lib.yaw_heading_control_update.restype = Output
    return lib


def test_recorded_high_yaw_never_disables_or_reverses_correction():
    if not DATASET.exists():
        pytest.skip(f"recorded replay dataset absent: {DATASET}")

    with DATASET.open(newline="") as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith("#")))

    cfg = Config(0.15, 0.050, 0.020, 0.35, 0.80, 8.0, 0.15, 0.02, 0.50)
    ctl = Controller()
    high_yaw = []
    last_t = None

    with tempfile.TemporaryDirectory() as td:
        lib = _build_library(Path(td) / "libyaw_heading.so")
        lib.yaw_heading_control_init(ctypes.byref(ctl))
        for row in rows:
            yaw = float(row["yaw_dps"])
            now = float(row["t_s"])
            dt = 0.02 if last_t is None else max(0.005, min(0.2, now - last_t))
            last_t = now
            feedforward = float(row.get("c_learn") or row.get("c") or 0.21)
            out = lib.yaw_heading_control_update(
                ctypes.byref(ctl), ctypes.byref(cfg), ctypes.byref(Input(
                    dt, yaw, 0.0, 0.40, feedforward, 0.0,
                    True, True, True, False, True,
                )))
            if yaw < -10.0:
                high_yaw.append((yaw, out))

    assert len(high_yaw) >= 50
    assert all(out.active for _, out in high_yaw)
    assert all(out.dynamic_c > 0.0 for _, out in high_yaw)
