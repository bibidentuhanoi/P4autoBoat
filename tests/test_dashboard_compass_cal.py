"""dashboard.html's compass-calibration card, driven through a full session
in node with stand-in page elements: Calibrate press -> waits for the boat's
answer (run id) -> hold still / spin checklist -> Cancel -> PASS -> idle with
the last result; DISARM-first refusal; first-boot "gyro drift + level saved"."""
from pathlib import Path
import shutil
import subprocess
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_compass_cal_card_session():
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    html = (ROOT / "main" / "dashboard.html").read_text()
    start = html.index("// ── Compass + IMU calibration ──")
    end = html.index("function updateCalibrateStatus(cs) {")
    with tempfile.TemporaryDirectory() as tmp:
        logic = Path(tmp) / "ccal_logic.js"
        logic.write_text(html[start:end])
        result = subprocess.run(
            [node, str(ROOT / "tests" / "dashboard_compass_cal_check.js"), str(logic)],
            capture_output=True, text=True,
        )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout
