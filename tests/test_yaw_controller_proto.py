from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

FIELDS = {
    "heading_target_deg": 9,
    "heading_error_deg": 10,
    "yaw_target_dps": 11,
    "p_term": 12,
    "i_term": 13,
    "dynamic_c": 14,
    "effective_c": 15,
    "c_limit": 16,
    "ctrl_active": 17,
    "heading_hold": 18,
    "saturated": 19,
}

MOTOR_FIELDS = {
    "heading_target_deg": 14,
    "heading_error_deg": 15,
    "p_term": 16,
    "i_term": 17,
    "dynamic_c": 18,
    "effective_c": 19,
    "c_limit": 20,
    "ctrl_active": 21,
    "heading_hold": 22,
    "saturated": 23,
    "fusion_age_ms": 24,
    "rate_error_dps": 25,
    "motor_yaw_target_dps": 26,
    "motor_yaw_filt_dps": 27,
}


def test_bench_status_adds_controller_fields_without_changing_old_tags():
    proto = (ROOT / "main" / "proto" / "boat.proto").read_text()
    body = re.search(r"message BenchStatus\s*\{(.*?)\n\}", proto, re.S).group(1)
    for name, tag in FIELDS.items():
        assert re.search(rf"\b(?:float|bool)\s+{name}\s*=\s*{tag}\s*;", body)
    for name, tag in {
        "state": 1, "kind": 2, "base": 3, "samples": 4,
        "file_index": 5, "elapsed_s": 6, "learn_c": 7, "p_on": 8,
    }.items():
        assert re.search(rf"\b(?:uint32|float|bool)\s+{name}\s*=\s*{tag}\s*;", body)


def test_generated_python_round_trips_controller_diagnostics():
    from proto import boat_pb2

    values = {
        "heading_target_deg": 181.5,
        "heading_error_deg": -2.25,
        "yaw_target_dps": -1.8,
        "p_term": -0.09,
        "i_term": 0.04,
        "dynamic_c": -0.05,
        "effective_c": 0.16,
        "c_limit": 1.0,
        "ctrl_active": True,
        "heading_hold": True,
        "saturated": False,
    }
    wire = boat_pb2.BoatMessage(
        bench_status=boat_pb2.BenchStatus(**values)
    ).SerializeToString()
    assert len(wire) <= 250
    decoded = boat_pb2.BoatMessage.FromString(wire).bench_status
    for name, value in values.items():
        actual = getattr(decoded, name)
        if isinstance(value, bool):
            assert actual is value
        else:
            assert abs(actual - value) < 1e-5


def test_motor_status_carries_live_controller_diagnostics():
    from proto import boat_pb2

    proto = (ROOT / "main" / "proto" / "boat.proto").read_text()
    body = re.search(r"message MotorStatus\s*\{(.*?)\n\}", proto, re.S).group(1)
    for name, tag in MOTOR_FIELDS.items():
        field_type = "uint32" if name == "fusion_age_ms" else "bool" if name in {
            "ctrl_active", "heading_hold", "saturated"
        } else "float"
        assert re.search(rf"\b{field_type}\s+{name}\s*=\s*{tag}\s*;", body)

    values = {
        "heading_target_deg": 181.5,
        "heading_error_deg": -2.25,
        "p_term": -0.09,
        "i_term": 0.04,
        "dynamic_c": -0.05,
        "effective_c": 0.16,
        "c_limit": 1.0,
        "ctrl_active": True,
        "heading_hold": True,
        "saturated": False,
        "fusion_age_ms": 7,
        "rate_error_dps": -1.8,
        "motor_yaw_target_dps": -1.25,
        "motor_yaw_filt_dps": 0.55,
    }
    wire = boat_pb2.BoatMessage(
        motor_status=boat_pb2.MotorStatus(**values)
    ).SerializeToString()
    assert len(wire) <= 250
    decoded = boat_pb2.BoatMessage.FromString(wire).motor_status
    for name, value in values.items():
        actual = getattr(decoded, name)
        if isinstance(value, bool):
            assert actual is value
        elif isinstance(value, int):
            assert actual == value
        else:
            assert abs(actual - value) < 1e-5
