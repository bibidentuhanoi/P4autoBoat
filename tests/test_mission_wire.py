"""MissionCommand / MissionStatus on the wire (spec section 6).

MissionStatus must travel as ONE ESP-NOW packet (the link drops fragments
under motor load) and be labelled MSG_MISSION_STATUS (0x09) by the boat's
ESP-NOW sender, which recognises BoatMessage field 22 by its two tag bytes."""
import re
from pathlib import Path

from proto import boat_pb2

ROOT = Path(__file__).resolve().parents[1]
ESPNOW_MAX_PAYLOAD = 244
ESPNOW_HDR = 4


def test_the_wire_tags_are_what_the_sender_matches():
    status = boat_pb2.BoatMessage()
    status.mission_status.state = 2
    assert status.SerializeToString()[:2] == b"\xb2\x01"
    cmd = boat_pb2.BoatMessage()
    cmd.mission.start = True
    assert cmd.SerializeToString()[:2] == b"\xaa\x01"


def test_a_fully_populated_mission_status_is_one_espnow_packet():
    msg = boat_pb2.BoatMessage()
    st = msg.mission_status
    for field in st.DESCRIPTOR.fields:
        if field.type == field.TYPE_BOOL:
            setattr(st, field.name, True)
        elif field.type == field.TYPE_UINT32:
            setattr(st, field.name, 0xFFFFFFFF)
        elif field.type in (field.TYPE_FLOAT, field.TYPE_DOUBLE):
            setattr(st, field.name, -123456.789)
        else:
            raise AssertionError("unexpected field type in MissionStatus: %s" % field.name)
    encoded = len(msg.SerializeToString())
    assert encoded + ESPNOW_HDR <= ESPNOW_MAX_PAYLOAD, encoded
    size = int(re.search(r"#define boat_MissionStatus_size\s+(\d+)",
                         (ROOT / "main" / "proto" / "boat.pb.h").read_text()).group(1))
    assert size + 2 + 2 + ESPNOW_HDR <= ESPNOW_MAX_PAYLOAD      # tag + length + header


def test_the_espnow_sender_labels_mission_status():
    proto_h = (ROOT / "main" / "transports" / "espnow_protocol.h").read_text()
    assert re.search(r"MSG_MISSION_STATUS\s*=\s*0x09", proto_h)
    send = (ROOT / "main" / "transports" / "espnow_transport.c").read_text()
    assert ("case 0xB2: msg_type = (len > 1 && buf[1] == 0x01) ? MSG_MISSION_STATUS : MSG_SENSOR; break;"
            in send)
    # the old labels are untouched
    assert "case 0x1A: msg_type = MSG_STATUS;" in send
    assert "case 0x32: msg_type = MSG_MOTOR_STATUS;" in send
