"""Live signed yaw rate (gyro Z) on both control UIs.

This exists to establish which way round the installed IMU reads, BEFORE any
steering work depends on it. So the one thing that must not happen is the value
being quietly altered on the way to the screen: a negation, an abs(), or an
extra filter would corrupt the very measurement being taken and the corruption
would look exactly like a real reading.

The value therefore has to survive, signed and raw, along two independent
transports that reach two different UIs:

    WiFi/WS   FusionResult.yaw_rate -> SensorSnapshot.imu.yaw_rate (protobuf)
              -> dashboard.html

    ESP-NOW   FusionResult.yaw_rate -> espnow_telemetry_t.yaw_rate (packed
              struct, no protobuf at all) -> tools/espnow_drive.py

and tools/espnow_drive.py also decodes the FULL protobuf snapshot, so it has
two input paths of its own that must agree.

The schema-parity check below is not decoration. protobuf.js silently DROPS
fields the embedded schema does not declare, so a boat.proto change that is not
mirrored into dashboard.html produces no error anywhere -- it just makes the
value vanish. That has already cost this project a UI lockout once.
"""

import re
import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from proto import boat_pb2  # noqa: E402


class YawRateProtoTest(unittest.TestCase):

    def test_the_field_exists_with_the_agreed_number_and_type(self):
        f = boat_pb2.IMUData.DESCRIPTOR.fields_by_name.get('yaw_rate')
        self.assertIsNotNone(f, 'IMUData.yaw_rate missing from the generated pb2')
        self.assertEqual(f.number, 4)
        self.assertEqual(f.type, f.TYPE_FLOAT)

    def test_a_negative_rate_survives_a_round_trip(self):
        """Signedness end to end through real protobuf encode/decode -- not a
        property of the field declaration, so it gets exercised rather than
        assumed."""
        for value in (-12.34, -0.01, 0.0, +0.01, +180.0):
            snap = boat_pb2.SensorSnapshot()
            snap.imu.yaw_rate = value
            got = boat_pb2.SensorSnapshot.FromString(snap.SerializeToString())
            self.assertAlmostEqual(got.imu.yaw_rate, value, places=4)

    def test_the_nanopb_outputs_were_regenerated(self):
        """The .pb.c/.pb.h are committed, so a proto edit without a regenerate
        leaves the firmware unable to send the field at all."""
        h = (ROOT / 'main' / 'proto' / 'boat.pb.h').read_text()
        self.assertIn('#define boat_IMUData_yaw_rate_tag                4', h)
        self.assertRegex(h, r'float\s+yaw_rate;')
        c = (ROOT / 'main' / 'proto' / 'boat.pb.c').read_text()
        self.assertIn('boat_IMUData', c)

    def test_the_dashboard_schema_matches_boat_proto(self):
        """The parity trap: protobuf.js drops undeclared fields SILENTLY. The
        embedded schema must carry the same field number and type."""
        proto = (ROOT / 'main' / 'proto' / 'boat.proto').read_text()
        m = re.search(r'message IMUData \{(.*?)\}', proto, re.S)
        self.assertIsNotNone(m)
        proto_fields = dict(re.findall(r'(\w+)\s*=\s*(\d+)\s*;', m.group(1)))

        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        m = re.search(r'message IMUData \{(.*?)\}', dash)
        self.assertIsNotNone(m, 'IMUData missing from the embedded schema')
        dash_fields = dict(re.findall(r'(\w+)\s*=\s*(\d+)\s*;', m.group(1)))

        self.assertEqual(proto_fields, dash_fields,
                         'dashboard.html IMUData is out of sync with boat.proto '
                         '-- protobuf.js will silently drop the difference')
        self.assertEqual(dash_fields.get('yaw_rate'), '4')
        self.assertIn('float yaw_rate = 4;', m.group(1))


class YawRateFirmwareTest(unittest.TestCase):

    def test_both_transports_send_the_same_fusion_field(self):
        """One source value, two transports. If they ever diverge the two UIs
        disagree about the boat and there is no way to tell which is right."""
        src = (ROOT / 'main' / 'sensor_task.c').read_text()
        self.assertIn('.yaw_rate = imu.yaw_rate,', src,
                      'the ESP-NOW compact path stopped sending yaw_rate')
        self.assertIn('snap.imu.yaw_rate = imu.yaw_rate;', src,
                      'the WiFi/protobuf path does not populate yaw_rate')

    def test_the_firmware_does_not_alter_the_value_on_the_way_out(self):
        """No negation, no abs, no scaling at the telemetry boundary."""
        src = (ROOT / 'main' / 'sensor_task.c').read_text()
        for bad in ('-imu.yaw_rate', 'fabsf(imu.yaw_rate)', 'fabs(imu.yaw_rate)',
                    'imu.yaw_rate *', 'YAW_GYRO_SIGN'):
            self.assertNotIn(bad, src,
                             'sensor_task.c is modifying the yaw rate (%s); it '
                             'must ship raw and signed' % bad)


class YawRatePythonToolTest(unittest.TestCase):
    """tools/espnow_drive.py has TWO decode paths and both feed one renderer."""

    def setUp(self):
        self.src = (ROOT / 'tools' / 'espnow_drive.py').read_text()

    def test_the_compact_struct_layout_still_carries_yaw_rate_last(self):
        m = re.search(r"FIELD_TELEMETRY_FMT = '([^']+)'", self.src)
        self.assertIsNotNone(m)
        fmt = m.group(1)
        self.assertTrue(fmt.endswith('f'), 'yaw_rate must stay the last field')
        # Mirrors espnow_telemetry_t: 3f + B + 2d + 2f + B + f + f
        self.assertEqual(fmt, '<fffBddffBff')
        # and the unpack must actually round-trip a NEGATIVE value
        packed = struct.pack(fmt, 1.0, 2.0, 3.0, 1, 4.0, 5.0, 6.0, 7.0, 8, 9.0, -12.34)
        self.assertAlmostEqual(struct.unpack(fmt, packed)[-1], -12.34, places=4)

    def test_the_compact_path_stores_the_signed_value(self):
        self.assertIn("'yaw_rate': yaw_rate,", self.src)

    def test_the_full_snapshot_path_stores_it_too(self):
        """The full path REPLACES the telemetry dict wholesale, so a missing
        key does not fall back to the blank default -- it removes the key and
        the renderer KeyErrors on every frame."""
        self.assertIn("'yaw_rate': s.imu.yaw_rate,", self.src)

    def test_both_python_paths_produce_the_same_keys(self):
        """Whichever transport is live, the renderer sees the same dict shape."""
        blank = self._dict_keys_after("def _blank_telemetry")
        compact = self._dict_keys_after("(pitch, roll, heading, gps_valid")
        full = self._dict_keys_after("s = msg.sensors")
        self.assertIn('yaw_rate', blank)
        self.assertEqual(compact, full,
                         'the compact and full telemetry paths disagree about '
                         'which keys they set')
        self.assertTrue(compact.issubset(blank),
                        'a path sets a key the blank default does not define')

    def _dict_keys_after(self, anchor):
        i = self.src.index(anchor)
        chunk = self.src[i:i + 900]
        return set(re.findall(r"'(\w+)':", chunk)) - {'have', 'last_rx_monotonic'}

    def test_the_renderer_shows_it_signed_and_falls_back_to_dashes(self):
        self.assertIn("$('t-yawrate')", self.src)
        self.assertIn('<span class="val" id="t-yawrate">--</span>', self.src)
        m = re.search(r"\$\('t-yawrate'\)\.textContent = (.*?);", self.src, re.S)
        self.assertIsNotNone(m)
        expr = m.group(1)
        self.assertIn("'+'", expr, 'positive values must carry an explicit +')
        self.assertIn('toFixed(2)', expr)
        self.assertIn("'--'", expr, 'no value must render as --')

    def test_the_tool_does_not_alter_the_value(self):
        for bad in ('-t.yaw_rate', 'Math.abs(t.yaw_rate)', "-yaw_rate",
                    'abs(yaw_rate)'):
            self.assertNotIn(bad, self.src,
                             'espnow_drive.py is modifying the yaw rate (%s)' % bad)


class YawRateDashboardTest(unittest.TestCase):

    def setUp(self):
        self.src = (ROOT / 'main' / 'dashboard.html').read_text()

    def test_the_card_has_a_yaw_rate_readout_defaulting_to_dashes(self):
        self.assertIn('id="imu-yawrate">--</div>', self.src)

    def test_the_renderer_formats_it_signed_from_the_snapshot(self):
        self.assertIn("$('imu-yawrate').textContent = yawRateText(s.imu.yawRate);",
                      self.src)
        m = re.search(r'function yawRateText\(v\) \{(.*?)\n\}', self.src, re.S)
        self.assertIsNotNone(m, 'yawRateText() missing')
        body = m.group(1)
        self.assertIn("return '--'", body, 'no value must render as --')
        self.assertIn('toFixed(2)', body)
        self.assertIn("'+'", body, 'positive values must carry an explicit +')

    def test_a_dropped_link_blanks_the_rate_rather_than_freezing_it(self):
        """A frozen attitude reads as "level"; a frozen RATE reads as "turning
        right now", which is actively misleading while taking a measurement."""
        i = self.src.index('ws.onclose')
        self.assertIn("$('imu-yawrate').textContent = '--';",
                      self.src[i:i + 900])

    def test_the_dashboard_does_not_alter_the_value(self):
        for bad in ('-s.imu.yawRate', 'Math.abs(s.imu.yawRate)',
                    's.imu.yawRate *', 'yawRateText(-'):
            self.assertNotIn(bad, self.src,
                             'dashboard.html is modifying the yaw rate (%s)' % bad)

    def test_neither_ui_labels_a_sign_left_or_right_yet(self):
        """The direction has NOT been measured. Labelling a sign now would bake
        in a guess that later work would trust."""
        tool = (ROOT / 'tools' / 'espnow_drive.py').read_text()
        for name, text in (('dashboard.html', self.src), ('espnow_drive.py', tool)):
            for line in text.splitlines():
                low = line.lower()
                if 'yawrate' in low.replace('_', '') and (
                        'left' in low or 'right' in low):
                    # a comment saying the direction is NOT established is fine
                    if 'not' in low or 'yet' in low or 'depends' in low:
                        continue
                    self.fail('%s labels a yaw-rate sign as left/right before '
                              'it was measured: %s' % (name, line.strip()))


class YawRateFormattingTest(unittest.TestCase):
    """The formatting rule both UIs implement, checked as behaviour."""

    @staticmethod
    def fmt(v):
        if not isinstance(v, (int, float)):
            return '--'
        return ('+' if v >= 0 else '') + ('%.2f' % v)

    def test_the_agreed_format(self):
        self.assertEqual(self.fmt(12.34), '+12.34')
        self.assertEqual(self.fmt(-12.34), '-12.34')
        self.assertEqual(self.fmt(0.0), '+0.00')
        self.assertEqual(self.fmt(None), '--')
        # the case the explicit sign exists for
        self.assertNotEqual(self.fmt(0.2), self.fmt(-0.2))


if __name__ == '__main__':
    unittest.main()
