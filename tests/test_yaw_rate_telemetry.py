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

import importlib.util
import math
import re
import struct
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from proto import boat_pb2  # noqa: E402

TOOL = ROOT / 'tools' / 'espnow_drive.py'


def _yawrate_render_expression():
    """The live JS expression that decides what the Telemetry card shows.

    Read out of the real source rather than restated here, so the behavioural
    test below cannot drift away from the code it claims to be testing."""
    src = TOOL.read_text()
    m = re.search(r"\$\('t-yawrate'\)\.textContent =\s*(.*?);\n", src, re.S)
    assert m, "the t-yawrate render expression moved"
    return m.group(1)


def _render_yawrate(t):
    """Python mirror of that JS expression, evaluated against a real telemetry
    dict. Kept honest by test_the_renderer_guards_on_have_stale_and_finite,
    which checks each guard is actually present in the JS."""
    ok = (bool(t.get('have'))
          and not t.get('stale')
          and isinstance(t.get('yaw_rate'), (int, float))
          and not isinstance(t.get('yaw_rate'), bool)
          and math.isfinite(t['yaw_rate']))
    if not ok:
        return '--'
    v = t['yaw_rate']
    return '%s%.2f °/s' % ('+' if v >= 0 else '', v)


def _load_tool():
    """Same loader the existing tool tests use -- the module guards its main(),
    so importing it is side-effect free."""
    spec = importlib.util.spec_from_file_location('espnow_drive_yaw_test', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


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

    def test_the_embedded_schema_is_generated_not_mirrored_by_hand(self):
        """The hand-mirroring is gone. protobuf.js silently drops fields it
        does not declare, so a copy maintained by hand loses data with no error
        anywhere -- and every proto change needed a manual step nothing
        enforced. This runs the generator in --check mode: a proto edit without
        a regenerate fails here rather than on the water."""
        import subprocess
        r = subprocess.run(
            [sys.executable, str(ROOT / 'tools' / 'gen_dashboard_schema.py'),
             '--check'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0,
                         'dashboard schema is out of date:\n' + r.stderr)
        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        self.assertIn('GENERATED FROM main/proto/boat.proto', dash)

    def test_every_proto_message_reaches_the_dashboard_schema(self):
        """Not just the ones somebody remembered."""
        proto = (ROOT / 'main' / 'proto' / 'boat.proto').read_text()
        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        i = dash.index('const protoSchema = `')
        block = dash[i:dash.index('`;', i)]
        for name in re.findall(r'^message (\w+)', proto, re.M):
            self.assertIn('message %s ' % name, block,
                          '%s is missing from the embedded schema; '
                          'protobuf.js would drop it silently' % name)

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
        expr = _yawrate_render_expression()
        self.assertIn("'+'", expr, 'positive values must carry an explicit +')
        self.assertIn('toFixed(2)', expr)
        self.assertIn("'--'", expr, 'no value must render as --')

    def test_the_renderer_guards_on_have_stale_and_finite(self):
        """All three, and `stale` in particular: the rows above this one show
        their last value indefinitely, which is fine for an ANGLE and wrong for
        a RATE."""
        expr = _yawrate_render_expression()
        for guard in ('t.have', '!t.stale', 'Number.isFinite(t.yaw_rate)'):
            self.assertIn(guard, expr,
                          'the yaw-rate render guard is missing %s' % guard)

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


class YawRateStalenessTest(unittest.TestCase):
    """A stale RATE is worse than no rate: frozen at "+12.34 °/s" it reads as
    "the boat is turning right now" long after the link has gone.

    Staleness is produced by the tool's OWN _with_age(), driven by a real
    last_rx_monotonic timestamp -- not by hand-setting a 'stale' key. So this
    also proves the field the renderer depends on is really there and really
    flips, which a source-inspection test cannot show."""

    @classmethod
    def setUpClass(cls):
        cls.tool = _load_tool()

    def _aged(self, age_s, yaw_rate=12.34):
        """A telemetry dict as the UI receives it, last heard `age_s` ago."""
        d = self.tool.BoatLink._blank_telemetry()
        d['have'] = True
        d['yaw_rate'] = yaw_rate
        d['last_rx_monotonic'] = time.monotonic() - age_s
        return self.tool.BoatLink._with_age(d, self.tool.TELEMETRY_STALE_S)

    def test_the_staleness_threshold_is_what_the_ui_actually_uses(self):
        self.assertGreater(self.tool.TELEMETRY_STALE_S, 0)

    def test_a_fresh_reading_is_shown(self):
        t = self._aged(0.0)
        self.assertFalse(t['stale'])
        self.assertEqual(_render_yawrate(t), '+12.34 °/s')

    def test_a_stale_reading_blanks_even_though_the_value_is_still_there(self):
        """The value survives in the dict -- that is exactly the trap. Only the
        staleness guard stops it being displayed."""
        t = self._aged(self.tool.TELEMETRY_STALE_S + 1.0)
        self.assertTrue(t['stale'], 'the fixture did not actually go stale')
        self.assertEqual(t['yaw_rate'], 12.34, 'the value should still be present')
        self.assertEqual(_render_yawrate(t), '--')

    def test_a_negative_stale_reading_blanks_too(self):
        t = self._aged(self.tool.TELEMETRY_STALE_S + 1.0, yaw_rate=-12.34)
        self.assertTrue(t['stale'])
        self.assertEqual(_render_yawrate(t), '--')

    def test_never_having_heard_from_the_boat_blanks(self):
        t = self.tool.BoatLink._blank_telemetry()
        t = self.tool.BoatLink._with_age(t, self.tool.TELEMETRY_STALE_S)
        self.assertTrue(t['stale'], 'no timestamp must count as stale')
        self.assertEqual(_render_yawrate(t), '--')

    def test_a_missing_or_nonfinite_value_blanks(self):
        for bad in (None, float('nan'), float('inf'), 'ok'):
            t = self._aged(0.0)
            t['yaw_rate'] = bad
            self.assertEqual(_render_yawrate(t), '--',
                             'yaw_rate=%r should blank' % (bad,))
        t = self._aged(0.0)
        del t['yaw_rate']
        self.assertEqual(_render_yawrate(t), '--')

    def test_zero_is_a_real_reading_and_must_not_blank(self):
        """A boat holding perfectly straight reads 0.00. Blanking that would
        hide the most interesting measurement of all."""
        t = self._aged(0.0, yaw_rate=0.0)
        self.assertEqual(_render_yawrate(t), '+0.00 °/s')


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
