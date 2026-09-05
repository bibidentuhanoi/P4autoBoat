"""Five review findings on the BASE10 work, each pinned end to end.

1. STOP must abort a firmware-owned bench run. It sent motor zeros, which the
   run overrides every cycle; only DISARM could stop it. Now STOP also sends
   BenchCommand.abort, a dedicated backward-compatible field -- not a magic
   kind, and not by turning STOP into DISARM.
2. The SD filename shown in the UI was derived from BENCH_KIND_NAME.charAt(0),
   so BASE10 displayed 'B' while the boat wrote 'G'.
3. Laptop recording completeness assumed a 3 s drive. A 3 s fragment of a 10 s
   run passed as complete.
4. (tools/bench_analyze.py -- see test_bench_analyze.py.)
5. (firmware SD save honesty -- see test_runtime_architecture.py, BENCH_MAIN.)
"""

import importlib.util
import json
import re
import subprocess
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'
SPEC = importlib.util.spec_from_file_location('espnow_drive_stopabort', TOOL)
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


def _link(sent):
    link = D.BoatLink.__new__(D.BoatLink)
    link._lock = threading.RLock()
    link._now = __import__('time').monotonic
    link.pb2 = D.load_boat_pb2()
    link.connected = True
    for k, v in dict(throttle=0.0, motor_left=0.0, motor_right=0.0,
                     motor_split=False, rudder=0.0, winch_speed=0.0,
                     winch_lease_until=0.0, winch_command_seq=0,
                     armed_cmd=True, force=False, calibrating=False,
                     servo_rail_cut=None, p_assist_on=False,
                     assist_rudder_on=False, assist_request_id=0,
                     _assist_req_seq=0, _assist_off_req_id=None,
                     _assist_off_next_retry=0.0, _assist_off_started=0.0,
                     _last_hb_state=None, session_id=None, session_seq=0,
                     session_last_hb=0.0, lease_expired_at=None,
                     bench_yaw_samples=[], bench_yaw=None, bench_run=None,
                     _bench_write=None, bench_csv_name=None,
                     bench_dir=Path('/tmp'), rudder_test=None,
                     rudder_test_result=None, _rudder_test_write=None,
                     rudder_test_dir=Path('/tmp'), seq=0, last_error=None,
                     ser=None, port='/dev/null').items():
        setattr(link, k, v)
    for b in ('telemetry', 'motor_status', 'bench_status', 'calibrate_status',
              'system_status', 'bridge_status'):
        setattr(link, b, getattr(D.BoatLink, '_blank_' + b)())
    link._close_locked = lambda: None

    def w(payload):
        m = link.pb2.BoatMessage()
        m.ParseFromString(payload)
        sent.append(m)
        return True
    link._write_locked = w
    return link


class StopSendsBenchAbortTest(unittest.TestCase):

    def setUp(self):
        self.sent = []
        self.link = _link(self.sent)

    def _bench_msgs(self):
        return [m for m in self.sent if m.WhichOneof('payload') == 'bench']

    def test_the_field_exists_and_is_number_five(self):
        f = self.link.pb2.BenchCommand.DESCRIPTOR.fields_by_name.get('abort')
        self.assertIsNotNone(f, 'BenchCommand.abort missing from the pb2 -- '
                                'regenerate proto/boat_pb2.py')
        self.assertEqual(f.number, 5)
        self.assertEqual(f.type, f.TYPE_BOOL)

    def test_stop_sends_a_bench_abort(self):
        """THE fix. STOP must reach the run the boat owns, not just the
        manual path the run overrides."""
        ok, err = self.link.stop(1)
        self.assertTrue(ok, err)
        aborts = [m for m in self._bench_msgs() if m.bench.abort]
        self.assertEqual(len(aborts), 1, 'STOP did not send BenchCommand.abort')

    def test_stop_still_sends_its_manual_zeros_too(self):
        """Additive. The zeros still go out for the manual path -- STOP's
        existing semantics are preserved, the abort is on top."""
        self.link.stop(1)
        kinds = [m.WhichOneof('payload') for m in self.sent]
        self.assertIn('motor', kinds)
        self.assertIn('steer', kinds)
        self.assertIn('winch', kinds)
        self.assertIn('bench', kinds)

    def test_stop_does_not_touch_the_arm_state(self):
        """'do not globally change STOP into DISARM'."""
        self.link.armed_cmd = True
        self.link.stop(1)
        self.assertTrue(self.link.armed_cmd)
        self.assertNotIn('arm_cmd', [m.WhichOneof('payload') for m in self.sent])

    def test_the_abort_is_a_pure_abort_not_a_disguised_start(self):
        self.link.stop(1)
        a = [m for m in self._bench_msgs() if m.bench.abort][0]
        self.assertEqual(a.bench.kind, 0)
        self.assertEqual(a.bench.base, 0.0)
        self.assertEqual(a.bench.delta, 0.0)
        self.assertEqual(a.bench.reset_c, 0.0)

    def test_starting_a_run_never_sets_abort(self):
        """Backward compatibility the other way: every existing start is
        unchanged on the wire, abort=false, exactly as before the field."""
        ok, err = self.link.send_bench('both', 0.2, 0.0, 1)
        self.assertTrue(ok, err)
        starts = self._bench_msgs()
        self.assertEqual(len(starts), 1)
        self.assertFalse(starts[0].bench.abort)
        self.assertEqual(starts[0].bench.kind, 0)

    def test_stop_while_disconnected_is_still_a_refusal(self):
        self.link.connected = False
        ok, err = self.link.stop(1)
        self.assertFalse(ok)
        self.assertEqual(self._bench_msgs(), [])


class SdFilenameDisplayTest(unittest.TestCase):
    """The letter the UI shows must be the letter the boat writes."""

    EXPECT = {0: 'T20_B_01.CSV', 1: 'T20_L_01.CSV',
              2: 'T20_R_01.CSV', 3: 'T20_G_01.CSV'}

    def _display(self, kind):
        src = TOOL.read_text()
        m = re.search(r"\$\('bench-file'\)\.textContent = (.*?);\n", src, re.S)
        self.assertIsNotNone(m, 'the bench-file expression moved')
        expr = m.group(1)
        # BENCH_KIND_NAME / BENCH_SD_CHAR as the page defines them
        defs = re.findall(r'^const (BENCH_KIND_NAME|BENCH_SD_CHAR) = .*?;$',
                          src, re.M)
        consts = '\n'.join(l for l in src.splitlines()
                           if l.startswith(('const BENCH_KIND_NAME =',
                                            'const BENCH_SD_CHAR =')))
        js = ('%s\nvar bn = %s; var pct = Math.round(bn.base * 100);\n'
              'console.log(%s);' % (consts, json.dumps(
                  {'kind': kind, 'base': 0.2, 'file_index': 1, 'have': True}),
                  expr))
        out = subprocess.run(['node', '-e', js], capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stderr)
        return out.stdout.strip()

    def test_every_kind_displays_the_letter_the_boat_writes(self):
        for kind, want in self.EXPECT.items():
            self.assertEqual(self._display(kind), want,
                             'kind %d shows the wrong SD filename' % kind)

    def test_the_letter_comes_from_an_explicit_map_not_charAt(self):
        src = TOOL.read_text()
        self.assertIn("const BENCH_SD_CHAR = { 0: 'B', 1: 'L', 2: 'R', 3: 'G' };",
                      src)
        m = re.search(r"\$\('bench-file'\)\.textContent = (.*?);\n", src, re.S)
        self.assertNotIn('charAt', m.group(1),
                         "still deriving the SD letter from the display name; "
                         "BASE10.charAt(0) is 'B' and the boat writes 'G'")

    def test_the_map_agrees_with_the_firmware(self):
        mc = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn("(s_bench.kind == BENCH_KIND_LEFT) ? 'L'", mc)
        self.assertIn("(s_bench.kind == BENCH_KIND_RIGHT) ? 'R'", mc)
        self.assertIn("(s_bench.kind == BENCH_KIND_BASE_LONG) ? 'G' : 'B'", mc)


class DurationAwareCompletenessTest(unittest.TestCase):
    """A 3 s fragment of a 10 s run is not a complete run."""

    @staticmethod
    def _samples(seconds, hz=20.0, yaw=-5.0):
        n = int(seconds * hz)
        return [(1000.0 + i / hz, yaw, None) for i in range(n)]

    def test_the_per_kind_duration_map(self):
        self.assertEqual(D.BENCH_DRIVE_S_BY_KIND[0], 3.0)
        self.assertEqual(D.BENCH_DRIVE_S_BY_KIND[1], 3.0)
        self.assertEqual(D.BENCH_DRIVE_S_BY_KIND[2], 3.0)
        self.assertEqual(D.BENCH_DRIVE_S_BY_KIND[3], 10.0)
        self.assertEqual(D.BENCH_DRIVE_S, 3.0)      # the default is unchanged

    def test_three_seconds_of_a_ten_second_run_is_incomplete(self):
        s = D.summarize_yaw(self._samples(3.0), drive_s=10.0)
        self.assertTrue(s['incomplete'])

    def test_two_seconds_of_a_ten_second_run_is_incomplete(self):
        s = D.summarize_yaw(self._samples(2.0), drive_s=10.0)
        self.assertTrue(s['incomplete'])

    def test_adequate_ten_second_coverage_passes(self):
        """The existing policy -- BENCH_YAW_MIN_COVERAGE of the drive -- applied
        to the right drive length."""
        s = D.summarize_yaw(self._samples(10.0), drive_s=10.0)
        self.assertFalse(s['incomplete'])
        # and at exactly the policy threshold
        s = D.summarize_yaw(self._samples(10.0 * D.BENCH_YAW_MIN_COVERAGE + 0.1),
                            drive_s=10.0)
        self.assertFalse(s['incomplete'])

    def test_ordinary_three_second_runs_judge_exactly_as_before(self):
        self.assertFalse(D.summarize_yaw(self._samples(3.0))['incomplete'])
        self.assertTrue(D.summarize_yaw(self._samples(1.0))['incomplete'])
        # explicit default == implicit default
        self.assertEqual(D.summarize_yaw(self._samples(3.0))['incomplete'],
                         D.summarize_yaw(self._samples(3.0), drive_s=3.0)['incomplete'])

    def test_the_handler_passes_the_kind_duration_through(self):
        """The call site, not just the function: a BASE10 run judged with the
        3 s default would pass on a fragment."""
        sent = []
        link = _link(sent)

        def st(state, kind, **kw):
            link._handle_bench_status(link.pb2.BenchStatus(state=state, kind=kind, **kw))

        # BASE10 with only 3 s of caught frames -> incomplete
        st(D.BENCH_STATE_RUN, 3, base=0.2)
        link.bench_yaw_samples = self._samples(3.0)
        st(D.BENCH_STATE_SAVED, 3, base=0.2, file_index=1)
        self.assertTrue(link.bench_yaw['incomplete'])

        # BASE10 with 10 s of frames -> complete
        link.bench_run = None; link._bench_write = None
        st(D.BENCH_STATE_RUN, 3, base=0.2)
        link.bench_yaw_samples = self._samples(10.0)
        st(D.BENCH_STATE_SAVED, 3, base=0.2, file_index=2)
        self.assertFalse(link.bench_yaw['incomplete'])

        # ordinary BASE with 3 s -> complete, exactly as before
        link.bench_run = None; link._bench_write = None
        st(D.BENCH_STATE_RUN, 0, base=0.2)
        link.bench_yaw_samples = self._samples(3.0)
        st(D.BENCH_STATE_SAVED, 0, base=0.2, file_index=3)
        self.assertFalse(link.bench_yaw['incomplete'])


class StaleCommentsTest(unittest.TestCase):
    """The bench comments that said every run is 3 s are no longer true.

    The RUDDER-test comments that say 4.5 s are left alone: that profile is
    0.5 + 3.0 + 1.0 s and really is 4.5 s."""

    def test_bench_comments_no_longer_claim_a_fixed_three_seconds(self):
        src = TOOL.read_text()
        i = src.index('BENCH_DRIVE_S = 3.0')
        block = src[i - 400:i + 900]
        self.assertIn('BASE_LONG', block)
        self.assertIn('10', block)
        j = src.index('def _collect_bench_yaw_locked')
        cap = src[j:j + 900]
        # it may still say 3 s -- as long as it says 10 s for a BASE10 too
        self.assertIn('BASE10', cap)
        self.assertIn('10 s', cap)


if __name__ == '__main__':
    unittest.main()
