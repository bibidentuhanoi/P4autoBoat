"""STOP must be understood by the firmware the boat is actually running.

The P4 runs main 9543ed1. Its BenchCommand has no `abort` field, and a
BoatMessage carrying a bench payload -- even an empty one -- is dispatched by
main's pipeline as a bench START with kind=0/base=0; bench_start() clamps a
zero base rather than refusing it. A STOP that put BenchCommand.abort on the
wire would therefore, on that firmware, start a 3 s zero-throttle BASE run:
boat busy, junk SD and laptop files, sticks ignored.

So ordinary and lake-test STOP send exactly what 9543ed1 understands -- motor
zeros, centred steer, winch zero, immediately -- and a bench the tool believes
is running or was just requested is stopped by DISARM, which every firmware
version honours.

The 9543ed1 schema is taken from git at test time and loaded into a private
descriptor pool, so these tests decode the frames exactly as THAT firmware
reads them.
"""

import ast
import re
import subprocess
import sys
import time
import unittest
from pathlib import Path

from google.protobuf import descriptor_pool, message_factory

ROOT = Path(__file__).resolve().parents[1]
MAIN_COMMIT = '9543ed1'          # what is flashed on the P4

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_lake_id import LakeBase, T            # noqa: E402


def _main_schema_pool():
    r = subprocess.run(['git', '-C', str(ROOT), 'show', MAIN_COMMIT + ':proto/boat_pb2.py'],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise AssertionError('cannot read %s:proto/boat_pb2.py from git: %s' % (MAIN_COMMIT, r.stderr))
    m = re.search(r"AddSerializedFile\((b'(?:[^'\\]|\\.)*')\)", r.stdout)
    if not m:
        raise AssertionError('serialized descriptor not found in the %s boat_pb2.py' % MAIN_COMMIT)
    pool = descriptor_pool.DescriptorPool()
    pool.AddSerializedFile(ast.literal_eval(m.group(1)))
    return pool


MAIN_POOL = _main_schema_pool()
MainBoatMessage = message_factory.GetMessageClass(MAIN_POOL.FindMessageTypeByName('boat.BoatMessage'))


def as_main_firmware(payload):
    """Decode one wire payload the way main 9543ed1 does."""
    m = MainBoatMessage()
    m.ParseFromString(payload)
    return m


class _RawWire(LakeBase):
    """LakeBase, plus every raw payload the tool writes, in order."""

    def setUp(self):
        super().setUp()
        self.raw = []
        inner = self.link._write_locked

        def capture(payload):
            self.raw.append(bytes(payload))
            return inner(payload)
        self.link._write_locked = capture

    def main_kinds(self):
        return [as_main_firmware(p).WhichOneof('payload') for p in self.raw]


class ManualStopOnMainFirmwareTest(_RawWire):

    def test_the_main_schema_really_has_no_abort_field(self):
        """Sanity for the proof itself: the pool is 9543ed1's, not the tool's."""
        old = MAIN_POOL.FindMessageTypeByName('boat.BenchCommand')
        self.assertNotIn('abort', old.fields_by_name)
        self.assertIn('abort', T.load_boat_pb2().BenchCommand.DESCRIPTOR.fields_by_name)

    def test_manual_stop_sends_zeros_immediately_and_no_bench_payload(self):
        self.link.throttle = 0.4; self.link.rudder = -0.3
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        msgs = [as_main_firmware(p) for p in self.raw]
        kinds = [m.WhichOneof('payload') for m in msgs]
        self.assertNotIn('bench', kinds)
        self.assertEqual(kinds, ['motor', 'steer', 'winch'])
        self.assertEqual((msgs[0].motor.left, msgs[0].motor.right), (0.0, 0.0))
        self.assertEqual((msgs[1].steer.left, msgs[1].steer.right), (0.0, 0.0))
        self.assertEqual(msgs[2].winch.speed, 0.0)
        self.assertTrue(self.link.armed_cmd)              # no bench: STOP is not DISARM
        self.assertEqual((self.link.throttle, self.link.rudder, self.link.winch_speed),
                         (0.0, 0.0, 0.0))

    def test_stop_disarms_a_bench_run_the_boat_reports_running(self):
        self.link.bench_status = dict(self.link.bench_status, have=True, state=T.BENCH_STATE_RUN,
                                      last_rx_monotonic=time.monotonic())
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        msgs = [as_main_firmware(p) for p in self.raw]
        kinds = [m.WhichOneof('payload') for m in msgs]
        self.assertNotIn('bench', kinds)
        self.assertEqual(kinds, ['motor', 'steer', 'winch', 'arm_cmd'])
        self.assertFalse(msgs[3].arm_cmd.arm)
        self.assertFalse(self.link.armed_cmd)

    def test_stop_disarms_a_bench_run_just_requested(self):
        """Between the request and the boat's first BenchStatus the run is, as
        far as this tool can know, about to be live."""
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        self.raw.clear()
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch', 'arm_cmd'])
        self.assertFalse(self.link.armed_cmd)

    def test_a_request_the_boat_has_answered_or_that_is_old_does_not_disarm(self):
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        # the boat answered: the run already ended
        self.link._handle_bench_status(self.link.pb2.BenchStatus(
            state=T.BENCH_STATE_SAVED, kind=0, base=0.2, file_index=1))
        self.raw.clear()
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch'])
        self.assertTrue(self.link.armed_cmd)
        # a request older than the pending window, never answered
        self.link.bench_requested_at = time.monotonic() - T.BENCH_REQUEST_PENDING_S - 0.1
        self.raw.clear()
        self.link.stop(self.link.winch_command_seq + 1)
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch'])
        self.assertTrue(self.link.armed_cmd)

    def test_stop_while_disconnected_sends_nothing(self):
        self.link.connected = False
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertFalse(ok)
        self.assertEqual(self.raw, [])


class LakeStopOnMainFirmwareTest(_RawWire):

    def test_a_lake_run_stopped_by_the_operator_speaks_only_main_payloads(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        n_before = len(self.raw)
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        r = self._wait_result()
        self.assertIn('STOP', r['reason'])
        kinds = self.main_kinds()
        self.assertNotIn('bench', kinds)
        self.assertTrue(set(kinds) <= {'motor', 'steer', 'winch'}, kinds)
        # STOP's own frames, sent inside stop(): the lake finish zeros, then STOP's
        stop_msgs = [as_main_firmware(p) for p in self.raw[n_before:]]
        stop_kinds = [m.WhichOneof('payload') for m in stop_msgs]
        self.assertEqual(stop_kinds, ['motor', 'steer', 'motor', 'steer', 'winch'])
        for m in stop_msgs:
            k = m.WhichOneof('payload')
            if k == 'motor':
                self.assertEqual((m.motor.left, m.motor.right), (0.0, 0.0))
            elif k == 'steer':
                self.assertEqual((m.steer.left, m.steer.right), (0.0, 0.0))
            else:
                self.assertEqual(m.winch.speed, 0.0)
        self.assertTrue(self.link.armed_cmd)              # no bench: lake STOP is not DISARM

    def test_a_complete_lake_run_speaks_only_main_payloads(self):
        r = self._run_full()
        self.assertEqual(r['status'], 'complete')
        kinds = self.main_kinds()
        self.assertNotIn('bench', kinds)
        self.assertTrue(set(kinds) <= {'motor', 'steer', 'winch'}, kinds)
        self.assertTrue(kinds, 'the finish sent nothing')

    def test_base10_and_every_bench_request_is_refused_while_the_lake_test_runs(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(3.0)
        for kind in ('both_long', 'both', 'left', 'right'):
            ok, err = self.link.send_bench(kind, 0.2, 0.0, self.link.winch_command_seq + 1)
            self.assertFalse(ok, kind)
        self.assertNotIn('bench', self.main_kinds())
        self.assertIsNotNone(self.link.lake_id)

    def test_the_tool_has_no_bench_abort_sender_left(self):
        src = (ROOT / 'tools' / 'espnow_drive.py').read_text()
        self.assertNotIn('_send_bench_abort_locked', src)
        self.assertNotIn('bench.abort = True', src)


if __name__ == '__main__':
    unittest.main()
