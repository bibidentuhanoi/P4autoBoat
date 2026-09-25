"""STOP must be understood by the firmware the boat is actually running.

The boat runs MAIN (c8a4c5e): the working firmware, the one every change is
checked against.  The same frames are also checked against the older main
9543ed1, which is stricter: its BenchCommand has no `abort` field, and a
BoatMessage carrying a bench payload -- even an empty one -- is dispatched by
its pipeline as a bench START with kind=0/base=0; bench_start() clamps a
zero base rather than refusing it. A STOP that put BenchCommand.abort on the
wire would therefore, on that firmware, start a 3 s zero-throttle BASE run:
boat busy, junk SD and laptop files, sticks ignored.

So ordinary and lake-test STOP send exactly what both understand -- motor
zeros, centred steer, winch zero, immediately -- and a bench the tool believes
is running or was just requested is stopped by DISARM, which every firmware
version honours.  Every STOP also carries one MissionCommand.stop (the mission
firmware's; it reaches a mission the tool lost track of): neither main has
such a field, both decode an empty message and their pipelines drop it
("Unhandled message type") -- checked frame by frame and in their source.

Each firmware's schema is taken from git at test time and loaded into a
private descriptor pool, so these tests decode the frames exactly as THAT
firmware reads them.
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
MAIN_COMMIT = 'c8a4c5e'          # main: the working firmware on the boat
OLDER_MAIN_COMMIT = '9543ed1'    # the older main, no BenchCommand.abort

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_lake_id import LakeBase, T            # noqa: E402


def _git_show(commit, path):
    r = subprocess.run(['git', '-C', str(ROOT), 'show', '%s:%s' % (commit, path)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise AssertionError('cannot read %s:%s from git: %s' % (commit, path, r.stderr))
    return r.stdout


def _schema_pool(commit):
    src = _git_show(commit, 'proto/boat_pb2.py')
    m = re.search(r"AddSerializedFile\((b'(?:[^'\\]|\\.)*')\)", src)
    if not m:
        raise AssertionError('serialized descriptor not found in the %s boat_pb2.py' % commit)
    pool = descriptor_pool.DescriptorPool()
    pool.AddSerializedFile(ast.literal_eval(m.group(1)))
    return pool


POOLS = {c: _schema_pool(c) for c in (MAIN_COMMIT, OLDER_MAIN_COMMIT)}
MESSAGES = {c: message_factory.GetMessageClass(pool.FindMessageTypeByName('boat.BoatMessage'))
            for c, pool in POOLS.items()}


def as_firmware(payload, commit):
    """Decode one wire payload the way that firmware does."""
    m = MESSAGES[commit]()
    m.ParseFromString(payload)
    return m


class FirmwareSchemaTest(unittest.TestCase):
    """Sanity for the proof itself: the pools really are those firmwares'."""

    def test_only_the_older_main_lacks_bench_abort(self):
        older = POOLS[OLDER_MAIN_COMMIT].FindMessageTypeByName('boat.BenchCommand')
        self.assertNotIn('abort', older.fields_by_name)
        main = POOLS[MAIN_COMMIT].FindMessageTypeByName('boat.BenchCommand')
        self.assertIn('abort', main.fields_by_name)
        self.assertIn('abort', T.load_boat_pb2().BenchCommand.DESCRIPTOR.fields_by_name)

    def test_neither_main_knows_the_mission_and_both_drop_what_they_do_not_know(self):
        for commit in (MAIN_COMMIT, OLDER_MAIN_COMMIT):
            fields = POOLS[commit].FindMessageTypeByName('boat.BoatMessage').fields_by_name
            self.assertNotIn('mission', fields, commit)
            pipeline = _git_show(commit, 'main/pipeline.c')
            switch = pipeline[pipeline.index('switch (s_rx_msg.which_payload)'):]
            default = switch[switch.index('default:'):]
            default = default[:default.index('break;')]
            self.assertIn('Unhandled message type', default, commit)
            self.assertNotIn('handler', default, commit)      # logged, never dispatched


class _RawWire(LakeBase):
    """LakeBase, plus every raw payload the tool writes, in order."""

    FIRMWARE = MAIN_COMMIT

    def setUp(self):
        super().setUp()
        self.raw = []
        inner = self.link._write_locked

        def capture(payload):
            self.raw.append(bytes(payload))
            return inner(payload)
        self.link._write_locked = capture

    def as_main_firmware(self, payload):
        return as_firmware(payload, self.FIRMWARE)

    def main_kinds(self):
        """What the firmware makes of each frame.  Every STOP also carries one
        MissionCommand.stop (it reaches a mission the tool lost track of);
        main has no such field, decodes an EMPTY message and its pipeline
        drops it (default: "Unhandled message type") -- 'ignored' here, and
        only ever that exact frame."""
        kinds = []
        for p in self.raw:
            old = self.as_main_firmware(p)
            kind = old.WhichOneof('payload')
            if kind is None:
                self.assertEqual(old.ListFields(), [])          # nothing main can act on
                new = T.load_boat_pb2().BoatMessage()
                new.ParseFromString(p)
                self.assertEqual(new.WhichOneof('payload'), 'mission')
                self.assertTrue(new.mission.stop and not new.mission.start)
                kind = 'ignored'
            kinds.append(kind)
        return kinds


class ManualStopOnMainFirmwareTest(_RawWire):

    def test_manual_stop_sends_zeros_immediately_and_no_bench_payload(self):
        self.link.throttle = 0.4; self.link.rudder = -0.3
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        msgs = [self.as_main_firmware(p) for p in self.raw]
        kinds = self.main_kinds()
        self.assertNotIn('bench', kinds)
        self.assertEqual(kinds, ['motor', 'steer', 'winch', 'ignored'])
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
        msgs = [self.as_main_firmware(p) for p in self.raw]
        kinds = self.main_kinds()
        self.assertNotIn('bench', kinds)
        self.assertEqual(kinds, ['motor', 'steer', 'winch', 'ignored', 'arm_cmd'])
        self.assertFalse(msgs[4].arm_cmd.arm)
        self.assertFalse(self.link.armed_cmd)

    def test_stop_disarms_a_bench_run_just_requested(self):
        """Between the request and the boat's first BenchStatus the run is, as
        far as this tool can know, about to be live."""
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        self.raw.clear()
        ok, err = self.link.stop(self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch', 'ignored', 'arm_cmd'])
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
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch', 'ignored'])
        self.assertTrue(self.link.armed_cmd)
        # a request older than the pending window, never answered
        self.link.bench_requested_at = time.monotonic() - T.BENCH_REQUEST_PENDING_S - 0.1
        self.raw.clear()
        self.link.stop(self.link.winch_command_seq + 1)
        self.assertEqual(self.main_kinds(), ['motor', 'steer', 'winch', 'ignored'])
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
        self.assertTrue(set(kinds) <= {'motor', 'steer', 'winch', 'ignored'}, kinds)
        # STOP's own frames, sent inside stop(): the lake finish zeros, then STOP's
        stop_msgs = [self.as_main_firmware(p) for p in self.raw[n_before:]]
        stop_kinds = kinds[n_before:]
        self.assertEqual(stop_kinds, ['motor', 'steer', 'motor', 'steer', 'winch', 'ignored'])
        for m in stop_msgs:
            k = m.WhichOneof('payload')
            if k is None:
                continue                                  # the ignored mission stop
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


class ManualStopOnOlderMainTest(ManualStopOnMainFirmwareTest):
    FIRMWARE = OLDER_MAIN_COMMIT


class LakeStopOnOlderMainTest(LakeStopOnMainFirmwareTest):
    FIRMWARE = OLDER_MAIN_COMMIT


if __name__ == '__main__':
    unittest.main()
