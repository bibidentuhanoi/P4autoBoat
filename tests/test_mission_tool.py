"""The laptop side of the out-and-back mission (tools/espnow_drive.py).

The BOAT flies the mission; the tool sends START (with the settings) and STOP,
shows what the boat reports, and records it. Run on the lake tests' fake-clock
link (tests/test_lake_id.py LakeBase): no serial port, every frame captured.
"""
import csv
import json
import threading
import time
import unittest
from pathlib import Path

from tests.test_lake_id import LakeBase, T
from tests.test_espnow_drive import run_page_js


class MissionBase(LakeBase):

    def setUp(self):
        super().setUp()
        link = self.link
        link.mission = None
        link.mission_result = None
        link._mission_writer = None
        link._mission_req_seq = 0
        link.mission_status = T.BoatLink._blank_mission_status()
        link.mission_dir = self.tmp
        link._mission_open = None
        link._mission_open_answer_by = None
        link._mission_stop_until = None
        self.frames = []

        def _write(payload):
            msg = link.pb2.BoatMessage()
            msg.ParseFromString(payload)
            kind = msg.WhichOneof('payload')
            self.frames.append((kind, getattr(msg, kind) if kind else None))
            return True
        link._write_locked = _write

    def kinds(self):
        return [k for k, _ in self.frames]

    def missions(self):
        return [m for k, m in self.frames if k == 'mission']

    def tick(self, dt=1.0 / 15.0):
        self.clock.advance(dt)
        with self.link._lock:
            self.link._mission_tick_locked(self.clock.t)

    def boat(self, **fields):
        """A MissionStatus from the boat, through the real frame decoder."""
        msg = self.link.pb2.BoatMessage()
        for k, v in fields.items():
            setattr(msg.mission_status, k, v)
        frame = T.build_frame(T.MSG_MISSION_STATUS, msg.SerializeToString(), seq=3)
        self.link._handle_incoming_frame(frame[:-1])

    def start(self, **settings):
        ok, err = self.link.start_mission(settings, command_seq=1)
        self.assertTrue(ok, err)
        return self.link.mission['request_id']

    def finished(self, timeout=5.0):
        """Wait for the off-lock writer to finish the summary."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.link.mission_result and list(self.tmp.glob('MISSION_*/summary.json')):
                return json.loads(next(self.tmp.glob('MISSION_*/summary.json')).read_text())
            time.sleep(0.02)
        self.fail('no summary.json')


class StartTest(MissionBase):

    def test_bad_settings_are_refused_before_anything_is_sent(self):
        for bad, words in (({'out_distance_m': 2.0}, 'out distance'),
                           ({'home_radius_m': 6.0}, 'twice'),
                           ({'throttle': 0.8}, 'throttle'),
                           ({'approach_throttle': 0.5}, 'approach'),
                           ({'stage': 4}, 'stage')):
            ok, err = self.link.start_mission(bad, command_seq=1)
            self.assertFalse(ok)
            self.assertIn(words, err)
        self.assertEqual(self.frames, [])

    def test_refused_while_another_test_or_a_boat_mission_runs(self):
        self.link.lake_id = {'dummy': True}
        ok, err = self.link.start_mission({}, 1)
        self.assertFalse(ok); self.assertIn('lake test', err)
        self.link.lake_id = None
        self.boat(run_id=4, request_id=99, state=3)            # the boat is mid-mission
        ok, err = self.link.start_mission({}, 1)
        self.assertFalse(ok); self.assertIn('STOP it first', err)

    def test_start_sends_the_settings_and_repeats_until_answered(self):
        self.link.throttle = 0.3; self.link.rudder = 0.2          # a stick left up
        rid = self.start(out_distance_m=12.0, home_radius_m=2.5, throttle=0.45,
                         approach_throttle=0.2, turn_right=False, stage=3)
        # presence only from now on: a non-zero stick would end the mission
        self.assertEqual((self.link.throttle, self.link.rudder), (0.0, 0.0))
        first = self.missions()[0]
        self.assertTrue(first.start and not first.stop)
        self.assertEqual((first.request_id, first.stage, round(first.out_distance_m, 2),
                          round(first.throttle, 2), first.turn_right), (rid, 3, 12.0, 0.45, False))
        for _ in range(8):                                        # ~0.5 s
            self.tick()
        self.assertEqual(len(self.missions()), 2)                 # repeated once
        self.assertEqual(self.missions()[1].request_id, rid)      # the SAME press
        self.boat(run_id=5, request_id=rid, state=1)              # HOME: answered
        n = len(self.missions())
        for _ in range(30):
            self.tick()
        self.assertEqual(len(self.missions()), n)                 # no more STARTs
        self.assertTrue(self.link.status()['mission']['run']['accepted'])

    def test_no_answer_ends_the_run_as_no_answer(self):
        self.start()
        for _ in range(int(T.MISSION_START_ANSWER_S * 15) + 3):
            self.tick()
        summary = self.finished()
        self.assertEqual(summary['status'], 'no_answer')
        self.assertIsNone(self.link.mission)

    def test_a_refusal_is_the_boats_reason(self):
        rid = self.start()
        self.boat(run_id=1, request_id=rid, state=7, reason=33)
        summary = self.finished()
        self.assertEqual((summary['status'], summary['reason']), ('refused', 'not armed'))


class RecordTest(MissionBase):

    def test_every_status_is_a_row_and_the_end_writes_the_summary(self):
        rid = self.start()
        for state, dist in ((1, 0.0), (2, 3.0), (2, 8.0), (3, 10.5), (4, 7.0), (4, 3.0)):
            self.clock.advance(0.2)
            self.boat(run_id=2, request_id=rid, state=state, dist_home_m=dist,
                      home_lat=21.04, home_lon=105.88, home_radius_m=2.5, out_distance_m=10.0)
        self.clock.advance(3.0)                                    # a radio gap
        self.boat(run_id=2, request_id=rid, state=5, reason=3, dist_home_m=1.8, closest_m=1.7,
                  file_index=7, record_state=3)
        summary = self.finished()
        self.assertEqual((summary['status'], summary['reason']), ('done', 'in the home zone'))
        self.assertEqual(summary['status_rows'], 7)
        self.assertEqual(summary['status_gaps_over_1s'], 1)
        self.assertEqual(summary['boat_record'], 'MSN_007.CSV on the boat SD card')
        run = next(self.tmp.glob('MISSION_FULL_T40_D10_*'))
        rows = list(csv.DictReader(l for l in open(run / 'samples.csv') if not l.startswith('#')))
        self.assertEqual([r['state_name'] for r in rows],
                         ['HOME', 'OUTBOUND', 'OUTBOUND', 'TURN', 'RETURN', 'RETURN', 'DONE'])
        self.assertAlmostEqual(float(rows[-1]['rx_gap_s']), 3.0, places=2)
        res = self.link.status()['mission']['result']
        self.assertEqual((res['status'], res['file_index']), ('done', 7))

    def test_the_next_run_takes_the_next_folder(self):
        for i in range(2):
            rid = self.start()
            self.boat(run_id=10 + i, request_id=rid, state=6, reason=10)
            self.finished()
            self.link.mission_result = None
        names = sorted(p.name for p in self.tmp.glob('MISSION_*'))
        self.assertEqual(names, ['MISSION_FULL_T40_D10_001', 'MISSION_FULL_T40_D10_002'])

    def test_merge_puts_the_boat_record_beside_the_laptops(self):
        rid = self.start()
        for k in range(5):
            self.clock.advance(0.2)
            self.boat(run_id=3, request_id=rid, state=2, elapsed_s=0.2 * k)
        self.boat(run_id=3, request_id=rid, state=6, reason=10, elapsed_s=1.0)
        self.finished()
        run = next(self.tmp.glob('MISSION_*'))
        boat_csv = self.tmp / 'MSN_003.CSV'
        with open(boat_csv, 'w') as fh:
            fh.write('# out-and-back mission run 3\nt_s,state,lat\n')
            for i in range(60):                              # 3 s at 20 Hz
                fh.write('%.2f,2,21.04\n' % (i * 0.05))
        merged = T.mission_merge(run, boat_csv)
        rows = list(csv.DictReader(open(merged)))
        self.assertEqual(len(rows), 60)                      # every boat row kept
        saw = [int(r['laptop_saw_status']) for r in rows]
        self.assertEqual(saw[0], 1)                          # 0.0 s: the laptop saw it
        self.assertEqual(saw[-1], 0)                         # 2.95 s: the laptop did not
        self.assertTrue((run / 'boat_MSN_003.CSV').exists())


class StopTest(MissionBase):

    def test_stop_sends_mission_stop_and_disarm_and_repeats_until_confirmed(self):
        rid = self.start()
        self.boat(run_id=6, request_id=rid, state=2)
        self.frames.clear()
        ok, err = self.link.stop(5)
        self.assertTrue(ok, err)
        kinds = self.kinds()
        self.assertIn('mission', kinds)
        self.assertTrue(self.missions()[0].stop)
        arm = [m for k, m in self.frames if k == 'arm_cmd']
        self.assertTrue(arm and not arm[0].arm)                  # DISARM as the backup
        self.assertLess(kinds.index('motor'), kinds.index('mission'))   # zeros first
        for _ in range(10):
            self.tick()
        self.assertGreaterEqual(len(self.missions()), 11)        # every tick, unconfirmed
        self.boat(run_id=6, request_id=rid, state=6, reason=10)
        n = len(self.missions())
        for _ in range(10):
            self.tick()
        self.assertEqual(len(self.missions()), n)                # confirmed: quiet
        summary = self.finished()
        self.assertEqual((summary['status'], summary['reason']), ('aborted', 'STOP'))

    def test_stop_reaches_a_mission_the_laptop_did_not_start(self):
        # A restarted tool: no local run, but the boat reports one flying.
        self.boat(run_id=8, request_id=12345, state=4)
        self.link.stop(9)
        self.assertTrue(self.missions() and self.missions()[0].stop)
        self.assertTrue(any(k == 'arm_cmd' for k in self.kinds()))

    def test_a_plain_stop_sends_one_mission_stop_and_never_disarms(self):
        """No mission known: the zeros, then ONE MissionCommand.stop -- what
        reaches a mission this tool lost track of, ignored by a boat with none
        -- and no DISARM, no repeats (a plain STOP stays a plain STOP)."""
        self.link.stop(3)
        self.assertEqual(self.kinds(), ['motor', 'steer', 'winch', 'mission'])
        self.assertTrue(self.missions()[0].stop and not self.missions()[0].start)
        for _ in range(10):
            self.tick()
        self.assertEqual(self.kinds(), ['motor', 'steer', 'winch', 'mission'])

    def test_stop_still_reaches_a_start_that_was_never_answered(self):
        """The boat took START but none of its reports arrive: the tool gives
        the run up as no_answer -- and the boat is still flying it."""
        rid = self.start()
        for _ in range(int(T.MISSION_START_ANSWER_S * 15) + 3):
            self.tick()
        self.assertEqual(self.finished()['status'], 'no_answer')
        self.frames.clear()
        ok, err = self.link.stop(5)
        self.assertTrue(ok, err)
        self.assertTrue(self.missions()[0].stop)
        arm = [m for k, m in self.frames if k == 'arm_cmd']
        self.assertTrue(arm and not arm[0].arm)                  # DISARM as the backup
        for _ in range(10):
            self.tick()
        self.assertGreaterEqual(len(self.missions()), 11)        # every tick, unconfirmed
        # the boat's report gets through at last: that run is over
        self.boat(run_id=3, request_id=rid, state=6, reason=10)
        n = len(self.missions())
        for _ in range(10):
            self.tick()
        self.assertEqual(len(self.missions()), n)                # confirmed: quiet
        self.frames.clear()
        self.link.stop(6)                                        # STOP is plain again
        self.assertEqual(self.kinds(), ['motor', 'steer', 'winch', 'mission'])

    def test_a_start_the_boat_never_took_closes_on_its_next_idle_report(self):
        rid = self.start()
        # inside the answer window an idle report proves nothing: START may be in flight
        self.boat(run_id=2, request_id=rid - 1, state=5, reason=1)
        self.assertEqual(self.link._mission_open, rid)
        for _ in range(int(T.MISSION_START_ANSWER_S * 15) + 3):
            self.tick()
        self.finished()
        self.assertEqual(self.link._mission_open, rid)           # no answer: still open
        # after it, the boat alive and idle on an older run: it never took ours
        self.boat(run_id=2, request_id=rid - 1, state=5, reason=1)
        self.assertIsNone(self.link._mission_open)
        self.frames.clear()
        self.link.stop(5)
        self.assertEqual(self.kinds(), ['motor', 'steer', 'winch', 'mission'])   # no DISARM

    def test_an_unconfirmed_stop_is_reported(self):
        rid = self.start()
        self.boat(run_id=6, request_id=rid, state=2)
        self.link.stop(5)
        for _ in range(int(T.MISSION_STOP_RETRY_S * 15) + 3):
            self.tick()
        summary = self.finished()
        self.assertEqual(summary['status'], 'stop_unconfirmed')


class ProgressTest(MissionBase):

    def test_no_progress_is_a_warning_never_a_stop(self):
        rid = self.start()
        self.boat(run_id=2, request_id=rid, state=4, dist_home_m=6.0)
        for _ in range(31):
            self.clock.advance(1.0)
            self.boat(run_id=2, request_id=rid, state=4, dist_home_m=5.8)   # creeping
        run = self.link.status()['mission']['run']
        self.assertTrue(run['no_progress'])
        self.assertIn('no progress', run['warnings'])
        self.assertNotIn('mission', [k for k in self.kinds()[1:]
                                     if k == 'mission' and self.missions()[-1].stop])
        self.clock.advance(1.0)
        self.boat(run_id=2, request_id=rid, state=4, dist_home_m=4.5)      # moving again
        self.assertFalse(self.link.status()['mission']['run']['no_progress'])


class PageTest(unittest.TestCase):

    def test_the_card_sits_under_the_lake_card_and_starts_with_its_settings(self):
        page = T.PAGE
        self.assertLess(page.index('id="lake-card"'), page.index('id="mission-card"'))
        self.assertLess(page.index('id="mission-card"'), page.index('id="map-panel"'))
        result = run_page_js(r"""
vm.createContext(context);
vm.runInContext(script, context);
context.applyStatus(Object.assign({}, CONNECTED_STATUS));   // how the page learns it is connected
const el = id => context.document.getElementById(id);
el('msn-stage').value = '2'; el('msn-out').value = '12';
el('msn-radius').value = '3'; el('msn-thr').value = '45';
el('msn-appr').value = '20'; el('msn-turn').value = 'left';
el('msn-dry').checked = true;
(async function () {
  await context.startMission();
  const post = calls.filter(c => c.path === '/api/mission')[0];
  const boat = { have: true, stale: true, age_s: 6.2, run_id: 3, request_id: 7, state: 4,
    state_name: 'RETURN', reason: 0, reason_text: '', dry_run: false, approach: true,
    elapsed_s: 51, dist_target_m: 4.2, bearing_target_deg: 200, dist_home_m: 4.2,
    cross_track_m: 0.6, turned_deg: 181, heading_deg: 197, wanted_heading_deg: 203,
    hold_active: true, p_switch: true, beta_deg: 12, beta_valid: true, compass_bad: false,
    sats: 12, pdop: 1.4, speed_acc_mps: 0.09, course_samples: 88, gps_outliers: 1,
    outages: 2, longest_outage_s: 7.5, home_lat: 0, home_lon: 0, home_radius_m: 3, out_distance_m: 12 };
  context.applyStatus(Object.assign({}, CONNECTED_STATUS, { mission: { boat: boat,
    run: { name: 'MISSION_TURN_T45_D12_001', accepted: true, stopping: false, no_progress: true,
           warnings: ['no progress'], rows: 250, recording_error: null, elapsed_s: 52 },
    result: null, defaults: {} } }));
  console.log(JSON.stringify({ post: post && post.body,
    pill: el('msn-pill').textContent, radio: el('msn-radio').textContent,
    step: el('msn-step').textContent, warn: el('msn-warn').textContent,
    start_disabled: el('msn-start').disabled,
    lake_disabled: el('lake-start').disabled }));
  process.exit(0);
})();
""")
        self.assertEqual(result.returncode, 0, result.stderr[-2000:])
        out = json.loads(result.stdout.strip().splitlines()[-1])
        s = out['post']['settings']
        self.assertEqual((s['stage'], s['out_distance_m'], s['home_radius_m'], s['throttle'],
                          s['approach_throttle'], s['turn_right'], s['dry_run']),
                         (2, 12, 3, 0.45, 0.2, False, True))
        self.assertEqual(out['pill'], 'RETURN')
        self.assertIn('no signal 6 s -- the boat continues on its own', out['radio'])
        self.assertIn('slow approach', out['step'])
        self.assertIn('NO PROGRESS', out['warn'])
        self.assertTrue(out['start_disabled'])
        self.assertTrue(out['lake_disabled'])                      # a mission locks the other tests


if __name__ == '__main__':
    unittest.main()


class StreamAndCliTest(MissionBase):

    def test_a_mission_streams_only_the_zero_presence(self):
        rid = self.start()
        self.boat(run_id=2, request_id=rid, state=2)
        self.frames.clear()
        with self.link._lock:
            self.link._stream_send_locked()
        self.assertEqual(self.kinds(), ['motor'])                 # no steer, no winch
        motor = self.frames[0][1]
        self.assertEqual((motor.left, motor.right), (0.0, 0.0))

    def test_the_merge_command_line(self):
        import subprocess
        import sys
        rid = self.start()
        self.boat(run_id=3, request_id=rid, state=6, reason=10, elapsed_s=0.0)
        self.finished()
        run = next(self.tmp.glob('MISSION_*'))
        boat_csv = self.tmp / 'MSN_004.CSV'
        boat_csv.write_text('# boat\nt_s,state\n0.00,2\n0.05,2\n')
        res = subprocess.run([sys.executable, str(T.__file__), '--merge-mission', str(run), str(boat_csv)],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(res.returncode, 0, res.stderr[-1000:])
        self.assertTrue((run / 'merged.csv').exists())


class InterlockTest(MissionBase):
    """Server-side refusals while a mission may own the jets (a disabled
    button is decoration: a stale tab or a curl reaches the API directly)."""

    def _fly(self):
        rid = self.start()
        self.boat(run_id=4, request_id=rid, state=2)             # OUTBOUND
        return rid

    def test_runs_and_mode_changes_are_refused_while_a_mission_runs(self):
        self._fly()
        seq = lambda: self.link.winch_command_seq + 1           # noqa: E731
        refusals = {
            'bench': self.link.send_bench('both', 0.2, 0.0, seq()),
            'calibration': self.link.set_calibrate(True, seq()),
            'lake test': self.link.start_lake_id(0.20, 0.30, seq()),
            'rudder test': self.link.start_rudder_test(1, seq()),
            'P on': self.link.send_assist(True),
            'P off': self.link.send_assist(False),
        }
        for what, (ok, err) in refusals.items():
            self.assertFalse(ok, what)
            self.assertIn('mission is running', err, what)
        self.assertFalse(self.link.calibrating)                 # no keepalives will go out
        # what the operator reaches for to END it is never refused
        self.assertTrue(self.link.set_calibrate(False, seq())[0])
        ok, err = self.link.set_winch(0.5, seq())
        self.assertNotIn('mission', err or '')

    def test_a_mission_the_boat_reports_blocks_them_too(self):
        # a restarted tool: no local run, the boat says one is flying
        self.boat(run_id=8, request_id=4242, state=4)
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertFalse(ok)
        self.assertIn('mission is running', err)

    def test_they_work_again_once_the_run_is_over(self):
        rid = self._fly()
        self.boat(run_id=4, request_id=rid, state=5, reason=3)  # DONE
        self.finished()
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)

    def test_a_start_on_firmware_without_missions_is_forgotten(self):
        """The boat answers MotorStatus but has never sent a MissionStatus:
        no mission firmware. Its START started nothing -- STOP is plain again
        and the bench is not blocked."""
        self.start()
        for _ in range(int(3 * T.MISSION_START_ANSWER_S * 15) + 3):
            self.link.motor_status = dict(self.link.motor_status, have=True,
                                          last_rx_monotonic=self.clock.t)
            self.tick()
        self.assertIsNone(self.link._mission_open)
        self.frames.clear()
        self.link.stop(self.link.winch_command_seq + 1)
        self.assertNotIn('arm_cmd', self.kinds())
        ok, err = self.link.send_bench('both', 0.2, 0.0, self.link.winch_command_seq + 1)
        self.assertTrue(ok, err)

    def test_a_silent_boat_keeps_the_start_open(self):
        """No MotorStatus either: the downlink is gone, not the firmware --
        the boat may be flying, so STOP still disarms."""
        self.start()
        for _ in range(int(3 * T.MISSION_START_ANSWER_S * 15) + 3):
            self.tick()
        self.assertIsNotNone(self.link._mission_open)

    def test_mission_firmware_that_goes_quiet_keeps_the_start_open(self):
        """It has reported missions before, so it HAS mission firmware: a START
        it never answered may be flying even though MotorStatus still comes."""
        self.boat(run_id=3, request_id=7, state=5, reason=3)     # an earlier run's report
        self.start()
        for _ in range(int(3 * T.MISSION_START_ANSWER_S * 15) + 3):
            self.link.motor_status = dict(self.link.motor_status, have=True,
                                          last_rx_monotonic=self.clock.t)
            self.tick()
        self.assertIsNotNone(self.link._mission_open)


class RecordNameTest(MissionBase):
    """The run folder names the boat's SD file even when the boat's first
    terminal report went out while the file was still being written."""

    def test_the_end_waits_for_the_file_name(self):
        rid = self.start()
        self.boat(run_id=4, request_id=rid, state=2)
        self.boat(run_id=4, request_id=rid, state=5, reason=3, record_state=2)   # DONE, saving
        self.assertIsNotNone(self.link.mission)                                   # not yet
        self.tick()
        self.boat(run_id=4, request_id=rid, state=5, reason=3, record_state=3, file_index=7)
        summary = self.finished()
        self.assertEqual(summary['status'], 'done')
        self.assertEqual(summary['boat_record'], 'MSN_007.CSV on the boat SD card')

    def test_a_record_never_confirmed_still_ends_the_run(self):
        rid = self.start()
        self.boat(run_id=4, request_id=rid, state=5, reason=3, record_state=2)
        for _ in range(int(T.MISSION_RECORD_WAIT_S * 15) + 3):
            self.tick()
        summary = self.finished()
        self.assertEqual(summary['status'], 'done')
        self.assertIsNone(summary['boat_record'])
        self.assertIn('boat record not confirmed', summary['warnings'])
