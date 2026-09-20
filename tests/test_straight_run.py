"""STRAIGHT 3 s: the lake machinery running a straight-only profile.

precheck 2 s -> straight 3 s at the chosen throttle, rudder centred -> stop
5 s. Same gate, stream, abort table and recorder as the lake steering test;
its own folder name, its own summary, and Motor P not forced.
"""
import csv
import functools
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_lake_id import LakeBase, T  # noqa: E402


class StraightProfileTest(unittest.TestCase):

    def test_profile_is_precheck_straight_stop_10s(self):
        ph = T.lake_id_phases(0.2, 0.0, 'NA', profile='straight')
        self.assertEqual([p[0] for p in ph], ['precheck', 'straight', 'stop'])
        self.assertEqual(sum(p[1] for p in ph), 10.0)
        self.assertEqual(sum(p[1] for p in ph if p[2] > 0), 3.0)
        self.assertEqual((ph[1][2], ph[1][3]), (0.2, 0.0))
        self.assertEqual(T.lake_id_profile_s(ph), 10.0)
        self.assertEqual(T.lake_id_powered_s(ph), 3.0)

    def test_full_profile_is_the_default_and_unchanged(self):
        ph = T.lake_id_phases(0.2, 0.3, 'LR')
        self.assertEqual(len(ph), 7)
        self.assertEqual(T.lake_id_profile_s(ph), 57.0)
        self.assertEqual(T.lake_id_powered_s(ph), 50.0)
        self.assertEqual(ph, T.lake_id_phases(0.2, 0.3, 'LR', profile='full'))

    def test_long_straight_profile_keeps_the_same_flow_but_drives_30s(self):
        ph = T.lake_id_phases(0.4, 0.0, 'NA', profile='straight30')
        self.assertEqual([p[0] for p in ph], ['precheck', 'straight', 'stop'])
        self.assertEqual(T.lake_id_profile_s(ph), 37.0)
        self.assertEqual(T.lake_id_powered_s(ph), 30.0)
        self.assertEqual((ph[1][2], ph[1][3]), (0.4, 0.0))


class StraightRunTest(LakeBase):

    def _start_straight(self, throttle=0.20, seq=None, **kw):
        seq = self.link.winch_command_seq + 1 if seq is None else seq
        return self.link.start_lake_id(throttle, 0.0, seq, profile='straight', **kw)

    def _run_straight(self, throttle=0.20):
        ok, err = self._start_straight(throttle)
        self.assertTrue(ok, err)
        self._drive(2.5)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._drive(9.0)
        return self._wait_result()

    def test_full_run_completes_in_its_own_folder(self):
        r = self._run_straight()
        self.assertEqual(r['status'], 'complete', r.get('reason'))
        self.assertEqual(r['name'], 'STRAIGHT_T20_001')
        self.assertEqual(r['profile'], 'straight')
        rows, events, summary = self._files(r['name'])
        phases = [row['phase'] for row in rows]
        self.assertIn('straight', phases)
        self.assertNotIn('turn_a', phases)
        straight_rows = [row for row in rows if row['phase'] == 'straight']
        span = float(straight_rows[-1]['elapsed_s']) - float(straight_rows[0]['elapsed_s'])
        self.assertGreater(span, 2.5)
        self.assertLess(span, 3.1)
        self.assertEqual(summary['schema'], 'straight_run_summary_v1')
        self.assertEqual(summary['settings']['profile'], 'straight')
        self.assertEqual(summary['settings']['profile_s'], 10.0)
        self.assertEqual(summary['settings']['powered_s'], 3.0)
        self.assertEqual(len(summary['straight']['yaw_bins']), 6)
        self.assertIn(summary['straight']['dominant_side'], ('left', 'right', 'none'))
        self.assertIn('shape', summary['straight'])
        self.assertEqual(summary['mode']['motor_p'], 'ON')
        self.assertNotIn('turn_a', summary)
        self.assertTrue(any(e['event'] == 'complete' for e in events))

    def test_long_run_summary_covers_the_full_30_second_window(self):
        seq = self.link.winch_command_seq + 1
        ok, err = self.link.start_lake_id(0.40, 0.0, seq, profile='straight30')
        self.assertTrue(ok, err)
        self._drive(38.0)
        r = self._wait_result()
        self.assertEqual(r['status'], 'complete', r.get('reason'))
        _rows, _events, summary = self._files(r['name'])
        self.assertEqual(summary['settings']['profile'], 'straight30')
        self.assertEqual(summary['settings']['profile_s'], 37.0)
        self.assertEqual(summary['settings']['powered_s'], 30.0)
        self.assertEqual(len(summary['straight']['yaw_bins']), 60)

    def test_straight_phase_sends_the_throttle_with_rudder_centred(self):
        ok, err = self._start_straight(0.30)
        self.assertTrue(ok, err)
        self._drive(2.5)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertEqual(self.link.throttle, 0.3)
        self.assertEqual(self.link.rudder, 0.0)
        self.assertTrue(self.link.motor_split)
        self.assertEqual((self.link.motor_left, self.link.motor_right), (0.3, 0.3))

    def test_start_waits_until_the_boat_confirms_the_requested_p_mode(self):
        ok, err = self.link.send_assist(True)
        self.assertTrue(ok, err)
        req = self.link._assist_off_req_id
        self.link.motor_status = dict(
            self.link.motor_status, have=True, assist_motor_p=False,
            assist_rudder=False, assist_request_id=req,
            last_rx_monotonic=self.clock.t)

        ok, err = self._start_straight(0.30)
        self.assertFalse(ok)
        self.assertIn('confirm Motor P ON', err)
        self.assertIsNone(self.link.lake_id)

        self.link.motor_status['assist_motor_p'] = True
        with self.link._lock:
            self.link._assist_off_tick_locked(self.clock.t)
        ok, err = self._start_straight(0.30)
        self.assertTrue(ok, err)

    def test_runs_are_numbered_per_throttle(self):
        self.assertEqual(self._run_straight()['name'], 'STRAIGHT_T20_001')
        self.assertEqual(self._run_straight()['name'], 'STRAIGHT_T20_002')
        self.assertEqual(self._run_straight(0.30)['name'], 'STRAIGHT_T30_001')
        # the bench card's throttle box, not the lake test's two-value whitelist
        self.assertEqual(self._run_straight(0.40)['name'], 'STRAIGHT_T40_001')

    def test_bench_throttle_range_is_accepted_and_the_rest_refused(self):
        for thr in (0.05, 0.25, 0.60):
            ok, err = self._start_straight(thr)
            self.assertTrue(ok, (thr, err))
            self.assertEqual(self.link.lake_id['throttle'], thr)
            with self.link._lock:
                self.link._abort_lake_id_locked('test done')
            self._wait_result()
        for thr in (0.0, 0.04, 0.61, 1.0, 'x'):
            ok, err = self._start_straight(thr)
            self.assertFalse(ok, thr)
            self.assertIsNone(self.link.lake_id)
        # the lake profile keeps its whitelist
        ok, err = self._start(throttle=0.40)
        self.assertFalse(ok)

    def test_scan_hands_out_the_next_free_index_and_leaves_the_lake_scan_alone(self):
        (self.tmp / 'STRAIGHT_T20_003').mkdir()
        (self.tmp / 'STRAIGHT_T45_002').mkdir()          # any percent, not only T20/T30
        n = T.straight_run_scan(self.tmp)
        self.assertEqual(n['STRAIGHT_T20']['next_index'], 4)
        self.assertEqual(n['STRAIGHT_T30']['next_index'], 1)
        self.assertEqual(n['STRAIGHT_T45']['next_index'], 3)
        self.link._lake_id_refresh_next()
        nxt = self.link.lake_id_next()
        self.assertEqual(nxt['STRAIGHT_T20']['next_index'], 4)
        self.assertEqual(nxt['T20_M30']['next_index'], 1)
        self.assertEqual(nxt['T20_M30']['next_order'], 'LR')

    def _boat_reports_p(self, on):
        # _drive() re-reports the boat every tick with LakeBase._boat's default
        # (P ON); pin the simulated boat's P flag for the whole test instead.
        self._boat = functools.partial(LakeBase._boat, self, assist_p=on)

    def test_motor_p_off_is_accepted_and_recorded(self):
        self.link.p_assist_on = False
        self._boat_reports_p(False)
        self._boat(0.0, 0.0, 0.0)
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        self._drive(2.5)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertIs(self.link.lake_id['p_at_start'], False)
        with self.link._lock:
            self.link._abort_lake_id_locked('test done')
        r = self._wait_result()
        _rows, _events, summary = self._files(r['name'])
        self.assertEqual(summary['mode']['motor_p'], 'OFF')

    def test_the_lake_profile_still_insists_on_motor_p_on(self):
        self.link.p_assist_on = False
        self._boat(0.0, 0.0, 0.0, assist_p=False)
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('Motor P must be ON', err)
        self.assertIsNone(self.link.lake_id)

    def test_motor_p_disagreement_at_the_gate_aborts(self):
        self.link.p_assist_on = True
        self._boat_reports_p(False)
        self._boat(0.0, 0.0, 0.0)
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        self._drive(2.5)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('motor_p_consistent', r['reason'])

    def test_motor_p_flip_mid_run_aborts(self):
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        self._drive(4.0)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._boat(0.18, 0.22, 0.0, assist_p=False)
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('Motor P flipped', r['reason'])

    def test_rail_loss_mid_run_is_recorded_without_ending_motor_test(self):
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        self._drive(4.0)
        self._boat(0.0, 0.0, 0.0, servo=False)
        self._tick()
        self.assertIsNotNone(self.link.lake_id)
        self._writer_idle()
        events = list(csv.DictReader(open(self.tmp / self.link.lake_id['name'] / 'events.csv')))
        self.assertTrue(any(e['event'] == 'boat_failsafe' for e in events))

    def test_manual_input_aborts(self):
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        self._drive(4.0)
        self.link.set_state(throttle=0.5)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('manual control input', r['reason'])
        self.assertEqual(self.link.throttle, 0.0)

    def test_status_and_result_carry_the_profile(self):
        ok, err = self._start_straight()
        self.assertTrue(ok, err)
        with self.link._lock:
            s = self.link.lake_id_status_locked()
        self.assertEqual(s['profile'], 'straight')
        self.assertEqual(s['profile_s'], 10.0)
        r = self._run_straight_from_started()
        for k in ('shape', 'dominant_side', 'total_turn_deg'):
            self.assertIn(k, r)

    def _run_straight_from_started(self):
        self._drive(2.5)
        self._drive(9.0)
        return self._wait_result()

    def test_unknown_profile_is_refused(self):
        ok, err = self.link.start_lake_id(0.2, 0.0, self.link.winch_command_seq + 1,
                                          profile='zigzag')
        self.assertFalse(ok)
        self.assertIsNone(self.link.lake_id)


class ShapeTest(unittest.TestCase):
    """The shape label answers 'quiet then turns, turning from the start,
    reverses, or straight'. Rows are 20 Hz over a (2.0, 12.0) window."""

    WINDOW = (2.0, 12.0)

    def _rows(self, yaw_of_t):
        rows = []
        n = 0
        t = 2.0
        while t < 12.0:
            rows.append({'elapsed_s': round(t, 4), 'yaw_dps': yaw_of_t(t - 2.0),
                         'heading_deg': 90.0, 'boat_applied_left_cmd': 0.17,
                         'boat_applied_right_cmd': 0.23})
            n += 1
            t = 2.0 + n * 0.05
        return rows

    def test_straight_all_run(self):
        a = T.straight_run_analyze(self._rows(lambda t: 0.4), self.WINDOW)
        self.assertEqual(a['shape'], 'straight all run')
        self.assertEqual(a['dominant_side'], 'none')
        self.assertEqual(len(a['yaw_bins']), 20)
        self.assertAlmostEqual(a['yaw_bins'][0]['yaw_dps'], 0.4, places=3)

    def test_quiet_then_turning_right(self):
        a = T.straight_run_analyze(self._rows(lambda t: 0.0 if t < 1.5 else -15.0), self.WINDOW)
        self.assertEqual(a['shape'], 'quiet at start -> turning R by the end')
        self.assertEqual(a['dominant_side'], 'right')
        self.assertLess(a['total_turn_deg'], -100.0)
        self.assertLess(a['peak_right_dps'], -14.0)

    def test_turning_right_from_the_start(self):
        a = T.straight_run_analyze(self._rows(lambda t: -12.0), self.WINDOW)
        self.assertEqual(a['shape'], 'steady turn R')
        self.assertEqual(a['dominant_side'], 'right')
        self.assertGreater(a['time_right_pct'], 95.0)

    def test_reverses(self):
        a = T.straight_run_analyze(self._rows(lambda t: 10.0 if t < 5.0 else -10.0), self.WINDOW)
        self.assertEqual(a['shape'], 'REVERSES L -> R')
        self.assertEqual(a['dominant_side'], 'none')

    def test_no_rows_is_reported_not_crashed(self):
        a = T.straight_run_analyze([], self.WINDOW)
        self.assertEqual(a['shape'], 'no data')
        self.assertEqual(a['dominant_side'], 'none')
        self.assertIsNone(a['total_turn_deg'])

    def test_controller_summary_uses_only_the_rows_it_is_given(self):
        rows = [
            {'yaw_dps': -2.0, 'heading_deg': 359.0, 'ctrl_active': 1,
             'saturated': 0, 'p_term': 0.10, 'i_term': 0.05},
            {'yaw_dps': 1.0, 'heading_deg': 0.0, 'ctrl_active': 1,
             'saturated': 1, 'p_term': -0.40, 'i_term': 0.10},
            {'yaw_dps': 3.0, 'heading_deg': 2.0, 'ctrl_active': 0,
             'saturated': 0, 'p_term': 0.20, 'i_term': 0.08},
        ]
        got = T.yaw_controller_summary(rows)
        self.assertAlmostEqual(got['heading_change_deg'], 3.0)
        self.assertAlmostEqual(got['mean_yaw_dps'], 2.0 / 3.0, places=4)
        self.assertAlmostEqual(got['peak_yaw_dps'], 3.0)
        self.assertAlmostEqual(got['active_fraction'], 2.0 / 3.0, places=4)
        self.assertAlmostEqual(got['saturated_fraction'], 1.0 / 3.0, places=4)
        self.assertAlmostEqual(got['peak_abs_p'], 0.40)
        self.assertAlmostEqual(got['peak_abs_i'], 0.10)
        self.assertAlmostEqual(got['final_i'], 0.08)

    def test_controller_summary_does_not_invent_missing_metrics(self):
        got = T.yaw_controller_summary([])
        for name in ('heading_change_deg', 'mean_yaw_dps', 'peak_yaw_dps',
                     'active_fraction', 'saturated_fraction', 'peak_abs_p',
                     'peak_abs_i', 'final_i'):
            self.assertIsNone(got[name], name)


if __name__ == '__main__':
    unittest.main()


class StraightRecordingTest(LakeBase):
    """'Record everything meaningful': besides the boat's telemetry, every row
    carries the link (bridge counters, RSSI), the tool's own send timing and
    the boat's last known learned c, and the summary reduces them."""

    def _bridge(self, rssi=-60, pkts=100, frames=50, drops=0):
        self.link.bridge_status = dict(self.link.bridge_status, have=True, uplink_rssi_dbm=rssi,
                                       espnow_pkts=pkts, frames_out=frames, reasm_drops=drops,
                                       last_rx_monotonic=self.clock.t)

    def _bench_c(self, c=0.171, **diagnostic):
        self.link.bench_status = dict(self.link.bench_status, have=True, learn_c=c,
                                      last_rx_monotonic=self.clock.t - 30.0,
                                      **diagnostic)

    def test_rows_carry_link_tool_timing_and_learned_c(self):
        self._bridge()
        self._bench_c()
        self.link._last_send_mono = self.clock.t - 0.05
        ok, err = self.link.start_lake_id(0.2, 0.0, self.link.winch_command_seq + 1, profile='straight')
        self.assertTrue(ok, err)
        self._drive(2.5)
        self._bridge(rssi=-70, pkts=160, frames=80, drops=1)
        self.link._last_send_mono = self.clock.t - 0.02
        self._frame(yaw=0.0, heading=90.0)
        with self.link._lock:
            self.link._abort_lake_id_locked('test done')
        r = self._wait_result()
        rows, _events, summary = self._files(r['name'])
        last = rows[-1]
        for c in ('tool_send_age_s', 'bridge_age_s', 'bridge_uplink_rssi_dbm', 'bridge_espnow_pkts',
                  'bridge_frames_out', 'bridge_reasm_drops', 'bench_learn_c_last', 'bench_status_age_s'):
            self.assertIn(c, last)
        self.assertEqual(last['bridge_uplink_rssi_dbm'], '-70')
        self.assertEqual(last['bridge_espnow_pkts'], '160')
        self.assertAlmostEqual(float(last['tool_send_age_s']), 0.02, places=3)
        self.assertEqual(last['bench_learn_c_last'], '0.171')
        self.assertGreater(float(last['bench_status_age_s']), 30.0)
        link = summary['link']
        self.assertEqual(link['bridge_rssi_min_dbm'], -70)
        self.assertEqual(link['bridge_rssi_max_dbm'], -60)
        self.assertEqual(link['bridge_espnow_pkts_delta'], 60)
        self.assertEqual(link['bridge_frames_out_delta'], 30)
        self.assertEqual(link['bridge_reasm_drops_delta'], 1)
        self.assertGreaterEqual(link['tool_send_age_max_s'], 0.05)

    def test_rows_and_summary_carry_yaw_controller_diagnostics(self):
        diagnostic = {
            'heading_target_deg': 91.0, 'heading_error_deg': 1.0,
            'yaw_target_dps': 0.8, 'p_term': 0.12, 'i_term': 0.04,
            'dynamic_c': 0.16, 'effective_c': 0.20, 'c_limit': 1.0,
            'ctrl_active': True, 'heading_hold': True, 'saturated': False,
        }
        self._bench_c()
        self.link.motor_status = dict(
            self.link.motor_status,
            have=True,
            last_rx_monotonic=self.clock.t,
            motor_yaw_target_dps=diagnostic['yaw_target_dps'],
            motor_yaw_filt_dps=0.3,
            **{name: value for name, value in diagnostic.items()
               if name != 'yaw_target_dps'},
        )
        ok, err = self.link.start_lake_id(
            0.2, 0.0, self.link.winch_command_seq + 1, profile='straight')
        self.assertTrue(ok, err)
        self._drive(2.5)
        with self.link._lock:
            self.link._abort_lake_id_locked('test done')
        result = self._wait_result()
        rows, _events, summary = self._files(result['name'])
        drive = [r for r in rows if r['phase'] == 'straight']
        self.assertTrue(drive)
        for name, expected in diagnostic.items():
            want = 1 if expected is True else 0 if expected is False else expected
            self.assertAlmostEqual(float(drive[-1][name]), want, places=4, msg=name)
        controller = summary['straight']['controller']
        self.assertAlmostEqual(controller['active_fraction'], 1.0)
        self.assertAlmostEqual(controller['saturated_fraction'], 0.0)
        self.assertAlmostEqual(controller['peak_abs_p'], 0.12)
        self.assertAlmostEqual(controller['peak_abs_i'], 0.04)
        self.assertAlmostEqual(controller['final_i'], 0.04)

    def test_the_lake_profile_gets_the_same_columns_and_link_section(self):
        self._bridge()
        ok, err = self._start()
        self.assertTrue(ok, err)
        self._drive(2.5)
        with self.link._lock:
            self.link._abort_lake_id_locked('test done')
        r = self._wait_result()
        rows, _events, summary = self._files(r['name'])
        self.assertIn('tool_send_age_s', rows[0])
        self.assertEqual(rows[-1]['bridge_uplink_rssi_dbm'], '-60')
        self.assertEqual(summary['schema'], 'lake_id_summary_v2')
        self.assertIn('bridge_rssi_min_dbm', summary['link'])

    def test_stream_send_stamps_the_tool_send_time(self):
        self.link.throttle = 0.2
        self.link.rudder = 0.0
        self.sent.clear()
        with self.link._lock:
            self.link._stream_send_locked()
        self.assertEqual(self.link._last_send_mono, self.clock.t)
        self.assertIn(('motor', 0.2, 0.2), self.sent)
        self.assertIn(('steer', 0.0), self.sent)
