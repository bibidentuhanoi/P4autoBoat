"""Lake ID can record a run with the raw diagnostic firmware."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_lake_id import LakeBase, T


class RawLakeTest(LakeBase):
    def _boat(self, *args, **kwargs):
        kwargs.setdefault('assist_p', False)
        super()._boat(*args, **kwargs)

    def setUp(self):
        super().setUp()
        self.link.raw_throttle_test = True
        self.link.p_assist_on = False
        self._boat(0.0, 0.0, 0.0, assist_p=False)

    def test_lake_run_accepts_p_off(self):
        ok, error = self._start()
        self.assertTrue(ok, error)
        self.clock.advance(2.0)
        self._all_fresh()
        self._tick()
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertEqual(self.link.throttle, 0.2)
        self.link._abort_lake_id_locked('test complete')
        self._wait_result()

    def test_full_raw_run_records_p_off_and_raw_metadata(self):
        result = self._run_full()
        rows, _events, summary = self._files(result['name'])
        self.assertEqual(summary['status'], 'complete')
        self.assertTrue(summary['settings']['raw_throttle_test'])
        self.assertEqual(summary['mode']['motor_p'], 'OFF')
        self.assertEqual(summary['provenance']['firmware_label'], 'raw-throttle-test')
        self.assertTrue(any(float(row['cmd_throttle']) == 0.2 for row in rows))
        self.assertTrue(all(row['boat_assist_motor_p'] == '0' for row in rows))

    def test_p_on_telemetry_aborts_powered_raw_run(self):
        ok, error = self._start()
        self.assertTrue(ok, error)
        self._drive(3.0)
        self._boat(0.2, 0.2, 0.0, assist_p=True)
        self._tick()
        result = self._wait_result()
        _rows, _events, summary = self._files(result['name'])
        self.assertIn('must stay OFF', summary['reason'])

    def test_raw_mode_rejects_enabling_motor_p(self):
        ok, error = self.link._send_assist_locked(True, False)
        self.assertFalse(ok)
        self.assertIn('requires Motor P', error)
        self.assertFalse(self.link.p_assist_on)
        ok, error = self.link._send_assist_locked(False, False)
        self.assertTrue(ok, error)
        self.assertFalse(self.link.p_assist_on)

    def test_raw_run_refuses_motor_p_on(self):
        self.link.p_assist_on = True
        ok, error = self._start()
        self.assertFalse(ok)
        self.assertIn('Motor P must be OFF', error)
        self.assertIsNone(self.link.lake_id)
