import math
import unittest

from docking_preview import PreviewSupervisor, acquisition_events, candidate, fault_cases


class DockingPreviewTest(unittest.TestCase):
    def test_waits_past_five_seconds_and_requires_fresh_post_stop_samples(self):
        s = PreviewSupervisor(accuracy_validated=True)
        s.tick(20.)
        self.assertEqual(s.state, 'ACQUIRING')
        for i in range(6):
            t = 20.+i*.2
            s.sample(t, 'A', 1.2, .01, 0., True)
            s.tick(t)
        self.assertEqual(s.state, 'STOPPING_YOLO')
        s.tick(21.2, detector_disabled=True)
        self.assertEqual(s.state, 'VERIFYING_RGBD')
        self.assertFalse(s.samples)
        for i in range(4):
            t = 21.4+i*.2
            s.sample(t, 'A', 1.2, .01, 0., True)
            s.tick(t)
        self.assertEqual(s.state, 'READY')
        self.assertTrue(s.frozen)

    def test_duplicate_motion_target_switch_and_cancel(self):
        s = PreviewSupervisor()
        s.sample(1., 'A', 1., 0., 0., True)
        s.sample(1., 'A', 1., 0., 0., True)
        self.assertEqual(len(s.samples), 1)
        s.sample(1.2, 'B', 1., 0., 0., True)
        self.assertEqual(len(s.samples), 1)
        s.sample(1.4, 'B', 1., 0., 0., True, stationary=False)
        self.assertEqual(len(s.samples), 0)
        s.tick(10., cancel=True)
        self.assertEqual(s.state, 'CANCELED')

    def test_absolute_accuracy_gate_blocks_stable_biased_pose(self):
        s = PreviewSupervisor(accuracy_validated=False)
        for i in range(12):
            t=8.+i*.2
            s.sample(t,'A',1.21,.005,0.,True)
            s.tick(t)
        self.assertEqual(s.state,'ACQUIRING')
        self.assertIn('absolute accuracy',s.reason)

    def test_diagnostic_keeps_pallet_measurements_frozen_mode_stops_them(self):
        diagnostic=PreviewSupervisor(accuracy_validated=True,mode='DIAGNOSTIC')
        frozen=PreviewSupervisor(accuracy_validated=True,mode='FROZEN_TARGET')
        for s in (diagnostic,frozen):
            s.state='READY';s.frozen=s.mode=='FROZEN_TARGET'
            self.assertTrue(s.start_motion())
        self.assertTrue(diagnostic.pallet_tracking)
        self.assertFalse(frozen.pallet_tracking)

    def test_synthetic_path_and_fault_latch(self):
        path = candidate(.05, 5., 1.2)
        self.assertTrue(path['illustrative_feasible'])
        self.assertTrue(math.isfinite(path['max_abs_kappa_1pm']))
        self.assertEqual(path['rows'][-1][1], 0.)
        self.assertGreater(path['rows'][-1][0], path['rows'][0][0])
        self.assertTrue(all(not case['auto_resume'] and not case['motor_command']
                            for case in fault_cases()))
        events = acquisition_events()
        self.assertEqual(events[50]['state'], 'ACQUIRING')
        self.assertTrue(any(e['state'] == 'READY' for e in events))
        self.assertTrue(all(not e['motor_command'] for e in events))


if __name__ == '__main__':
    unittest.main()
