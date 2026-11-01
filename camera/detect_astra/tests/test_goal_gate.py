import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_goal_gate import RecognitionGoalGate


class GoalGateTest(unittest.TestCase):
    def test_only_new_observed_active_goal_can_trigger_once(self):
        g = RecognitionGoalGate(100.)
        for status in (1, 2, 4):
            self.assertFalse(g.update([('old', 99., status)]))
        self.assertFalse(g.update([('unobserved', 101., 4)]))
        self.assertFalse(g.update([('new', 102., 2)]))
        self.assertTrue(g.update([('new', 102., 4)]))
        self.assertFalse(g.update([('new', 102., 4)]))
        self.assertFalse(g.armed)

    def test_cancel_and_abort_require_rearm(self):
        for status in (5, 6):
            g = RecognitionGoalGate(100.)
            g.update([('a', 101., 2)])
            self.assertFalse(g.update([('a', 101., status)]))
            self.assertEqual(g.phase, 'GOAL_FAILED')
            self.assertFalse(g.update([('a', 101., 4)]))
            g.arm(103.)
            self.assertFalse(g.update([('b', 104., 1)]))
            self.assertTrue(g.update([('b', 104., 4)]))

    def test_preempted_goal_success_cannot_trigger_during_new_navigation(self):
        g = RecognitionGoalGate(100.)
        g.update([('a', 101., 2)])
        self.assertFalse(g.update([('a', 101., 4), ('b', 102., 2)]))
        self.assertTrue(g.navigation_active)
        self.assertEqual(g.goal_id, 'b')
        self.assertTrue(g.update([('a', 101., 4), ('b', 102., 4)]))
        self.assertFalse(g.update([('c', 103., 2)]))
        self.assertEqual(g.phase, 'WAITING_ARM')

    def test_canceling_never_confirms(self):
        g = RecognitionGoalGate(100.)
        self.assertFalse(g.update([('a', 101., 3)]))
        self.assertFalse(g.update([('a', 101., 4)]))


if __name__ == '__main__':
    unittest.main()
