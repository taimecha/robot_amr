"""Kinematic and watchdog tests: no ROS or robot hardware."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pallet_alignment_core import AlignmentConfig, AlignmentController, OdomHistory, OdomSample, wrap


class AlignmentTest(unittest.TestCase):
    def seed(self, c, forward=1.5655, left=0., heading=0.):
        for t in (.9, .95, 1.):
            c.odom.add(OdomSample(t, 0., 0., 0.))
            c.observe(t, t, forward, left, heading)

    def test_user_tolerances_and_goal_offset(self):
        c = AlignmentController()
        self.seed(c)
        self.assertEqual(c.config.lateral_tolerance_m, .02)
        self.assertEqual(c.config.standoff_m, 1.2)
        self.assertEqual(c.config.heading_tolerance_deg, 2.)
        c.tick(1., 1.)
        self.assertAlmostEqual(c.goal[0], 0.)
        self.assertAlmostEqual(c.errors(c.odom.samples[-1])[0], 0.)

    def test_strict_lateral_and_heading_boundaries(self):
        for y, yaw, settling in ((.0199, 1.99, True), (.02, 0, False), (0, 2., False)):
            c = AlignmentController()
            self.seed(c, left=y, heading=math.radians(yaw))
            self.assertTrue(c.start(1., 1.)[0])
            c.tick(1., 1.)
            self.assertEqual(c.state == 'SETTLING', settling)

    def test_stale_invalid_and_stop_zero(self):
        c = AlignmentController()
        self.seed(c, forward=2.)
        c.start(1., 1.)
        self.assertGreater(c.tick(1., 1.)['linear_mps'], 0)
        c.invalidate_target('Invalid depth')
        self.assertEqual((c.v, c.w), (0., 0.))
        c.tick(1.05, 1.05)
        self.assertEqual(c.state, 'WAIT_DATA')
        c.tick(1.85, 1.85)
        self.assertEqual(c.state, 'LOST_DATA')
        self.assertFalse(c.active)
        self.assertEqual((c.v, c.w), (0., 0.))
        c.stop()
        self.assertFalse(c.active)

    def test_duplicate_measurements_cannot_confirm(self):
        c = AlignmentController()
        c.odom.add(OdomSample(1., 0., 0., 0.))
        for _ in range(10):
            c.observe(1., 1., 2., 0., 0.)
        self.assertEqual(c.confirmed, 1)
        self.assertFalse(c.start(1., 1.)[0])

    def test_timestamp_interpolation_and_world_filter(self):
        c = AlignmentController()
        c.odom.add(OdomSample(1., 0., 0., 0.))
        c.odom.add(OdomSample(1.1, .02, 0., .02))
        pose = c.odom.at(1.05)
        self.assertAlmostEqual(pose.x, .01)
        self.assertAlmostEqual(pose.yaw, .01)
        self.assertIsNone(c.odom.at(1.11))
        for t in (1., 1.05, 1.1):
            p = c.odom.at(t)
            dx, dy = 2.-p.x, -p.y
            c.observe(t, t+.05, math.cos(p.yaw)*dx+math.sin(p.yaw)*dy,
                      -math.sin(p.yaw)*dx+math.cos(p.yaw)*dy, -p.yaw)
        self.assertAlmostEqual(c.target[0], 2.)
        self.assertAlmostEqual(c.target[1], 0.)
        self.assertAlmostEqual(c.target[2], 0.)

    def test_no_reverse_or_motion_outside_vision_range(self):
        for forward, left, yaw in ((1.50, 0, 0), (1.5655, .1, 0), (2., 0, 19.)):
            c = AlignmentController()
            self.seed(c, forward, left, math.radians(yaw))
            c.start(1., 1.)
            c.tick(1., 1.)
            self.assertFalse(c.active)
            self.assertEqual(c.state, 'REPOSITION_REQUIRED')
            self.assertEqual((c.v, c.w), (0., 0.))

    def test_interlock_immediately_disarms(self):
        c = AlignmentController()
        self.seed(c, forward=2.)
        c.start(1., 1.)
        c.tick(1., 1.)
        c.tick(1.01, 1.01, False, 'Laser lost')
        self.assertEqual(c.state, 'BLOCKED')
        self.assertFalse(c.active)
        self.assertEqual((c.v, c.w), (0., 0.))

    def test_closed_loop_mirrored_offsets_and_small_angle(self):
        for left, heading in ((0., 0.), (.05, 0.), (-.05, 0.), (0., 1.4), (.05, 5.), (-.05, -5.), (.10, 0.), (-.10, 0.)):
            with self.subTest(left=left, heading=heading):
                c = AlignmentController()
                x = y = yaw = v = w = 0.
                for i in range(2400):
                    t = 1.+i*.05
                    c.odom.add(OdomSample(t, x, y, yaw, v, w))
                    dx, dy = 2.1655-x, left-y
                    c.observe(t, t, math.cos(yaw)*dx+math.sin(yaw)*dy,
                              -math.sin(yaw)*dx+math.cos(yaw)*dy, wrap(math.radians(heading)-yaw))
                    if i == 3:
                        self.assertTrue(c.start(t, t)[0])
                    oldv, oldw = v, w
                    result = c.tick(t, t)
                    v, w = c.v, c.w
                    self.assertLessEqual(abs(v), c.config.max_linear_mps+1e-6)
                    self.assertLessEqual(abs(w), .200001)
                    if v != 0:  # Immediate safety/completion stops intentionally override acceleration.
                        self.assertLessEqual(abs(v-oldv), .004001)
                    if w != 0:
                        self.assertLessEqual(abs(w-oldw), .020001)
                    x += v*math.cos(yaw)*.05
                    y += v*math.sin(yaw)*.05
                    yaw = wrap(yaw+w*.05)
                    if i > 3 and not c.active:
                        break
                self.assertEqual(result['state'], 'ALIGNED', result)
                e = result['errors']
                self.assertLessEqual(abs(e['distance_m']), .02)
                self.assertLess(abs(e['lateral_m']), .02)
                self.assertLess(abs(e['heading_deg']), 2.)
                if heading == 1.4:
                    self.assertAlmostEqual(yaw, 0.)
                    self.assertAlmostEqual(e['heading_deg'], 1.4)


if __name__ == '__main__':
    unittest.main()
