"""The pallet mask must leave nearby people/boxes visible in both scans."""

import math
import unittest

from r3_pallet_obstacles import classify_ranges, pallet_return


class PalletObstacleMaskTest(unittest.TestCase):
    def setUp(self):
        self.geometry = {'pallet_length': .355, 'pallet_width': .355}
        self.target = (1.0, .08, 0.)

    def test_only_frozen_pallet_box_is_exempt(self):
        self.assertTrue(pallet_return(1.0, .08, self.target, self.geometry))
        self.assertFalse(pallet_return(.96, .08, self.target, self.geometry))
        self.assertFalse(pallet_return(1.1, .30, self.target, self.geometry))
        self.assertFalse(pallet_return(1.40, .08, self.target, self.geometry))

    def test_ray_mask_keeps_adjacent_obstacle(self):
        # Two returns in one scan: the center of the pallet and an object
        # just outside its side face. A broad angular mask would hide both.
        ranges = [1.1, math.hypot(1.1, .31)]
        angles = [math.atan2(.08, 1.1), math.atan2(.31, 1.1)]
        filtered, pallet, other = classify_ranges(
            ranges, angles[0], angles[1] - angles[0], (0., 0., 0.),
            self.target, self.geometry)
        self.assertEqual((pallet, other), (1, 1))
        self.assertTrue(math.isinf(filtered[0]))
        self.assertEqual(filtered[1], ranges[1])

    def test_transform_follows_odom_not_camera_guess(self):
        filtered, pallet, other = classify_ranges(
            [1.1, .4], 0., math.pi / 2, (0., .08, 0.),
            self.target, self.geometry)
        self.assertEqual((pallet, other), (1, 1))
        self.assertTrue(math.isinf(filtered[0]))
        self.assertEqual(filtered[1], .4)


if __name__ == '__main__':
    unittest.main()
