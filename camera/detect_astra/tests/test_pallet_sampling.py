from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pallet_target_geometry import _sample_front_legs, _sample_leg


class PalletSamplingTest(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((60, 300), 1500, np.uint16)
        self.intrinsics = (np.array([[500., 0, 150], [0, 500, 30], [0, 0, 1]]), np.zeros(5))

    def component(self, center, holes=False):
        u, v = np.meshgrid(np.arange(center-3, center+4), np.arange(10, 36))
        u, v = u.ravel(), v.ravel()
        if holes:
            keep = ~((v == 22) & (u <= center))
            u, v = u[keep], v[keep]
        return dict(u=u, v=v, width=17, top=10, bottom=35)

    def sample(self, components):
        return _sample_front_legs(components, self.depth, self.intrinsics, 10, 35,
                                  0, 0, 370, .65, 3.5, 60)

    def test_valid_original_band_is_unchanged(self):
        components = [self.component(100), self.component(200)]
        expected = [_sample_leg(c, self.depth, self.intrinsics, (18, 26), 0, 0, 370,
                                .65, 3.5, 60) for c in components]
        samples, rows = self.sample(components)
        self.assertEqual(rows, (18, 26))
        self.assertEqual(samples, expected)

    def test_small_mask_holes_expand_both_legs_without_lowering_sample_count(self):
        components = [self.component(100, holes=True), self.component(200)]
        with self.assertRaisesRegex(ValueError, 'too few mask pixels'):
            _sample_leg(components[0], self.depth, self.intrinsics, (18, 26), 0, 0,
                        370, .65, 3.5, 60)
        samples, rows = self.sample(components)
        self.assertEqual(rows, (17, 27))
        self.assertTrue(all(s['samples'] >= 60 for s in samples))
        self.assertTrue(all(s['forward_m'] == 1.5 for s in samples))

    def test_invalid_depth_and_unstable_surface_still_fail(self):
        components = [self.component(100), self.component(200)]
        self.depth[:] = 0
        with self.assertRaisesRegex(ValueError, 'insufficient valid depth'):
            self.sample(components)
        self.depth[:] = (1000 + np.arange(60) * 20).astype(np.uint16)[:, None]
        with self.assertRaisesRegex(ValueError, 'not one stable front surface'):
            self.sample(components)


if __name__ == '__main__':
    unittest.main()
