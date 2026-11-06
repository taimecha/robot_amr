"""The RGB-D lock must use a measured edge and reject transient pose noise."""
import math
import unittest

import numpy as np

from pallet_rgbd_filter import StablePoseWindow,front_edge_yaw


class PalletRGBDFilterTest(unittest.TestCase):
    def test_front_edge_heading_with_depth_noise(self):
        rng=np.random.default_rng(14)
        lateral=rng.uniform(-.17,.17,5000)
        forward=1.4+math.tan(math.radians(8))*lateral+rng.uniform(0,.355,5000)
        forward+=rng.normal(0,.002,5000)
        forward[:150]+=rng.normal(.12,.04,150)
        self.assertAlmostEqual(math.degrees(front_edge_yaw(forward,lateral)),8,delta=1.5)

    def test_lock_waits_for_stable_heading_and_ignores_outlier(self):
        window=StablePoseWindow()
        angles=(8.0,8.5,7.8,23.0,8.2,8.1,7.9)
        result=None
        for i,angle in enumerate(angles):
            result=window.add(.2*i,1.9,.1,math.radians(angle))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(math.degrees(result[2]),8.1,delta=.5)
        self.assertEqual(len(window.rows),6)


if __name__=='__main__':
    unittest.main()
