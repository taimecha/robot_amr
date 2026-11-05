import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_rgbd_health import RGBDHealth


class HealthTest(unittest.TestCase):
    def fill(self, health, fps=25, seconds=6, drift=0.):
        for i in range(int(fps*seconds)+1):
            t = i/fps
            health.sample(100+t, t, 100+t, True, 2+drift*t, .01, math.radians(1))
        return health.snapshot(seconds, 100+seconds)

    def test_requires_full_window_and_current_data(self):
        h = RGBDHealth()
        self.assertFalse(self.fill(h, seconds=4)['stable'])
        self.assertTrue(self.fill(h)['stable'])
        self.assertFalse(h.snapshot(6.3, 106.3)['stable'])
        h.reset()
        self.assertFalse(h.snapshot(6.3, 106.3)['stable'])

    def test_accepts_low_rate_when_fresh_but_rejects_drift_invalid_and_replay(self):
        self.assertTrue(self.fill(RGBDHealth(), fps=2)['stable'])
        self.assertFalse(self.fill(RGBDHealth(), drift=.01)['stable'])
        h = RGBDHealth(); self.fill(h)
        h.sample(106.04, 6.04, 106.04, False, 0, 0, 0)
        self.assertFalse(h.snapshot(6.04,106.04)['stable'])
        for i in range(10):
            h.sample(106.04,6.05,106.05,True,2,.01,0.)
        self.assertFalse(h.snapshot(6.05,106.05)['stable'])

    def test_future_stamp_and_stale_result_fail(self):
        h = RGBDHealth(); self.fill(h)
        h.sample(106.4,6.4,106.4,True,2,.01,math.radians(1))
        self.assertTrue(h.snapshot(6.4,106.4)['stable'])
        self.assertFalse(h.snapshot(6.7,106.7)['stable'])
        h.sample(110.,6.44,106.44,True,2,.01,0.)
        self.assertFalse(h.snapshot(6.44,106.44)['stable'])

    def test_yaw_wrap_is_a_small_spread(self):
        h=RGBDHealth()
        for i in range(11):
            t=i*.5
            h.sample(100+t,t,100+t,True,2.,.01,
                     math.radians(179.5 if i%2 else -179.5))
        result=h.snapshot(5.,105.)
        self.assertTrue(result['stable'])
        self.assertAlmostEqual(result['heading_range_deg'],1.,places=3)


if __name__ == '__main__':
    unittest.main()
