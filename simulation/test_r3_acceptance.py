"""User-approved longitudinal waiver must not weaken other tracking criteria."""
import unittest
from r3_score_gazebo import tracking_acceptance

class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.metrics=dict(matched_samples=1,odom_samples=1,
                          terminal_truth_max_yaw_deg=.093,terminal_truth_max_lateral_m=.000764,
                          stopped_truth=dict(yaw_error_rad=.00075,ey_m=.00061,gap_m=.04162))
        self.odom=[dict(cmd_v_mps=.15)]

    def test_longitudinal_waiver_is_explicit(self):
        self.assertFalse(all(tracking_acceptance(self.metrics,self.odom).values()))
        self.assertTrue(all(tracking_acceptance(self.metrics,self.odom,False).values()))

    def test_waiver_does_not_allow_heading_or_lateral_errors(self):
        for key,value in [('terminal_truth_max_yaw_deg',.501),('terminal_truth_max_lateral_m',.0201)]:
            metrics=dict(self.metrics);metrics[key]=value
            self.assertFalse(all(tracking_acceptance(metrics,self.odom,False).values()))

    def test_waiver_does_not_allow_speed_or_reverse(self):
        for v in (-.001,.151):
            self.assertFalse(all(tracking_acceptance(self.metrics,[dict(cmd_v_mps=v)],False).values()))

    def test_missing_truth_cannot_pass(self):
        self.metrics['matched_samples']=0
        self.assertFalse(all(tracking_acceptance(self.metrics,self.odom,False).values()))

if __name__=='__main__':unittest.main()
