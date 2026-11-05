import math
import unittest

from plan_sim import evaluate, swept_check, uncertainty_budget
from docking_preview import candidate
from quality_replay import StationaryGate, circular_location, depth_stats, synthetic_frames
from integrated_scenario import scenario


class PlanSimulationTest(unittest.TestCase):
    def test_obstacle_rejected_and_clear_candidate_has_uncertainty_margin(self):
        clear=evaluate(.02,2.,1.2)
        blocked=evaluate(.02,2.,1.2,obstacle=True)
        self.assertEqual(clear['result'],'ILLUSTRATIVE_FEASIBLE')
        self.assertGreater(clear['swept_clearance_m'],clear['uncertainty_m'])
        self.assertEqual(blocked['result'],'REPOSITION_REQUIRED')
        self.assertLess(blocked['swept_clearance_m'],0.)

    def test_swept_fork_collision_is_detected(self):
        path=candidate(0.,0.,1.2)
        # Placing a pallet face too close forces the body or fork into it.
        clearance,reason=swept_check(path['rows'],.3)
        self.assertLess(clearance,0.)
        self.assertIn('intersects',reason)

    def test_quality_replay_duplicate_depth_bias_and_yaw_wrap(self):
        gate=StationaryGate()
        for row in synthetic_frames(): gate.add(row)
        self.assertEqual(gate.reasons['duplicate_or_out_of_order'],1)
        self.assertEqual(gate.reasons['depth_samples'],1)
        self.assertGreater(abs(gate.pose()['x_m']-1.2),.005)
        self.assertLess(abs(abs(math.degrees(circular_location([
            math.radians(179),math.radians(-179)])))-180),2)
        self.assertIsNone(depth_stats([0.,float('nan')]*50,100))

    def test_target_switch_resets_pose_evidence(self):
        gate=StationaryGate()
        a=synthetic_frames()[0]
        gate.add(a)
        b=dict(a,stamp_s=1.,now_s=1.04,depth_stamp_s=1.015,target_id='B')
        gate.add(b)
        self.assertEqual(gate.reasons['target_change_reset'],1)
        self.assertEqual(len(gate.rows),1)

    def test_integrated_bias_blocks_and_corrected_case_waits_for_insertion(self):
        biased=scenario(0.)
        corrected=scenario(.01)
        self.assertEqual(biased['state'],'ACQUIRING')
        self.assertFalse(biased['motor_command_sent'])
        self.assertEqual(corrected['state'],'WAIT_INSERTION_VERIFICATION')
        self.assertFalse(corrected['insertion_verified'])
        self.assertFalse(corrected['lift_command_sent'])


if __name__=='__main__':
    unittest.main()
