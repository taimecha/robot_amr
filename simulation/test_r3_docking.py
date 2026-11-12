"""r3 frame, geometry, immutable plan and phase safety regression tests."""
import dataclasses
import json
import math
from pathlib import Path
import unittest
import numpy as np
from r3_core import (FrozenPlan, base_for_fork, collision, compose, corrected_base_scalar_target,
                     errors, fork_for_base, goal_base, inverse, optical_target, validate_geometry)
from r3_sim import FixedPlanFollower
from r3_camera_preview import locked_snapshot

FIXTURE=Path(__file__).parent/'r3/fixture.json'

class R3Tests(unittest.TestCase):
    def setUp(self):
        self.g=json.loads(FIXTURE.read_text())

    def test_goal_continuous_yaw_and_fork_offset(self):
        target=(2.,.13,math.radians(13.17))
        base=goal_base(target,self.g);e=errors(base,target,self.g)
        self.assertAlmostEqual(e['gap_m'],.020)
        self.assertAlmostEqual(e['ey_m'],0.)
        self.assertAlmostEqual(e['yaw_error_rad'],0.)
        self.assertAlmostEqual(base_for_fork(fork_for_base(base,self.g),self.g)[0],base[0])

    def test_housing_is_not_base_or_optical(self):
        self.g['housing_y']=.037;self.g['base_fork'][1]=.017
        validate_geometry(self.g)
        self.assertAlmostEqual(goal_base((2.,0.,0.),self.g)[1],-.017)
        self.g['base_fork'][1]=-.020
        with self.assertRaises(ValueError):validate_geometry(self.g)

    def test_optical_requires_full_transform_and_maps_once(self):
        meta=dict(feature='front_entry_corridor_center',applied_bias=[],calibration_hash='unit',capture_stamp_s=1.)
        with self.assertRaises(ValueError):
            optical_target(T_odom_base=np.eye(4),T_base_camera_optical=None,T_camera_pallet=np.eye(4),metadata=meta)
        camera=np.eye(4);camera[0,3]=.367
        measured=np.eye(4);measured[0,3]=1.
        result=optical_target(T_odom_base=np.eye(4),T_base_camera_optical=camera,T_camera_pallet=measured,metadata=meta)
        self.assertAlmostEqual(result['pose'][0],1.367)
        result=corrected_base_scalar_target(odom_at_capture=(0,0,0),forward_base_m=1.367,lateral_base_m=.01,
                      entry_heading_rad=.2,metadata=dict(meta,source_frame='base_link_corrected_scalars'))
        self.assertAlmostEqual(result['pose'][0],1.367)
        self.assertAlmostEqual(result['pose'][1],.01)

    def test_approved_dimensions_and_limits_cannot_be_relaxed(self):
        for key,value in [('max_speed',.151),('max_yaw_deg',.51),('max_lateral',.021),('lift_target',.085),('home_settle',.002),('tine_thickness',.008)]:
            g=dict(self.g);g[key]=value
            with self.assertRaises(ValueError):validate_geometry(g)
        validate_geometry(self.g)

    def test_four_legs_and_vertical_collision(self):
        goal=goal_base((1.8,0.,0.),self.g)
        self.assertIsNone(collision(goal,(1.8,0.,0.),self.g))
        g=dict(self.g);g['fork_bottom_z']=.101
        self.assertEqual(collision(goal,(1.8,0.,0.),g),'tine/deck')
        p=list(goal);p[1]+=.12
        self.assertEqual(collision(p,(1.8,0.,0.),self.g),'tine/pillar')

    def test_preview_wait_and_obstacle_stop_never_resume(self):
        a=goal_base((1.8,0.,0.),self.g,.405);end=goal_base((1.8,0.,0.),self.g)
        phases=[dict(type='STRAIGHT',poses=[a,end])]
        follower=FixedPlanFollower(phases,(1.8,0.,0.),self.g)
        self.assertEqual(follower.step(a),(0.,0.))
        self.assertFalse(follower.start(False));self.assertTrue(follower.start(True))
        self.assertEqual(follower.step(a,obstacle=True),(0.,0.))
        self.assertEqual(follower.step(a),(0.,0.));self.assertEqual(follower.state,'STOP_LATCHED')

    def test_realistic_endpoint_switches_from_approach_to_straight(self):
        target=(1.8,0.,0.)
        c=goal_base(target,self.g,.535)
        b=goal_base(target,self.g)
        phases=[dict(type='APPROACH',poses=[(0.,0.,0.),c]),
                dict(type='STRAIGHT',poses=[c,b])]
        follower=FixedPlanFollower(phases,target,self.g)
        follower.start(True)
        # 0.9 mm short of C is well inside physical odom resolution.
        near_c=(c[0]-.0009,c[1]+.003,math.radians(.456))
        for _ in range(11):
            follower.step(near_c)
        self.assertEqual(follower.state,'STRAIGHT')
        self.assertGreater(follower.step(near_c)[0],0.)

    def test_c_rotates_before_straight_when_yaw_shifts_fork_laterally(self):
        target=(1.8,0.,0.)
        g=dict(self.g,max_lateral=.009)
        c=goal_base(target,g,.535)
        b=goal_base(target,g)
        follower=FixedPlanFollower([dict(type='APPROACH',poses=[(0.,0.,0.),c]),
                                   dict(type='STRAIGHT',poses=[c,b])],target,g)
        follower.start(True)
        misaligned=(c[0],c[1]+.001,math.radians(-2.))
        self.assertGreater(abs(errors(misaligned,target,g)['ey_m']),g['max_lateral'])
        for _ in range(11):
            follower.step(misaligned)
        self.assertEqual(follower.state,'ALIGN_YAW')
        follower.step(misaligned)
        self.assertNotEqual(follower.state,'STOP_LATCHED')
        aligned=(c[0],c[1]+.001,c[2])
        for _ in range(7):
            follower.step(aligned)
        self.assertEqual(follower.state,'STRAIGHT')
        self.assertGreater(errors(aligned,target,g)['gap_m']-g['fork_length'],.10)

    def test_real_c_waits_for_camera_before_straight(self):
        target=(1.8,0.,0.)
        c=goal_base(target,self.g,.535)
        b=goal_base(target,self.g)
        follower=FixedPlanFollower([dict(type='APPROACH',poses=[(0.,0.,0.),c]),
                                   dict(type='STRAIGHT',poses=[c,b])],target,self.g)
        follower.start(True)
        near_c=(c[0]-.04,c[1]+.001,c[2]+math.radians(.46))
        v,w=follower.step(near_c,c_verified=False)
        self.assertEqual((v,w),(0.,0.))
        self.assertEqual(follower.state,'C_SETTLE')
        for _ in range(10):
            v,w=follower.step(near_c,c_verified=False)
        self.assertEqual((v,w),(0.,0.))
        self.assertEqual(follower.state,'WAIT_C_RGBD')
        follower.step(near_c,c_verified=True)
        self.assertEqual(follower.state,'STRAIGHT')

    def test_c_brake_does_not_hide_endpoint_overshoot(self):
        target=(1.8,0.,0.)
        c=goal_base(target,self.g,.535)
        b=goal_base(target,self.g)
        follower=FixedPlanFollower([dict(type='APPROACH',poses=[(0.,0.,0.),c]),
                                   dict(type='STRAIGHT',poses=[c,b])],target,self.g)
        follower.start(True)
        cmd=follower.step((c[0]+.004,c[1],c[2]),c_verified=False)
        self.assertEqual(cmd,(0.,0.))
        self.assertEqual(follower.state,'STOP_LATCHED')
        self.assertEqual(follower.reason,'phase endpoint overshot')

    def test_straight_lead_in_stops_pure_pursuit_from_oversteering_yaw(self):
        target=(2.,0.,0.)
        g=dict(self.g,max_speed=.05,max_lateral=.009)
        c=goal_base(target,g,.535)
        b=goal_base(target,g)
        phases=[dict(type='APPROACH',poses=[(0.,0.,0.),(c[0]-.12,0.,0.),c],
                     straight_lead_in_m=.12),
                dict(type='STRAIGHT',poses=[c,b])]
        follower=FixedPlanFollower(phases,target,g)
        follower.start(True)
        aligned=(c[0]-.08,.003,0.)
        v,w=follower.step(aligned)
        self.assertGreater(v,0.)
        self.assertAlmostEqual(w,0.)
        self.assertEqual(follower.state,'APPROACH')
        self.assertLess(follower.step((aligned[0],aligned[1],math.radians(.5)))[1],0.)

    def test_terminal_stop_checks_odom_goal(self):
        target=(1.8,0.,0.)
        c=goal_base(target,self.g,.405)
        b=goal_base(target,self.g)
        follower=FixedPlanFollower([dict(type='STRAIGHT',poses=[c,b])],target,self.g)
        follower.start(True)
        follower.step((b[0]-.003,b[1],b[2]))
        self.assertEqual(follower.state,'STOPPED')
        self.assertLessEqual(abs(errors((b[0]-.003,b[1],b[2]),target,self.g)['gap_m']-self.g['stop_gap']),.005)

    def test_pose_feedback_preserves_lateral_correction_in_straight_lead_in(self):
        target=(2.,0.,0.)
        g=dict(self.g,max_speed=.10,max_lateral=.009)
        c=goal_base(target,g,.535)
        b=goal_base(target,g)
        phases=[dict(type='APPROACH',poses=[(0.,0.,0.),c],
                     straight_lead_in_m=.12,tracking_controller='pose_feedback'),
                dict(type='STRAIGHT',poses=[c,b])]
        for lateral in (-.003,.003):
            follower=FixedPlanFollower(phases,target,g)
            follower.start(True)
            v,w=follower.step((c[0]-.08,lateral,0.))
            self.assertGreater(v,0.)
            self.assertLess(w*lateral,0.)
            self.assertEqual(follower.state,'APPROACH')
        blocked=FixedPlanFollower(phases,target,g)
        blocked.start(True)
        self.assertEqual(blocked.step((c[0]-.08,.010,0.)),(0.,0.))
        self.assertEqual(blocked.state,'STOP_LATCHED')

    def test_approach_braking_avoids_low_speed_stall_near_c(self):
        g=dict(self.g,max_speed=.05)
        phases=[dict(type='APPROACH',poses=[(0.,0.,0.),(1.,0.,0.)])]
        follower=FixedPlanFollower(phases,(3.,0.,0.),g)
        follower.start(True)
        follower.step((0.,0.,0.))
        follower.v=follower.follower.v=.05
        for _ in range(20):
            v,_=follower.step((.98,0.,0.))
        self.assertGreaterEqual(v,.03)

    def test_rotation_after_entry_refused(self):
        target=(1.8,0.,0.);p=goal_base(target,self.g,.30)
        follower=FixedPlanFollower([dict(type='ALIGN_YAW',poses=[p,(*p[:2],.1)])],target,self.g)
        follower.start(True)
        self.assertEqual(follower.step(p),(0.,0.));self.assertEqual(follower.state,'STOP_LATCHED')

    def test_current_camera_profile_not_proof_of_half_degree(self):
        session=dict(provenance='stationary_real_camera',stationary_verified=True)
        calibration=dict(confirmed_real=True,yaw_accuracy_bound_deg=.7769,accuracy_evidence='historical check')
        with self.assertRaisesRegex(ValueError,'0.5deg'):locked_snapshot(session,calibration)

    def test_curvature_speed_cap_cannot_bypass_acceleration_limit(self):
        phases=[dict(type='APPROACH',poses=[(0.,0.,0.),(1.,0.,0.)])]
        follower=FixedPlanFollower(phases,(3.,0.,0.),self.g)
        follower.start(True);follower.step((0.,0.,0.))
        follower.v=follower.follower.v=.15
        v,w=follower.step((.1,.4,1.))
        self.assertGreaterEqual(v,.15-.08*.05-1e-12)
        self.assertLessEqual(v,.15)
        self.assertLessEqual(abs(w),.4*.05+1e-12)

    def test_frozen_plan_does_not_follow_mutable_target(self):
        source={'pose':[1.8,0.,0.]}
        plan=FrozenPlan(json.dumps(source),json.dumps(self.g),'[]','{}');before=plan.plan_id
        source['pose'][0]=99.
        self.assertEqual(plan.plan_id,before)
        with self.assertRaises(dataclasses.FrozenInstanceError):plan.target_json='{}'

if __name__=='__main__':unittest.main()
