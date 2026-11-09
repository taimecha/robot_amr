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
