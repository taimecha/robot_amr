import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'scripts')]
from pallet_target_geometry import measure_pallet_target
from pallet_plane_heading import floor_depth_skew
from pallet_rgbd_estimator import RGBDTargetEstimator


class PlaneHeadingTest(unittest.TestCase):
    def scene(self,yaw=0.,skew=.0001,roll=0.):
        k=np.array([[613.,0,323.],[0,611.,238.],[0,0,1.]])
        v,u=np.mgrid[:480,:640]
        rx=(u-k[0,2])/k[0,0];ry=(v-k[1,2])/k[1,1]
        pitch=math.radians(20)
        angle=math.radians(roll)
        rotation=np.array([[math.cos(angle),-math.sin(angle),0.],
                           [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
        floor_normal=rotation@np.array([0.,math.cos(pitch),math.sin(pitch)])/.75
        pallet_normal=rotation@np.array([math.tan(math.radians(yaw)),
                                        -math.sin(pitch),math.cos(pitch)])/1.8
        inverse_floor=floor_normal[0]*rx+floor_normal[1]*ry+floor_normal[2]
        inverse_pallet=pallet_normal[0]*rx+pallet_normal[1]*ry+pallet_normal[2]
        mask=np.zeros((480,640),bool)
        mask[170:260,270:390]=True
        mask[260:300,270:288]=True;mask[260:300,372:390]=True
        inverse=np.where(mask,inverse_pallet,inverse_floor)-skew*(370-u)
        depth=np.clip(np.rint(1000/np.maximum(inverse,.0001)),0,65535).astype(np.uint16)
        return mask,np.array([270,170,390,300]),depth,(k,np.zeros(5))

    def test_independent_floor_skew_and_real_positive_negative_plane_angles(self):
        for skew in (.00007,.00010,.00015807):
            for yaw in (0.,5.,-5.):
                with self.subTest(skew=skew,yaw=yaw):
                    mask,box,depth,intrinsics=self.scene(yaw,skew)
                    measured_skew,_=floor_depth_skew(depth,intrinsics)
                    self.assertAlmostEqual(measured_skew,skew,delta=2e-6)
                    m=measure_pallet_target(mask,box,depth,intrinsics,heading_floor_reference=True)
                    self.assertAlmostEqual(math.degrees(m['face_yaw_rad']),yaw,delta=.15)
                    self.assertAlmostEqual(m['entry_heading_rad'],-m['face_yaw_rad'])

    def test_position_and_yaw_share_one_floor_depth_correction(self):
        mask,box,depth,intrinsics=self.scene()
        old=measure_pallet_target(mask,box,depth,intrinsics)
        with patch('pallet_plane_heading.floor_depth_skew',wraps=floor_depth_skew) as fit:
            new=measure_pallet_target(mask,box,depth,intrinsics,heading_floor_reference=True)
        fit.assert_called_once()
        shared=measure_pallet_target(mask,box,depth,intrinsics,
                                     inverse_depth_x_slope=new['heading_depth_x_slope'])
        for key in ('camera_forward_m','chassis_front_forward_m','opening_lateral_m',
                    'opening_width_m','leg_center_separation_m','left_depth_samples','right_depth_samples'):
            self.assertEqual(shared[key],new[key])
        self.assertGreater(abs(old['camera_forward_m']-new['camera_forward_m']),.005)
        self.assertEqual(new['position_depth_x_slope'],new['heading_depth_x_slope'])
        self.assertTrue(new['depth_correction_shared'])
        args=SimpleNamespace(pitch_deg=20,inverse_depth_x_slope=.00015807,reference_u=370,
            camera_ahead_of_chassis_m=.0665,expected_opening_width_m=.252,
            opening_width_tolerance_m=.08,confirm_frames=3,heading_floor_reference=True,
            measurement_bias=dict(forward_bias_m=.03843,lateral_bias_m=.02053,face_yaw_bias_deg=.89627))
        result=RGBDTargetEstimator(args).measure(mask,box,depth,intrinsics)
        self.assertAlmostEqual(math.degrees(result['face_yaw_rad']),0.,delta=.15)
        self.assertEqual(result['applied_heading_bias_deg'],0.)
        self.assertEqual(result['camera_forward_m'],new['camera_forward_m']+.03843)
        self.assertEqual(result['opening_lateral_m'],new['opening_lateral_m']+.02053)

    def test_unmeasured_camera_roll_can_bias_floor_referenced_heading(self):
        # A flat floor cannot distinguish physical camera roll from the
        # horizontal sensor skew. Expose this assumption instead of calling
        # the angle independently calibrated because its temporal spread is low.
        estimates=[]
        for roll in (-1.,0.,1.):
            mask,box,depth,intrinsics=self.scene(skew=.000082,roll=roll)
            measured=measure_pallet_target(mask,box,depth,intrinsics,heading_floor_reference=True)
            self.assertTrue(measured['heading_requires_zero_camera_roll'])
            self.assertFalse(measured['heading_camera_mount_yaw_verified'])
            estimates.append(math.degrees(measured['entry_heading_rad']))
        self.assertGreater(estimates[0],2.)
        self.assertAlmostEqual(estimates[1],0.,delta=.15)
        self.assertLess(estimates[2],-2.)

    def test_missing_floor_and_missing_leg_are_rejected(self):
        mask,box,depth,intrinsics=self.scene()
        for mode in ('floor','leg'):
            broken=depth.copy()
            if mode=='floor':broken[350:470,:]=0
            else:broken[260:300,270:288]=0
            with self.subTest(mode=mode),self.assertRaises(ValueError):
                measure_pallet_target(mask,box,broken,intrinsics,heading_floor_reference=True)


if __name__=='__main__':unittest.main()
