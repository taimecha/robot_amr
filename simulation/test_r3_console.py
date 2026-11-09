"""Pose reference, sensor provenance and the user's two-line display contract."""
import json
import math
from pathlib import Path
import unittest
import numpy as np
from matplotlib.figure import Figure
from r3_case import ForkCase
from r3_core import compose,inverse,errors,tangent_mismatch
from r3_plot import draw_two_paths
from r3_vision_core import measured_front,rotation
G=json.loads((Path(__file__).parent/'r3/fixture.json').read_text())

class ConsoleTests(unittest.TestCase):
    def test_unrestricted_signed_pose_and_wrapped_yaw(self):
        for d,y,a in [(25.,-12.,450.),(-5.,30.,-725.),(1000.,-1000.,180.)]:
            with self.subTest(d=d,y=y,a=a):
                c=ForkCase(d,y,a).scenario(G)
                f=compose(inverse(c['pallet_world']),c['fork_world'])
                self.assertAlmostEqual(f[0],-d);self.assertAlmostEqual(f[1],y)
                self.assertAlmostEqual(math.sin(f[2]),math.sin(math.radians(a)))
                e=errors((0,0,0),c['target_odom'],G)
                self.assertAlmostEqual(e['gap_m'],d);self.assertAlmostEqual(e['ey_m'],y)
                self.assertAlmostEqual(math.sin(e['yaw_error_rad']),math.sin(math.radians(a)))

    def test_varying_curvature_chord_is_not_assumed_to_bisect_tangents(self):
        t=np.linspace(0,1,10001);theta=.06*(1-(1-t)**3)
        b=(float(np.trapz(np.cos(theta),t)),float(np.trapz(np.sin(theta),t)),.06)
        self.assertEqual(tangent_mismatch((0,0,0),b),0.)
        self.assertGreater(tangent_mismatch((0,0,0),(-1,0,0)),math.radians(179.))

    def test_nonfinite_inputs_rejected(self):
        for v in (float('nan'),float('inf'),-float('inf')):
            with self.assertRaises(ValueError):ForkCase(v,0,0)

    def test_plot_has_exactly_planner_and_actual_fork_lines(self):
        ax=Figure().subplots();planned=[(.3655,-.02,0),(1.8,0,0)];actual=[(.3655,-.02,0),(1.79,.003,0)]
        draw_two_paths(ax,planned,actual,(1.82,0,0),G)
        self.assertEqual(len(ax.lines),2)
        np.testing.assert_allclose(ax.lines[0].get_xydata(),[p[:2] for p in planned])
        np.testing.assert_allclose(ax.lines[1].get_xydata(),[p[:2] for p in actual])

    def test_rgbd_estimate_uses_only_depth_intrinsics_tf_and_capture_odom(self):
        # Render a top-deck patch analytically into a native X-forward depth camera.
        u,v=np.meshgrid(np.arange(640),np.arange(480));fx=fy=600.;cx=320.;cy=100.
        depth=np.divide(.27*fy,v-cy,out=np.full(u.shape,np.nan),where=v>cy)
        y=-(u-cx)*depth/fx
        mask=(depth>=1.5)&(depth<=1.855)&(abs(y)<=.1775)
        pose,m=measured_front(mask,depth,[fx,0,cx,0,fy,cy,0,0,1],[0,0,.4],[0,0,0,1],(0,0,0),G)
        self.assertLess(abs(pose[0]-1.5),.03);self.assertLess(abs(pose[1]),.003)
        self.assertLess(abs(m['yaw_deg']),.5)
        rotated,_=measured_front(mask,depth,[fx,0,cx,0,fy,cy,0,0,1],[0,0,.4],[0,0,0,1],(2,-1,.7),G)
        np.testing.assert_allclose(rotated,compose((2,-1,.7),pose),atol=1e-10)

    def test_pallet_stays_fixed_when_only_vehicle_pose_changes(self):
        expected=np.array([[1.8225,-.0775],[2.1775,-.0775],
                           [2.1775,.2775],[1.8225,.2775],[1.8225,-.0775]])
        for d,y,yaw in ((1.5,0,0),(2,.25,35),(1,-.2,-20),(20,3,135)):
            with self.subTest(distance=d,lateral=y,yaw=yaw):
                case=ForkCase(d,y,yaw).scenario(G);ax=Figure().subplots()
                draw_two_paths(ax,[],[],case['target_odom'],G,scene=case)
                patches={p.get_gid():p for p in ax.patches if p.get_gid()}
                np.testing.assert_allclose(patches['pallet_outline'].get_xy(),expected,atol=1e-12)
                np.testing.assert_allclose(patches['initial_fork_root'].center,[1.8225-d,.1+y],atol=1e-12)
                self.assertEqual(ax.get_xlabel(),'world X (m)')

    def test_both_paths_and_initial_vehicle_are_projected_into_same_world(self):
        from copy import deepcopy
        from r3_core import fork_for_base,rectangle
        case=ForkCase(2,.25,35).scenario(G)
        start=fork_for_base((0,0,0),G)
        planned=[start,(start[0]+1,start[1]-.2,0.)]
        actual=[start[:2],(start[0]+.9,start[1]-.21)]
        original=deepcopy((planned,actual,case))
        ax=Figure().subplots();draw_two_paths(ax,planned,actual,case['target_odom'],G,scene=case)
        self.assertEqual(len(ax.lines),2)
        angle=math.radians(35);x0,y0=case['base_world'][:2]
        for line,path in zip(ax.lines,(planned,actual)):
            expected=[(x0+math.cos(angle)*p[0]-math.sin(angle)*p[1],
                       y0+math.sin(angle)*p[0]+math.cos(angle)*p[1]) for p in path]
            np.testing.assert_allclose(line.get_xydata(),expected,atol=1e-12)
        body=next(p for p in ax.patches if p.get_gid()=='initial_base')
        np.testing.assert_allclose(body.get_xy()[:4],rectangle(case['base_world'],G['body_bounds']),atol=1e-12)
        self.assertEqual((planned,actual,case),original)

    def test_measured_target_does_not_rotate_physical_display_pallet(self):
        case=ForkCase(1.5,.2,15).scenario(G)
        ax=Figure().subplots()
        draw_two_paths(ax,[],[],case['target_odom'],G,scene=case)
        first=next(p.get_xy().copy() for p in ax.patches if p.get_gid()=='pallet_outline')
        noisy_target=[case['target_odom'][0]+.02,case['target_odom'][1]-.03,case['target_odom'][2]+.1]
        draw_two_paths(ax,[],[],noisy_target,G,scene=case)
        second=next(p.get_xy() for p in ax.patches if p.get_gid()=='pallet_outline')
        np.testing.assert_allclose(first,second,atol=1e-12)

    def test_duplicate_world_guard_recognizes_only_its_partition(self):
        import os,subprocess
        from r3_world_guard import existing_servers
        children=[]
        try:
            for partition in ('tai_r3_sim_only_89','unrelated_partition'):
                children.append(subprocess.Popen(['gz sim server','5'],executable='/bin/sleep',env=dict(os.environ,GZ_PARTITION=partition)))
            found=existing_servers('tai_r3_sim_only_89')
            self.assertIn(children[0].pid,found);self.assertNotIn(children[1].pid,found)
        finally:
            for child in children:child.terminate();child.wait()

    def test_color_only_update_preserves_geometry_collision_and_other_models(self):
        import xml.etree.ElementTree as ET
        from copy import deepcopy
        from r3.appearance import set_pallet_materials,rgba
        original=ET.parse(Path(__file__).parent.parent/'ros2_ws/worlds/docking_one_pallet.sdf').getroot()
        changed=deepcopy(original)
        self.assertEqual(set_pallet_materials(changed),5)
        np.testing.assert_allclose(rgba(),[37/255,161/255,142/255,1.])
        # Erasing only the two authorized material attributes must restore the
        # exact original XML tree, including collisions/poses/inertials/world.
        for root in (original,changed):
            for v in root.findall('.//model[@name="single_floor_pallet"]/link/visual'):
                for key in ('ambient','diffuse'):v.find('material').remove(v.find('material/'+key))
        self.assertEqual(ET.tostring(original),ET.tostring(changed))

    def test_native_to_optical_conventions(self):
        # A 90 degree yaw transforms camera X into base Y.
        r=rotation([0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
        np.testing.assert_allclose(r@np.array([1,0,0]),[0,1,0],atol=1e-12)

if __name__=='__main__':unittest.main()
