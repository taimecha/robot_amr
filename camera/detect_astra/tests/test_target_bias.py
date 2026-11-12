import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pallet_target_bias import apply_target_bias, load_target_bias


class TargetBiasTest(unittest.TestCase):
    def measurement(self):
        yaw = math.radians(-0.9)
        difference = math.tan(yaw) * 0.32 / 2
        return dict(camera_forward_m=1.5445, chassis_front_forward_m=1.611,
                    opening_lateral_m=-0.0668, face_yaw_rad=yaw,
                    entry_heading_rad=-yaw, leg_center_separation_m=0.32,
                    left_forward_m=1.5445 + difference,
                    right_forward_m=1.5445 - difference,
                    left_depth_samples=100, right_depth_samples=120,
                    opening_width_m=0.252)

    def test_corrected_pose_plane_and_entry_heading_are_consistent(self):
        original = self.measurement()
        saved = dict(original)
        bias = dict(forward_bias_m=0.03843, lateral_bias_m=0.02053, face_yaw_bias_deg=0.89627)
        result = apply_target_bias(original, bias)
        self.assertEqual(original, saved)
        self.assertAlmostEqual(result['chassis_front_forward_m'] - 0.065, 1.58443)
        self.assertAlmostEqual(result['opening_lateral_m'], -0.04627)
        self.assertAlmostEqual(math.degrees(result['face_yaw_rad']), -0.00373)
        self.assertAlmostEqual(result['entry_heading_rad'], -result['face_yaw_rad'])
        self.assertAlmostEqual((result['left_forward_m'] + result['right_forward_m']) / 2,
                               result['camera_forward_m'])
        self.assertAlmostEqual(math.atan2(result['left_forward_m'] - result['right_forward_m'],
                                          result['leg_center_separation_m']), result['face_yaw_rad'])
        self.assertEqual(result['left_depth_samples'], 100)
        self.assertEqual(result['opening_width_m'], 0.252)

    def test_invalid_corrected_pose_is_rejected(self):
        for key, value in [('forward_bias_m', -10), ('lateral_bias_m', math.nan),
                           ('face_yaw_bias_deg', 90)]:
            bias = dict(forward_bias_m=0, lateral_bias_m=0, face_yaw_bias_deg=0)
            bias[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                apply_target_bias(self.measurement(), bias)

    def test_profile_rejects_missing_nonfinite_and_boolean_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bias.yaml'
            for value in ('{}', '[]', 'forward_bias_m: .nan',
                          'forward_bias_m: true\nlateral_bias_m: 0\nface_yaw_bias_deg: 0'):
                path.write_text(value)
                with self.subTest(value=value), self.assertRaises(ValueError):
                    load_target_bias(path)
            path.write_text('forward_bias_m: 0.04\nlateral_bias_m: 0.02\nface_yaw_bias_deg: 0.9')
            self.assertEqual(load_target_bias(path)['forward_bias_m'], 0.04)

    def test_heading_reference_preserves_positions_and_real_angle_changes(self):
        calibration = Path(__file__).resolve().parents[1]/'config/calibration'
        old = load_target_bias(calibration/'pallet_target_bias_2026_10_31.yaml')
        new = {**old, 'face_yaw_bias_deg':old['face_yaw_bias_deg']+3.13}
        self.assertEqual(old['forward_bias_m'],new['forward_bias_m'])
        self.assertEqual(old['lateral_bias_m'],new['lateral_bias_m'])
        reference = self.measurement()
        reference['face_yaw_rad'] = -math.radians(new['face_yaw_bias_deg'])
        corrected = apply_target_bias(reference,new)
        previous = apply_target_bias(reference,old)
        for key in ('camera_forward_m','chassis_front_forward_m','opening_lateral_m',
                    'opening_width_m','leg_center_separation_m','left_depth_samples','right_depth_samples'):
            self.assertEqual(corrected[key],previous[key])
        self.assertAlmostEqual(corrected['entry_heading_rad'],0)
        reference['face_yaw_rad'] -= math.radians(3)
        changed = apply_target_bias(reference,new)
        self.assertAlmostEqual(math.degrees(changed['entry_heading_rad']),3)

    def test_robot_mount_offset_replaces_old_lateral_bias(self):
        calibration = Path(__file__).resolve().parents[1]/'config/calibration'
        robot = load_target_bias(calibration/'pallet_target_bias_robot_2026_11_11.yaml')
        old = load_target_bias(calibration/'pallet_target_bias_2026_10_31.yaml')
        raw = self.measurement()
        raw['opening_lateral_m'] = .058
        corrected = apply_target_bias(raw, robot)
        self.assertAlmostEqual(corrected['opening_lateral_m'], .083)
        self.assertAlmostEqual(corrected['opening_lateral_m'] -
                               apply_target_bias(raw, old)['opening_lateral_m'],
                               .025 - old['lateral_bias_m'])


if __name__ == '__main__':
    unittest.main()
