"""The displayed camera offset must be applied once before a real plan locks."""

import unittest

from r3_camera_preview import locked_snapshot


class MeasurementReportTest(unittest.TestCase):
    def inputs(self):
        session = dict(
            provenance='stationary_real_camera', stationary_verified=True,
            adapter='corrected_base_scalars', start_odom=[0., 0., 0.],
            yolo_off_stamp_s=10., pallet_measurement_stopped_after_lock=True,
            post_yolo_off_samples=[dict(
                capture_stamp_s=10.1 + i*.1, odom_at_capture=[0., 0., 0.],
                forward_base_m=1.4, camera_lateral_raw_m=.058,
                lateral_base_m=.083, entry_heading_rad=0.) for i in range(5)])
        calibration = dict(
            confirmed_real=True, yaw_accuracy_bound_deg=.3,
            accuracy_evidence='independent measurement',
            corrected_scalar_adapter_confirmed=True,
            applied_bias={'lateral_bias_m': .025})
        return session, calibration

    def test_report_shows_one_mount_offset(self):
        session, calibration = self.inputs()
        snapshot = locked_snapshot(session, calibration)
        report = snapshot['measurement_report']
        self.assertAlmostEqual(report['camera_lateral_raw_m'], .058)
        self.assertAlmostEqual(report['camera_mount_offset_m'], .025)
        self.assertAlmostEqual(report['fork_lateral_corrected_m'], .083)

    def test_double_count_is_rejected(self):
        session, calibration = self.inputs()
        for sample in session['post_yolo_off_samples']:
            sample['lateral_base_m'] += .025
        with self.assertRaisesRegex(ValueError, 'added twice'):
            locked_snapshot(session, calibration)


if __name__ == '__main__':
    unittest.main()
