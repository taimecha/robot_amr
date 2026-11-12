"""Raw measurement evidence retains sensor values and never hides stamp mismatch."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from pallet_measurement_evidence import MeasurementEvidenceRecorder


class MeasurementEvidenceTest(unittest.TestCase):
    def inputs(self):
        rgb = np.arange(6*8*3, dtype=np.uint8).reshape(6, 8, 3)
        depth = np.arange(6*8, dtype=np.uint16).reshape(6, 8)+1000
        mask = np.zeros((6, 8), np.uint8)
        mask[1:5, 2:7] = 255
        return rgb, depth, (np.eye(3), np.zeros(5)), mask, [2., 1., 7., 5.]

    def test_roundtrip_preserves_raw_depth_intrinsics_and_measurement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            recorder = MeasurementEvidenceRecorder(root/'camera_evidence')
            rgb, depth, intrinsics, mask, bbox = self.inputs()
            expected_rgb, expected_depth = rgb.copy(), depth.copy()
            recorder.record(123_456_789_123, rgb, depth, intrinsics, mask, bbox,
                            {'opening_lateral_m': .05, 'height': float('nan')},
                            {'config': {'pitch_deg': 20., 'profile': Path('profile.yaml')},
                             'heading_filter_history': [{'entry_heading_rad': .01}]})
            rgb[:] = 0
            depth[:] = 0
            (root/'plan').mkdir()
            (root/'plan'/'target_snapshot.json').write_text(json.dumps(
                {'capture_stamp_s': 123.456789123, 'measurement_report': {'lateral': .05}}))
            index = recorder.flush()
            self.assertTrue(index['exact_locked_capture_available'])
            bundle = root/'camera_evidence'/index['selected_captures'][0]['directory']
            with np.load(bundle/'raw.npz', allow_pickle=False) as raw:
                np.testing.assert_array_equal(raw['rgb_bgr'], expected_rgb)
                np.testing.assert_array_equal(raw['depth'], expected_depth)
                np.testing.assert_array_equal(raw['rgb_k'], intrinsics[0])
                np.testing.assert_array_equal(raw['tracker_mask'], mask != 0)
            np.testing.assert_array_equal(cv2.imread(str(bundle/'depth_raw.png'),
                                                     cv2.IMREAD_UNCHANGED), expected_depth)
            saved = json.loads((bundle/'measurement.json').read_text())
            self.assertEqual(saved['measurement']['opening_lateral_m'], .05)
            self.assertIsNone(saved['measurement']['height'])
            self.assertEqual(saved['metadata']['config']['profile'], 'profile.yaml')

    def test_bounded_sampling_and_nearest_capture_are_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            recorder = MeasurementEvidenceRecorder(root/'camera_evidence', max_frames=3,
                                                    interval_ns=200_000_000)
            values = self.inputs()
            for stamp in (1_000_000_000, 1_100_000_000, 1_200_000_000,
                          1_400_000_000, 1_600_000_000, 1_800_000_000):
                recorder.record(stamp, *values, {}, {})
            self.assertEqual(len(recorder.frames), 3)
            self.assertEqual(recorder.first['stamp_ns'], 1_000_000_000)
            snapshot = root/'target.json'
            snapshot.write_text(json.dumps({'capture_stamp_s': 1.51}))
            index = recorder.flush(snapshot)
            self.assertFalse(index['exact_locked_capture_available'])
            self.assertEqual(index['closest_capture_stamp_ns'], 1_600_000_000)
            self.assertAlmostEqual(index['closest_capture_delta_s'], .09)
            self.assertIn(1_000_000_000, [x['stamp_ns'] for x in index['selected_captures']])
            recorder.clear()
            with self.assertRaisesRegex(ValueError, 'no successful'):
                recorder.flush()

    def test_write_failure_is_not_reported_as_saved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'not_a_directory'
            root.write_text('file')
            recorder = MeasurementEvidenceRecorder(root)
            recorder.record(1, *self.inputs(), {}, {})
            with self.assertRaises(OSError):
                recorder.flush()


if __name__ == '__main__':
    unittest.main()
