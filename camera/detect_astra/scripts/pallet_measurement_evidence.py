"""Bounded, lossless evidence for a locked RGB-D target; no ROS dependencies.

Only successful measurements are copied into RAM (at most 5 Hz, 24 frames).
Disk writes happen when target tracking is paused after planning. The index
states the exact delta from the locked target capture; nearest is never
misrepresented as the capture used by the planner.
"""
from collections import deque
import json
import math
from pathlib import Path
import threading

import cv2
import numpy as np


def json_value(value):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [json_value(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


class MeasurementEvidenceRecorder:
    def __init__(self, directory, *, max_frames=24, interval_ns=200_000_000):
        if max_frames < 2 or interval_ns < 0:
            raise ValueError('invalid evidence buffer bounds')
        self.directory = Path(directory)
        self.frames = deque(maxlen=max_frames)
        self.interval_ns = interval_ns
        self.first = None
        self.lock = threading.Lock()

    def clear(self):
        with self.lock:
            self.frames.clear()
            self.first = None

    def record(self, stamp_ns, rgb, depth, intrinsics, mask, bbox, measurement, metadata):
        with self.lock:
            if self.frames and stamp_ns-self.frames[-1]['stamp_ns'] < self.interval_ns:
                return False
            k, distortion = intrinsics
            frame = dict(stamp_ns=int(stamp_ns), rgb=rgb.copy(), depth=depth.copy(),
                         k=np.asarray(k).copy(), distortion=np.asarray(distortion).copy(),
                         mask=(np.asarray(mask) != 0).astype(np.uint8),
                         bbox=np.asarray(bbox).copy(), measurement=json_value(measurement),
                         metadata=json_value(metadata))
            self.frames.append(frame)
            if self.first is None:
                self.first = frame
            return True

    def flush(self, snapshot_path=None):
        with self.lock:
            frames = list(self.frames)
            first = self.first
        if not frames:
            raise ValueError('no successful RGB-D captures in evidence buffer')
        if snapshot_path is None:
            snapshot_path = self.directory.parent/'plan'/'target_snapshot.json'
        snapshot_path = Path(snapshot_path)
        snapshot = json.loads(snapshot_path.read_text()) if snapshot_path.exists() else None
        capture_ns = (round(float(snapshot['capture_stamp_s'])*1e9)
                      if snapshot and snapshot.get('capture_stamp_s') is not None else None)
        nearest = (min(range(len(frames)), key=lambda i: abs(frames[i]['stamp_ns']-capture_ns))
                   if capture_ns is not None else len(frames)-1)
        chosen = {frames[i]['stamp_ns']: frames[i]
                  for i in {max(0, nearest-1), nearest, min(len(frames)-1, nearest+1), len(frames)-1}}
        if first is not None:
            chosen[first['stamp_ns']] = first
        self.directory.mkdir(parents=True, exist_ok=True)
        entries = []
        for stamp, frame in sorted(chosen.items()):
            name = str(stamp)
            bundle = self.directory/name
            bundle.mkdir(exist_ok=True)
            # Uncompressed NPZ avoids compression CPU work on the Pi. Arrays
            # are exact decoded sensor values, not a depth visualization.
            np.savez(bundle/'raw.npz', rgb_bgr=frame['rgb'], depth=frame['depth'],
                     rgb_k=frame['k'], rgb_distortion=frame['distortion'],
                     tracker_mask=frame['mask'], tracker_bbox_xyxy=frame['bbox'])
            for filename, pixels in [('rgb.png', frame['rgb']),
                                     ('depth_raw.png', frame['depth']),
                                     ('tracker_mask.png', frame['mask']*255)]:
                if not cv2.imwrite(str(bundle/filename), pixels):
                    raise OSError(f'could not save {bundle/filename}')
            overlay = frame['rgb'].copy()
            contours, _ = cv2.findContours(frame['mask'], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
            x1, y1, x2, y2 = np.rint(frame['bbox']).astype(int)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 1)
            if not cv2.imwrite(str(bundle/'tracker_overlay.png'), overlay):
                raise OSError('could not save tracker overlay')
            delta = (stamp-capture_ns)/1e9 if capture_ns is not None else None
            manifest = dict(stamp_ns=stamp, locked_capture_delta_s=delta,
                            measurement=frame['measurement'], metadata=frame['metadata'],
                            rgb_encoding='bgr8', depth_encoding='16UC1', depth_units='millimetres',
                            mask_scope='tracked semantic pallet mask; front-leg selection is recomputed',
                            filtered_measurement=True)
            (bundle/'measurement.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
            entries.append(dict(directory=name, stamp_ns=stamp, locked_capture_delta_s=delta))
        closest_stamp = frames[nearest]['stamp_ns']
        # ROS timestamps serialized as float seconds can lose a few hundred ns.
        exact = capture_ns is not None and abs(closest_stamp-capture_ns) <= 1000
        index = dict(schema='pallet_measurement_evidence_v1',
                     locked_target_snapshot=json_value(snapshot),
                     snapshot_path=str(snapshot_path), locked_capture_stamp_ns=capture_ns,
                     closest_capture_stamp_ns=closest_stamp, exact_locked_capture_available=exact,
                     closest_capture_delta_s=((closest_stamp-capture_ns)/1e9
                                              if capture_ns is not None else None),
                     buffer_max_frames=self.frames.maxlen, sampling_interval_ns=self.interval_ns,
                     selected_captures=entries,
                     note='Bounded RGB-D evidence; nearby captures are explicitly timestamped. '
                          'A filtered heading also depends on prior measurements saved in metadata.')
        tmp = self.directory/'index.json.tmp'
        tmp.write_text(json.dumps(index, indent=2, allow_nan=False)+'\n')
        tmp.replace(self.directory/'index.json')
        return index
