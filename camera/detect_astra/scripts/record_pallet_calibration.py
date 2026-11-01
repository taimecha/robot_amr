#!/usr/bin/env python3
"""Record stationary, fresh /pallet/target samples against taped references.

Read-only: no model inference, camera subscription, parameter changes or motion.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from detected_pallets_interfaces.msg import PalletTarget


PROJECT = Path(__file__).resolve().parents[1]
FIELDS = ('camera_forward_m', 'chassis_front_forward_m', 'opening_lateral_m',
          'face_yaw_rad', 'entry_heading_rad', 'left_forward_m', 'right_forward_m',
          'leg_center_separation_m', 'opening_width_m', 'yolo_confidence',
          'rgb_depth_delta_ms', 'left_depth_samples', 'right_depth_samples')


def summary(samples, truth):
    """Keep variability visible; don't discard valid samples as outliers."""
    result = {}
    for name, expected in truth.items():
        values = [sample[name] for sample in samples]
        errors = sorted(abs(v - expected) for v in values)
        median = statistics.median(values)
        result[name] = {
            'truth': expected, 'median': median, 'median_error': median - expected,
            'sample_stddev': statistics.stdev(values) if len(values) > 1 else 0.0,
            'min': min(values), 'max': max(values),
            'p95_absolute_error': errors[math.ceil(0.95 * len(errors)) - 1],
        }
    return result


class CalibrationRecorder(Node):
    def __init__(self, args):
        super().__init__('pallet_calibration_recorder')
        self.args = args
        self.samples = []
        self.rejected = Counter()
        self.last_stamp_ns = 0
        self.subscription = self.create_subscription(
            PalletTarget, '/pallet/target', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def receive(self, msg):
        if len(self.samples) >= self.args.samples:
            return
        if not msg.valid:
            self.rejected[msg.status or 'invalid measurement'] += 1
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_ms = (self.get_clock().now().nanoseconds - stamp) / 1e6
        if msg.header.frame_id != 'camera_color_optical_frame':
            self.rejected['wrong frame'] += 1
            return
        if not stamp or not -100 <= age_ms <= 500:
            self.rejected['measurement age outside -100..500 ms'] += 1
            return
        if stamp <= self.last_stamp_ns:
            self.rejected['duplicate or out-of-order image'] += 1
            return
        values = {name: getattr(msg, name) for name in FIELDS}
        if not all(math.isfinite(v) for v in values.values()) or msg.camera_forward_m <= 0:
            self.rejected['nonfinite or nonpositive measurement'] += 1
            return
        if msg.left_depth_samples < 1 or msg.right_depth_samples < 1:
            self.rejected['missing leg depth samples'] += 1
            return
        self.last_stamp_ns = stamp
        self.samples.append({
            **values, 'stamp_ns': stamp, 'received_age_ms': age_ms,
            'source_status': msg.status,
            'fork_root_m': msg.chassis_front_forward_m - self.args.fork_root_offset_m,
            'lateral_m': msg.opening_lateral_m,
            'face_yaw_deg': math.degrees(msg.face_yaw_rad),
        })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True, help='Unique pose name')
    parser.add_argument('--fork-root-m', required=True, type=float,
                        help='True forward X gap from fork root plane to opening midpoint')
    parser.add_argument('--lateral-m', required=True, type=float,
                        help='True opening lateral position from vehicle centreline; left positive')
    parser.add_argument('--face-yaw-deg', required=True, type=float,
                        help='True face yaw; left side farther forward is positive')
    parser.add_argument('--fork-root-offset-m', type=float, default=0.065,
                        help='Fork root ahead of chassis front; verify physically (default 0.065)')
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--note', default='', help='Mount/setup/measurement notes')
    parser.add_argument('--output', type=Path,
                        default=PROJECT / 'data/calibration/pallet_samples.jsonl')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) for v in (args.fork_root_m, args.lateral_m,
                                         args.face_yaw_deg, args.fork_root_offset_m,
                                         args.timeout)):
        parser.error('References and timeout must be finite')
    if args.fork_root_m <= 0 or abs(args.face_yaw_deg) > 20 or not 0 <= args.fork_root_offset_m <= 0.3:
        parser.error('Invalid geometry reference')
    if args.samples < 10 or args.timeout <= 0:
        parser.error('Use at least 10 samples and a positive timeout')
    if not args.label.strip():
        parser.error('Pose label must not be empty')

    rclpy.init(args=sys.argv)
    node = CalibrationRecorder(args)
    started = time.monotonic()
    last_report = 0.0
    print('Giữ xe và pallet đứng yên. Đang đọc /pallet/target; bật YOLO để thu mẫu.', flush=True)
    try:
        while rclpy.ok() and len(node.samples) < args.samples and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.get_publishers_info_by_topic('/pallet/target')) > 1:
                raise RuntimeError('Có nhiều publisher /pallet/target; dừng thu để tránh trộn nguồn.')
            if time.monotonic() - last_report > 2:
                print(f'Mẫu hợp lệ: {len(node.samples)}/{args.samples}; '
                      f'bỏ qua: {sum(node.rejected.values())}', flush=True)
                last_report = time.monotonic()
        if len(node.samples) < args.samples:
            raise RuntimeError(f'Chưa đủ mẫu: {len(node.samples)}/{args.samples}. '
                               f'Không ghi tư thế này. Lý do bỏ: {dict(node.rejected)}')
        truth = {'fork_root_m': args.fork_root_m, 'lateral_m': args.lateral_m,
                 'face_yaw_deg': args.face_yaw_deg}
        report = summary(node.samples, truth)
        record = {
            'schema_version': 1, 'label': args.label,
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'duration_s': time.monotonic() - started,
            'truth': truth, 'fork_root_offset_m': args.fork_root_offset_m,
            'note': args.note, 'summary': report,
            'rejected': dict(node.rejected), 'samples': node.samples,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('a', encoding='utf-8') as output:
            output.write(json.dumps(record, allow_nan=False, ensure_ascii=False) + '\n')
        for name, scale, unit in (('fork_root_m', 100, 'cm'), ('lateral_m', 100, 'cm'),
                                  ('face_yaw_deg', 1, 'độ')):
            row = report[name]
            print(f'{name}: trung vị={row["median"] * scale:+.3f} {unit}; '
                  f'sai số={row["median_error"] * scale:+.3f} {unit}; '
                  f'độ lệch chuẩn={row["sample_stddev"] * scale:.3f} {unit}; '
                  f'P95 |sai số|={row["p95_absolute_error"] * scale:.3f} {unit}')
        print(f'Đã ghi {args.label}: {args.output}\nChưa thay đổi hệ số hiệu chỉnh.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc) or 'Đã hủy thu mẫu.', file=sys.stderr)
        sys.exit(1)
