"""Empirical residual correction of a measured front-opening pose.

This corrects derived horizontal geometry, not raw sensor depth or camera TF.
"""
import math
from pathlib import Path

import yaml


def load_target_bias(path):
    profile = yaml.safe_load(Path(path).read_text())
    if not isinstance(profile, dict):
        raise ValueError('Target bias profile must be a mapping')
    values = {}
    for key in ('forward_bias_m', 'lateral_bias_m', 'face_yaw_bias_deg'):
        value = profile.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Target bias {key} must be a finite number')
        values[key] = float(value)
    return values


def apply_target_bias(measured, bias):
    result = dict(measured)
    forward = measured['camera_forward_m'] + bias['forward_bias_m']
    lateral = measured['opening_lateral_m'] + bias['lateral_bias_m']
    yaw = measured['face_yaw_rad'] + math.radians(bias['face_yaw_bias_deg'])
    separation = measured['leg_center_separation_m']
    if not all(math.isfinite(v) for v in (forward, lateral, yaw, separation)):
        raise ValueError('Nonfinite corrected target geometry')
    if forward <= 0 or separation <= 0 or abs(yaw) > math.radians(20):
        raise ValueError('Corrected target outside geometry limits')
    # Left/right forwards are corrected plane estimates. Keep their midpoint
    # and yaw consistent with the corrected opening pose, rather than leaving
    # uncorrected diagnostics mixed with a corrected face angle.
    half_difference = math.tan(yaw) * separation / 2
    result.update(
        camera_forward_m=forward,
        chassis_front_forward_m=measured['chassis_front_forward_m'] + bias['forward_bias_m'],
        opening_lateral_m=lateral,
        face_yaw_rad=yaw,
        entry_heading_rad=-yaw,
        left_forward_m=forward + half_difference,
        right_forward_m=forward - half_difference,
    )
    return result
