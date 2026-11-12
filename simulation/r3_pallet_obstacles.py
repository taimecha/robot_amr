"""Project range returns into odom and exempt only the frozen pallet footprint.

The pallet exclusion is geometric, not semantic: a new object inside exactly
the same small volume cannot be distinguished after YOLO stops.
"""

import math

from r3_core import compose, inverse


def pallet_return(x, y, target, geometry, margin=0.015):
    """True for a point on the one recognized pallet body/legs in odom."""
    if not all(math.isfinite(v) for v in (x, y, *target, margin)) or margin < 0:
        raise ValueError('invalid pallet exclusion geometry')
    px, py, _ = compose(inverse(target), (x, y, 0.))
    return (-margin <= px <= geometry['pallet_length'] + margin
            and abs(py) <= geometry['pallet_width'] / 2 + margin)


def classify_ranges(ranges, angle_min, angle_increment, sensor_odom, target, geometry):
    """Return masked ranges and counts; retain every finite nonpallet return."""
    if not all(math.isfinite(v) for v in (angle_min, angle_increment, *sensor_odom)):
        raise ValueError('invalid scan transform')
    if angle_increment == 0:
        raise ValueError('invalid scan angle increment')
    filtered = list(ranges)
    pallet_count = other_count = 0
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0:
            continue
        ray = angle_min + i * angle_increment
        x, y, _ = compose(sensor_odom, (distance * math.cos(ray),
                                         distance * math.sin(ray), 0.))
        if pallet_return(x, y, target, geometry):
            filtered[i] = math.inf
            pallet_count += 1
        else:
            other_count += 1
    return filtered, pallet_count, other_count
