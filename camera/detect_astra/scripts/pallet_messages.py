"""Shared measured-target messages, without an inference dependency."""
import math
from detected_pallets_interfaces.msg import PalletTarget
from astra_rgbd_stream import OPTICAL_FRAME

def empty_message(stamp, status, delta_ms=math.nan):
    msg = PalletTarget()
    msg.header.stamp = stamp
    msg.header.frame_id = OPTICAL_FRAME
    msg.valid = False
    msg.status = status
    for name in ("camera_forward_m", "chassis_front_forward_m", "opening_lateral_m", "face_yaw_rad",
                 "entry_heading_rad", "opening_width_m", "opening_height_estimate_m", "leg_center_separation_m",
                 "left_forward_m", "right_forward_m", "yolo_confidence"):
        setattr(msg, name, math.nan)
    msg.rgb_depth_delta_ms = float(delta_ms)
    return msg

def confirmed(current, previous):
    return (abs(current['camera_forward_m'] - previous['camera_forward_m']) < 0.06
            and abs(current['opening_lateral_m'] - previous['opening_lateral_m']) < 0.035
            and abs(current['face_yaw_rad'] - previous['face_yaw_rad']) < math.radians(5))
