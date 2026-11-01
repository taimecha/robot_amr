#!/usr/bin/env python3
"""Experimental YOLO11 segmentation + registered Astra depth; never AGV control.

Source /opt/ros/jazzy/setup.bash, then use a Python with ROS and ultralytics.
Q quits; S saves the displayed pair, original data, masks, and measurements.
Distance is median optical Z inside the eroded mask, NOT Euclidean range.
XYZ uses the actual depth of a marked valid pixel, NOT the mask median.
"""
import argparse
from datetime import datetime, timezone
import importlib
import json
import os
from pathlib import Path
import sys
import threading
import time

# Report all dependencies in the selected interpreter, without installing anything.
_missing = []
for _name in ('rclpy', 'sensor_msgs', 'cv2', 'numpy', 'torch', 'ultralytics'):
    try:
        globals()[_name] = importlib.import_module(_name)
    except Exception as _exc:
        _missing.append(f'{_name}: {type(_exc).__name__}: {_exc}')
if _missing:
    raise SystemExit(f'Python: {sys.executable}\n' + '\n'.join(_missing) +
                     '\nSource /opt/ros/jazzy/setup.bash and use your existing YOLO Python.'
                     '\nNo packages have been installed or upgraded.')

import cv2
import numpy as np
import rclpy
import torch
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import remove_ros_args
from ultralytics import YOLO
from astra_rgbd_stream import (stamp_ns, decode_image, image_metadata, info_metadata,
                               validate_intrinsics, publisher_policy, CameraReceiver)

WINDOW = 'Astra Pro YOLO Depth'
SHAPE = (480, 640)
EXPERIMENTAL = 'XYZ ESTIMATED - CALIBRATION UNVERIFIED - NOT FOR AGV CONTROL'
OPTICAL_FRAME = 'camera_color_optical_frame'


def measure_mask(mask, depth, args, intrinsics):
    """Return statistics, candidate mask, valid sampling mask; never mutate inputs."""
    radius = args.erode_px
    candidate = cv2.erode(mask.astype(np.uint8), np.ones((2*radius+1, 2*radius+1), np.uint8),
                          borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    used = candidate & (depth > 0) & (depth >= args.min_depth_mm) & (depth <= args.max_depth_mm)
    total, count = int(candidate.sum()), int(used.sum())
    ratio = count / total if total else 0.0
    record = dict(candidate_pixels=total, valid_pixels=count, valid_ratio=ratio,
                  median_z_m=None, pixel_uv=None, pixel_z_m=None, xyz_m=None,
                  xyz_frame_id=None)
    if count < args.min_valid_pixels or ratio < args.min_valid_ratio:
        record['status'] = 'Invalid depth'
        return record, candidate, used
    median = float(np.median(depth[used]))
    ys, xs = np.nonzero(used)
    # Select a real measured pixel nearest the median depth; break ties by
    # proximity to the sampling region's center. No synthetic centroid depth.
    center_x, center_y = xs.mean(), ys.mean()
    order = np.lexsort(((xs-center_x)**2 + (ys-center_y)**2,
                        np.abs(depth[ys, xs].astype(np.float64) - median)))
    idx = order[0]
    u, v = int(xs[idx]), int(ys[idx])
    z = float(depth[v, u]) / 1000
    record.update(status='Experimental depth', median_z_m=median/1000,
                  pixel_uv=[u, v], pixel_z_m=z)
    if intrinsics is not None:
        k, distortion = intrinsics
        ray = cv2.undistortPoints(np.array([[[u, v]]], np.float64), k, distortion)[0, 0]
        if np.isfinite(ray).all():
            record['xyz_m'] = [float(ray[0]*z), float(ray[1]*z), z]
            record['xyz_frame_id'] = OPTICAL_FRAME
    return record, candidate, used

def pixel_points(uv, z, intrinsics):
    k, distortion = intrinsics
    rays = cv2.undistortPoints(np.asarray(uv, np.float64).reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    return np.column_stack((rays*z[:, None], z))


def fit_edge_plane(uv, z, intrinsics):
    """Robust local 3D plane, never a claim of pallet semantic identity."""
    points = pixel_points(uv, z, intrinsics)
    if len(points) < 40 or not np.isfinite(points).all():
        return None
    rng = np.random.default_rng(17)
    best = None
    for _ in range(40):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        offset = -normal @ a
        support = np.abs(points @ normal + offset) < 0.015
        if best is None or support.sum() > best.sum():
            best = support
    if best is None or best.mean() < .75:
        return None
    center = points[best].mean(axis=0)
    _, singular, vt = np.linalg.svd(points[best]-center, full_matrices=False)
    if singular[1] / np.sqrt(best.sum()) < .003:
        return None  # Insufficient surface extent: points nearly collinear.
    normal = vt[-1]
    offset = float(-normal @ center)
    residual = np.abs(points @ normal + offset)
    inliers = residual < .015
    rms = float(np.sqrt(np.mean(residual[inliers]**2)))
    # Camera-facing surface only; no gravity reference, so not proven vertical.
    if inliers.mean() < .75 or rms > .010 or abs(normal[2]) < .75:
        return None
    return normal, offset, float(inliers.mean()), rms, float(np.median(z[inliers]))


def continuous_lower_edge(options, reference, bbox):
    """Fallback only to a longer lower boundary of the same supported surface."""
    if reference.get('coverage', 1.) >= .65 and reference.get('continuity', 1.) >= .9:
        return None
    ab = reference['ab']
    n, d = reference['plane'][:2]
    matches = []
    for option in options:
        other = option['ab']
        dy = float(other[:, 1].mean()-ab[:, 1].mean())
        normal, offset = option['plane'][:2]
        sign = 1 if np.dot(n, normal) >= 0 else -1
        overlap = max(0., min(ab[1, 0], other[1, 0])-max(ab[0, 0], other[0, 0]))
        if (4 <= dy <= min(60., .45*(bbox[3]-bbox[1]))
                and option.get('coverage', 0) >= .65
                and option.get('continuity', 0) >= .9
                and option.get('max_gap', 1) <= .08
                and np.linalg.norm(other[1]-other[0]) >= 1.4*np.linalg.norm(ab[1]-ab[0])
                and overlap >= .7*(ab[1, 0]-ab[0, 0])
                and abs(np.dot(n, normal)) >= .97
                and abs(d-sign*offset) <= .025
                and abs(reference['z']-option['z']) <= .06):
            matches.append(option)
    return max(matches, key=lambda o: o['score']) if matches else None


class FrontEdgeTracker:
    """Experimental RGB+plane-supported candidates, not semantic front-edge detection."""
    def __init__(self):
        self.tracks = {}
        self.next_id = 0

    def select(self, rgb, depth, mask, used, intrinsics, bbox, cls, stamp):
        result = dict(status='FRONT EDGE NOT FOUND', semantic_front_verified=False,
                      confidence_kind='heuristic support score, not probability',
                      A_uv=None, B_uv=None, M_uv=None, sample_uv=None, xyz_m=None,
                      M_xyz_m=None, confidence=0.0, reason='no qualified RGB/Depth edge')
        if intrinsics is None:
            result['reason'] = 'registered RGB intrinsics unavailable'
            return result
        self.tracks = {k: v for k, v in self.tracks.items() if 0 <= stamp-v['stamp'] < 2.0}
        def iou(a, b):
            wh = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2]))
            inter = float(np.prod(wh))
            return inter/max(1., float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2]))-inter)
        matches = [(iou(bbox, t['bbox']), k) for k, t in self.tracks.items()
                   if t['cls'] == cls and t['stamp'] != stamp]
        matches.sort(reverse=True)
        if len(matches) > 1 and matches[1][0] > .5:
            result['reason'] = 'ambiguous instance association'
            return result
        track_id = matches[0][1] if matches and matches[0][0] > .5 else None
        previous = self.tracks.get(track_id)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 120)
        edges[~mask] = 0
        minimum = max(35, int((bbox[2]-bbox[0])*.35))
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=25,
                                minLineLength=minimum, maxLineGap=8)
        options = []
        diagnostics = dict(lines=0, mask_supported=0, plane_supported=0, depth_boundaries=0)
        result['diagnostics'] = diagnostics
        # Re-evaluate the old segment using current RGB and Depth, never old XYZ.
        lines = [] if lines is None else list(np.asarray(lines).reshape(-1, 4))
        if previous is not None:
            lines.insert(0, previous['ab'].reshape(4))
        edge_support = cv2.dilate(edges, np.ones((5, 5), np.uint8))
        if len(lines):
            for line in sorted(np.asarray(lines).reshape(-1, 4), key=lambda l: -np.hypot(l[2]-l[0], l[3]-l[1]))[:40]:
                diagnostics['lines'] += 1
                a, b = line[:2].astype(float), line[2:].astype(float)
                if a[0] > b[0]:
                    a, b = b, a
                direction = b-a
                length = np.linalg.norm(direction)
                if abs(direction[1]) > .27*abs(direction[0]):
                    continue
                tangent = direction/length
                perpendicular = np.array([-tangent[1], tangent[0]])
                along = a + np.linspace(.05, .95, 60)[:, None]*direction
                samples = np.rint(along).astype(int)
                if mask[samples[:, 1], samples[:, 0]].mean() < .9:
                    continue
                supported = edge_support[samples[:, 1], samples[:, 0]] > 0
                continuity = float(supported.mean())
                longest_gap = run = 0
                for present in supported:
                    run = 0 if present else run+1
                    longest_gap = max(longest_gap, run)
                if continuity < .6:
                    continue
                diagnostics['mask_supported'] += 1
                for side in (-1, 1):
                    band = np.rint(along[:, None, :] + side*perpendicular*np.arange(2, 14, 2)[None, :, None]).astype(int).reshape(-1, 2)
                    band = band[(band[:, 0]>=0)&(band[:, 0]<640)&(band[:, 1]>=0)&(band[:, 1]<480)]
                    valid = used[band[:, 1], band[:, 0]]
                    if valid.sum() < 60 or valid.mean() < .55:
                        continue
                    uv = band[valid]
                    z = depth[uv[:, 1], uv[:, 0]].astype(float)/1000
                    plane = fit_edge_plane(uv, z, intrinsics)
                    if plane is None:
                        continue
                    diagnostics['plane_supported'] += 1
                    normal, offset, support, rms, plane_z = plane
                    opposite = np.rint(along - side*perpendicular*8).astype(int)
                    ok = (opposite[:, 0]>=0)&(opposite[:, 0]<640)&(opposite[:, 1]>=0)&(opposite[:, 1]<480)
                    opposite = opposite[ok]
                    valid_other = depth[opposite[:, 1], opposite[:, 0]] > 0
                    # Require measured depth discontinuity, not just a texture edge or holes.
                    if valid_other.sum() < 20:
                        continue
                    other = opposite[valid_other]
                    pts = pixel_points(other, depth[other[:, 1], other[:, 0]]/1000., intrinsics)
                    discontinuity = float((np.abs(pts@normal+offset) > .04).mean())
                    if discontinuity < .6:
                        continue
                    diagnostics['depth_boundaries'] += 1
                    coverage = min(1., length/max(1., bbox[2]-bbox[0]))
                    score = float(support*valid.mean()*discontinuity*coverage)
                    option = dict(ab=np.array([a, b]), plane=plane, side=side,
                                  perpendicular=perpendicular, score=score, z=plane_z,
                                  coverage=coverage, continuity=continuity, max_gap=longest_gap/len(supported))
                    if not any(np.max(np.linalg.norm(option['ab']-o['ab'], axis=1)) < 10 for o in options):
                        options.append(option)
        if not options:
            # Keep identity briefly across dropped frames, but never reuse old XYZ.
            result['reason'] = 'no current edge with sufficient plane/depth support'
            return result
        # Prefer the nearest supported surface, not the smallest raw depth pixel.
        options.sort(key=lambda o: (o['z'], -o['score']))
        close = [o for o in options if abs(o['z']-options[0]['z']) < .06]
        close.sort(key=lambda o: -o['score'])
        chosen = close[0]
        fallback_selected = False
        pending = None
        if previous is not None:
            consistent = [o for o in close if np.max(np.linalg.norm(o['ab']-previous['ab'], axis=1)) < 15
                          and abs(o['z']-previous['z']) < .06]
            reference = min(consistent, key=lambda o: np.linalg.norm(o['ab']-previous['ab'])) if consistent else previous
            lower = continuous_lower_edge(close, reference, bbox)
            if lower is not None:
                old_pending = previous.get('lower_pending')
                same = old_pending is not None and np.max(np.linalg.norm(lower['ab']-old_pending['ab'], axis=1)) < 8
                pending = dict(ab=lower['ab'].copy(), hits=old_pending['hits']+1 if same else 1)
                if pending['hits'] >= 3:
                    chosen = lower
                    previous = None  # Never interpolate between two different edges.
                    fallback_selected = True
                    pending = None
                else:
                    previous['lower_pending'] = pending
            else:
                previous.pop('lower_pending', None)
            if not fallback_selected:
                if consistent:
                    chosen = reference
                else:
                    result['reason'] = 'waiting for supported tracked edge or confirmed continuous lower edge'
                    return result
        if previous is None and not fallback_selected and len(close) > 1 and close[0]['score'] < 1.3*close[1]['score']:
            result['reason'] = 'multiple equally near supported edges; no dominant candidate'
            return result
        if track_id is None:
            track_id = self.next_id
            self.next_id += 1
        hits = previous['hits']+1 if previous is not None else (3 if fallback_selected else 1)
        # EMA only for a geometrically consistent segment; measurement uses current depth.
        ab = chosen['ab']
        if previous is not None:
            shift = np.max(np.linalg.norm(chosen['ab']-previous['ab'], axis=1))
            ab = previous['ab'].copy() if shift <= 3 else .8*previous['ab'] + .2*chosen['ab']
        self.tracks[track_id] = dict(ab=ab, z=chosen['z'], hits=hits, cls=cls,
                                     bbox=bbox.copy(), stamp=stamp, plane=chosen['plane'],
                                     coverage=chosen['coverage'], continuity=chosen['continuity'],
                                     lower_pending=pending,
                                     selection='continuous lower edge' if fallback_selected else (previous.get('selection', 'automatic edge') if previous else 'automatic edge'))
        midpoint = ab.mean(axis=0)
        normal, offset, support, rms, _ = chosen['plane']
        result.update(A_uv=ab[0].tolist(), B_uv=ab[1].tolist(), M_uv=midpoint.tolist(),
                      raw_AB_uv=chosen['ab'].tolist(), confidence=chosen['score'],
                      selection=self.tracks[track_id]['selection'],
                      edge_coverage=chosen['coverage'], edge_continuity=chosen['continuity'],
                      stable_frames=hits, track_id=track_id,
                      plane_normal=normal.tolist(), plane_offset_m=offset,
                      plane_inlier_ratio=support, plane_rms_m=rms,
                      plane_orientation='camera-facing; gravity/vertical not verified')
        if hits < 3:
            result['reason'] = 'candidate pending 3 consistent frames'
            return result
        result.update(status='FRONT EDGE CANDIDATE - UNVERIFIED', reason='semantic identity not verified')
        # Search <=8 pixels from M, on the selected side and measured plane only.
        u0, v0 = np.rint(midpoint).astype(int)
        yy, xx = np.mgrid[max(0,v0-8):min(480,v0+9), max(0,u0-8):min(640,u0+9)]
        uv = np.column_stack((xx.ravel(), yy.ravel()))
        uv = uv[used[uv[:, 1], uv[:, 0]]]
        if len(uv):
            delta = uv-midpoint
            uv = uv[(np.linalg.norm(delta, axis=1)<=8) & (delta@chosen['perpendicular']*chosen['side']>=0)]
        if len(uv):
            pts = pixel_points(uv, depth[uv[:, 1], uv[:, 0]]/1000., intrinsics)
            on_plane = np.abs(pts@normal+offset) < .015
            uv, pts = uv[on_plane], pts[on_plane]
            if len(uv):
                distances = np.linalg.norm(uv-midpoint, axis=1)
                last_sample = previous.get('sample_uv') if previous else None
                if last_sample is not None:
                    same = np.flatnonzero(np.all(uv == last_sample, axis=1))
                    idx = int(same[0]) if len(same) else int(np.argmin(distances))
                else:
                    idx = int(np.argmin(distances))
                self.tracks[track_id]['sample_uv'] = uv[idx].tolist()
                result.update(sample_uv=uv[idx].tolist(), xyz_m=pts[idx].tolist(),
                              sample_z_m=float(pts[idx, 2]),
                              xyz_target='actual sampled pixel near M; NOT interpolated M')
        if result['xyz_m'] is None:
            result['reason'] = 'no valid on-plane pixel near M; no plane interpolation'
        return result


def put(image, text, x, y, color=(255, 255, 255), scale=0.48):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


class Processor:
    def __init__(self, args, model):
        self.args, self.model = args, model
        self.front_edges = FrontEdgeTracker()
        self.manual_ab = []
        self.manual_bbox = None
        self.manual_invalid = False
        self.show_old_point = False
        self.rgb_view = np.zeros((*SHAPE, 3), np.uint8)
        self.depth_view = self.rgb_view.copy()
        self.snapshot = None
        self.summary = 'No accepted pair displayed'
        self.xyz_status = 'XYZ unavailable'
        self.ratios = 'No masks'
        self.error = ''

    def mouse(self, event, x, y, flags, userdata):
        # OpenCV maps mouse coordinates to the full displayed canvas.
        if not getattr(self.args, 'manual_edge', False):
            return
        if event != cv2.EVENT_LBUTTONDOWN or not (0 <= x < 640 and 32 <= y < 512):
            return
        if len(self.manual_ab) == 2:
            self.manual_ab = []
        self.manual_ab.append([x, y-32])
        self.manual_bbox = None
        self.manual_invalid = False
        print('Manual edge: click A then B on RGB. R returns to automatic candidates.', self.manual_ab)

    def manual_measure(self, depth, mask, used, intrinsics, bbox):
        result = dict(status='MANUAL EDGE - UNVERIFIED', reason='select A and B on RGB',
                      semantic_front_verified=False, xyz_m=None,
                      confidence=None, confidence_kind='operator selection; not automatic confidence')
        if len(self.manual_ab) != 2:
            return result
        ab = np.asarray(self.manual_ab, float)
        midpoint = ab.mean(axis=0)
        u, v = np.rint(midpoint).astype(int)
        result.update(A_uv=ab[0].tolist(), B_uv=ab[1].tolist(), M_uv=midpoint.tolist(),
                      sample_uv=None, M_xyz_m=None,
                      xyz_target='actual rounded midpoint pixel; fixed image coordinates')
        if not mask[v, u]:
            result['reason'] = 'manual midpoint outside this instance mask'
            return result
        if self.manual_bbox is None:
            self.manual_bbox = bbox.copy()
        if np.max(np.abs(bbox-self.manual_bbox)) > 20:
            self.manual_invalid = True
        if self.manual_invalid:
            result['reason'] = 'pallet/bounding box moved; reselect A/B'
            return result
        if intrinsics is None:
            result['reason'] = 'registered RGB intrinsics unavailable'
            return result
        valid = mask & (depth >= self.args.min_depth_mm) & (depth <= self.args.max_depth_mm) & (depth > 0)
        if not valid[v, u]:
            result['reason'] = 'midpoint has no valid Depth; no alternate pixel used'
            return result
        patch = depth[max(0,v-2):v+3, max(0,u-2):u+3]
        support = valid[max(0,v-2):v+3, max(0,u-2):u+3]
        values = patch[support].astype(float)
        zmm = float(depth[v, u])
        if len(values) < 5 or abs(zmm-np.median(values)) > 30:
            result['reason'] = 'midpoint Depth lacks consistent local support'
            return result
        xyz = pixel_points([[u, v]], np.array([zmm/1000]), intrinsics)[0]
        if not np.isfinite(xyz).all():
            result['reason'] = 'invalid projection'
            return result
        result.update(sample_uv=[int(u), int(v)], sample_z_m=zmm/1000, xyz_m=xyz.tolist(),
                      reason='operator-selected edge; measured midpoint pixel; calibration unverified')
        return result

    def process(self, pair):
        rgb_msg, depth_msg, dt, ci, di, registration = pair
        if (rgb_msg.height, rgb_msg.width) != SHAPE or (depth_msg.height, depth_msg.width) != SHAPE:
            raise ValueError('RGB and Depth must both be 640x480; no resizing permitted')
        if depth_msg.encoding != '16UC1':
            raise ValueError(f'Expected 16UC1 millimeters, got {depth_msg.encoding}')
        rgb = decode_image(rgb_msg)
        depth = decode_image(depth_msg, depth=True)
        intrinsics, xyz_status = validate_intrinsics(rgb_msg, depth_msg, ci, registration)
        result = self.model.predict(source=rgb, conf=self.args.conf, imgsz=640,
                                    retina_masks=True, device=self.args.device, verbose=False)[0]
        if tuple(result.orig_shape) != SHAPE:
            raise ValueError('YOLO result does not reference original RGB dimensions')
        # retina_masks removes letterbox padding in Ultralytics. Do not stretch a
        # lower-resolution/padded mask. A shape mismatch is a hard error.
        count = 0 if result.boxes is None else len(result.boxes)
        if count and (result.masks is None or len(result.masks.data) != count):
            raise ValueError('Missing segmentation masks for detections')
        masks = [] if not count else result.masks.data.cpu().numpy() > 0.5
        if count and masks.shape[1:] != SHAPE:
            raise ValueError('Expected retina_masks in original 640x480 coordinates')
        rgb_view = rgb.copy()
        normalized = np.clip(depth.astype(np.float32)/self.args.max_depth_mm*255, 0, 255).astype(np.uint8)
        depth_view = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        depth_view[depth == 0] = (255, 0, 255)
        records, files, labels = [], {}, []
        for i, mask in enumerate(masks):
            box = result.boxes[i]
            cls, confidence = int(box.cls.item()), float(box.conf.item())
            name = result.names[cls]
            record, candidate, used = measure_mask(mask, depth, self.args, intrinsics)
            old_pixel = record['pixel_uv']
            old_xyz = record['xyz_m']
            bbox = box.xyxy[0].cpu().numpy()
            edge = self.front_edges.select(rgb, depth, mask, used, intrinsics, bbox, cls,
                                           stamp_ns(rgb_msg)/1e9) if record['median_z_m'] is not None else dict(
                status='FRONT EDGE NOT FOUND', reason='insufficient valid Depth', xyz_m=None)
            if self.manual_ab:
                edge = self.manual_measure(depth, mask, used, intrinsics, bbox)
            record.update(front_edge=edge, old_pixel_uv=old_pixel, old_xyz_m=old_xyz,
                          pixel_uv=edge.get('sample_uv'), pixel_z_m=edge.get('sample_z_m'),
                          xyz_m=edge.get('xyz_m'), xyz_frame_id=OPTICAL_FRAME if edge.get('xyz_m') else None)
            record.update(instance=i, class_id=cls, class_name=name, confidence=confidence)
            records.append(record)
            # Original mask remains unchanged for RGB; erosion is sampling only.
            color = np.array(((40 + i*83) % 200 + 30, (130 + i*47) % 200 + 30,
                              (80 + i*67) % 200 + 30), np.uint8)
            rgb_view[mask] = (0.65*rgb_view[mask] + 0.35*color).astype(np.uint8)
            contours, _ = cv2.findContours(candidate.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(depth_view, contours, -1, (0, 255, 255), 1)
            depth_view[used] = (0.4*depth_view[used] + 0.6*np.array([0, 255, 0])).astype(np.uint8)
            text = 'Invalid depth' if record['median_z_m'] is None else f"Zmed={record['median_z_m']:.3f} m"
            label = f'#{i} {name} {confidence:.2f} | {text} | valid={record["valid_ratio"]:.0%}'
            xyxy = box.xyxy[0].cpu().numpy()
            x = max(2, min(250, int(xyxy[0])))
            y = max(20, min(455, int(xyxy[1])))
            labels.append((label, x, y, record))
            if edge.get('A_uv') is not None:
                a, b, m = (tuple(np.rint(edge[k]).astype(int)) for k in ('A_uv', 'B_uv', 'M_uv'))
                for view in (rgb_view, depth_view):
                    cv2.line(view, a, b, (0, 255, 255), 2)
                    for point, title in ((a, 'A'), (b, 'B'), (m, 'M')):
                        cv2.drawMarker(view, point, (0, 0, 255), cv2.MARKER_CROSS, 13, 2)
                        put(view, title, point[0]+5, max(12, point[1]-6), scale=.4)
            if self.show_old_point and old_pixel is not None:
                for view in (rgb_view, depth_view):
                    cv2.circle(view, tuple(old_pixel), 5, (255, 0, 255), 1)
                    put(view, 'old', old_pixel[0]+6, old_pixel[1], scale=.4)
            if record['pixel_uv'] is not None:
                uv = tuple(record['pixel_uv'])
                for view in (rgb_view, depth_view):
                    cv2.drawMarker(view, uv, (0, 0, 0), cv2.MARKER_CROSS, 19, 4)
                    cv2.drawMarker(view, uv, (255, 255, 255), cv2.MARKER_CROSS, 17, 1)
                    put(view, f'P#{i}', uv[0]+8, max(16, uv[1]-8))
            files[f'mask_{i}.png'] = mask.astype(np.uint8)*255
            files[f'candidate_{i}.png'] = candidate.astype(np.uint8)*255
            files[f'used_pixels_{i}.png'] = used.astype(np.uint8)*255
        for label, x, y, record in labels:
            put(rgb_view, label, x, y, scale=0.43)
            put(rgb_view, record['front_edge']['status'], x, min(478, y+36), (0, 220, 255), scale=.40)
            xyz = record['xyz_m']
            if xyz is not None:
                put(rgb_view, 'P: X={:.3f} m Y={:.3f} m Z={:.3f} m'.format(*xyz), x, min(478, y+18), scale=0.43)
            else:
                put(rgb_view, 'XYZ unavailable', x, min(478, y+18), scale=0.43)
        self.rgb_view, self.depth_view = rgb_view, depth_view
        self.summary = f'displayed dt={dt:+.2f} ms | RGB/Depth 640x480 | detections={count}'
        self.xyz_status = (xyz_status + ' | edge XYZ available') if any(r['xyz_m'] is not None for r in records) else 'XYZ unavailable: ' + ('; '.join(r['front_edge'].get('reason', '') for r in records) or 'no pallet')
        self.ratios = ', '.join(f'#{i}: {r["valid_pixels"]}/{r["candidate_pixels"]} ({r["valid_ratio"]:.1%})'
                                for i, r in enumerate(records)) or 'No masks'
        self.error = ''
        self.snapshot = (
            dict(rgb=image_metadata(rgb_msg), depth=image_metadata(depth_msg), delta_ms=dt,
                 color_camera_info=info_metadata(ci), depth_camera_info=info_metadata(di),
                 xyz_status=xyz_status, experimental=True, calibration_verified=False,
                 warning=EXPERIMENTAL, xyz_frame_id=OPTICAL_FRAME,
                 intrinsics_source='/camera/color/camera_info',
                 registration_check=registration, depth_intrinsics_used=False,
                 rgb_intrinsics_valid=intrinsics is not None,
                 distance_definition='median optical Z, not Euclidean range',
                 xyz_definition='P on candidate plane near M; not XYZ of M; +X right, +Y down, +Z forward',
                 front_edge_semantics_verified=False,
                 measurements=records, erode_px=self.args.erode_px,
                 min_valid_pixels=self.args.min_valid_pixels, min_valid_ratio=self.args.min_valid_ratio,
                 depth_range_mm=[self.args.min_depth_mm, self.args.max_depth_mm],
                 model=str(self.args.model), conf=self.args.conf, imgsz=640),
            dict(files, **{'rgb.png': rgb, 'depth_mm.png': depth,
                          'rgb_annotated.png': rgb_view, 'depth_annotated.png': depth_view}))

    def compose(self, fps, status):
        canvas = np.zeros((660, 1280, 3), np.uint8)
        canvas[32:512, :640] = self.rgb_view
        canvas[32:512, 640:] = self.depth_view
        put(canvas, 'RGB: AUTO EDGE / M=midpoint P=Depth sample | R:reset | O:old', 10, 23)
        put(canvas, 'Depth: GREEN=valid samples, YELLOW=eroded mask, MAGENTA=zero', 650, 23, scale=0.43)
        for y, line in enumerate((EXPERIMENTAL, f'FPS={fps:.1f} | {self.summary}',
                                  self.xyz_status, f'Valid pixels: {self.ratios}',
                                  self.error or status,
                                  'Zmed = optical depth, not range. HOLD retains last result. Q: quit | S: save')):
            put(canvas, line[:165], 10, 536+y*22, (0, 220, 255) if y == 0 else (255, 255, 255))
        return canvas

    def save(self, canvas):
        if self.snapshot is None:
            print('No accepted result to save')
            return
        metadata, files = self.snapshot
        directory = self.args.output_dir / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        try:
            directory.mkdir(parents=True, exist_ok=False)
            for name, data in {**files, 'inspection.png': canvas}.items():
                if not cv2.imwrite(str(directory/name), data):
                    raise OSError(f'Failed writing {name}')
            (directory/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
            print(f'Saved {directory.resolve()}')
        except (OSError, cv2.error) as exc:
            print(f'Save failed (folder may be incomplete): {exc}', file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parent/'models/best_v3.pt')
    parser.add_argument('--manual-edge', action='store_true', help='Optional manual A/B diagnostic mode; default is automatic')
    parser.add_argument('--show-old-point', action='store_true', help='Legacy option; press O to toggle the old comparison point')
    parser.add_argument('--conf', type=float, default=0.60,
                        help='YOLO confidence threshold (default: 0.60)')
    parser.add_argument('--device', default='0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--max-delta-ms', type=float, default=50)
    parser.add_argument('--max-age-ms', type=float, default=250, help='Maximum pending frame residence time')
    parser.add_argument('--display-fps', type=float, default=10)
    parser.add_argument('--erode-px', type=int, default=3)
    parser.add_argument('--min-valid-pixels', type=int, default=100)
    parser.add_argument('--min-valid-ratio', type=float, default=0.30)
    parser.add_argument('--min-depth-mm', type=int, default=1)
    parser.add_argument('--max-depth-mm', type=int, default=10000)
    parser.add_argument('--output-dir', type=Path, default=Path('yolo_depth_captures'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not np.isfinite(args.conf) or not 0 <= args.conf <= 1:
        parser.error('--conf must be finite and between 0 and 1')
    if not np.isfinite([args.max_delta_ms, args.max_age_ms, args.display_fps, args.min_valid_ratio]).all():
        parser.error('Numeric arguments must be finite')
    if not 0 <= args.max_delta_ms <= 50 or args.max_age_ms <= 0 or args.display_fps <= 0:
        parser.error('Require delta 0..50 ms, age > 0 and FPS > 0')
    if not 0 <= args.erode_px <= 50 or args.min_valid_pixels < 1 or not 0 < args.min_valid_ratio <= 1:
        parser.error('Require erosion 0..50, min pixels >= 1, valid ratio in (0,1]')
    if not 1 <= args.min_depth_mm < args.max_depth_mm <= 65535:
        parser.error('Invalid depth range in millimeters')
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}')
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No graphical display: enable WSLg/X server')
    print(f'Python={sys.executable} | OpenCV={cv2.__version__} | NumPy={np.__version__} | '
          f'Ultralytics={ultralytics.__version__} | device={args.device}')
    print(EXPERIMENTAL)
    model = YOLO(str(args.model))
    if model.task != 'segment':
        parser.error(f'Model task is {model.task}, expected segment')
    # Warm up before subscribing, so initial model setup cannot queue camera frames.
    model.predict(np.zeros((*SHAPE, 3), np.uint8), conf=args.conf, imgsz=640,
                  retina_masks=True, device=args.device, verbose=False)
    rclpy.init(args=sys.argv)
    node = CameraReceiver(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    receiver_thread = threading.Thread(target=executor.spin, daemon=True)
    receiver_thread.start()
    processor = Processor(args, model)
    fps, count, started = 0.0, 0, time.monotonic()
    next_display = 0.0
    canvas = processor.compose(fps, node.status)
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | getattr(cv2, 'WINDOW_GUI_NORMAL', 0))
        cv2.resizeWindow(WINDOW, 1280, 660)
        cv2.setMouseCallback(WINDOW, processor.mouse)
        while rclpy.ok():
            now = time.monotonic()
            if now >= next_display:
                pair = node.take_pair()
                if pair is not None:
                    try:
                        processor.process(pair)
                    except Exception as exc:
                        processor.error = f'HOLD / {type(exc).__name__}: {exc}'
                        print(processor.error, file=sys.stderr)
                elapsed = time.monotonic()-started
                if elapsed >= 1:
                    fps = count/elapsed
                    print(f'FPS={fps:.2f} | {processor.summary} | valid={processor.ratios} | {node.status}')
                    started, count = time.monotonic(), 0
                canvas = processor.compose(fps, node.status)
                cv2.imshow(WINDOW, canvas)
                count += 1
                next_display = time.monotonic() + 1/args.display_fps
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), ord('Q')):
                break
            if key in (ord('r'), ord('R')):
                processor.manual_ab = []
                processor.manual_bbox = None
                processor.front_edges = FrontEdgeTracker()
            if key in (ord('o'), ord('O')):
                processor.show_old_point = not processor.show_old_point
            if key in (ord('s'), ord('S')):
                processor.save(canvas)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        receiver_thread.join(timeout=2)
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
