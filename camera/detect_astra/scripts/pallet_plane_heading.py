"""Horizontal pallet-plane angle using a level-floor depth-skew reference.

Opt-in for a camera with verified zero roll and a visible flat floor ROI.
This module returns an angle only; it never changes position measurements.
"""
import math
import cv2
import numpy as np


def _robust_fit(matrix, values, base_weights=None):
    base = np.ones(len(values)) if base_weights is None else base_weights
    weights = base.copy()
    if np.linalg.matrix_rank(matrix) != 3:
        raise ValueError('Heading plane is geometrically degenerate')
    for _ in range(5):
        root = np.sqrt(weights)
        coeff = np.linalg.lstsq(matrix*root[:,None],values*root,rcond=None)[0]
        residual = values-matrix@coeff
        scale = max(.0015,1.4826*float(np.median(abs(residual-np.median(residual)))))
        weights = base*np.minimum(1.,1.5*scale/np.maximum(abs(residual),1e-8))
    if not np.isfinite(coeff).all():
        raise ValueError('Nonfinite heading plane')
    return coeff,residual


def _rays(u,v,intrinsics):
    k,distortion = intrinsics
    return cv2.undistortPoints(np.column_stack((u,v)).astype(float).reshape(-1,1,2),
                               k,distortion).reshape(-1,2)


def floor_depth_skew(depth,intrinsics):
    if depth.shape != (480,640):
        raise ValueError('Heading floor reference requires calibrated 640x480 depth')
    v,u = np.mgrid[350:470:6,30:610:6]
    z = depth[v,u].astype(float)*.001
    valid = (z>.65)&(z<3.5)
    if valid.sum()<500 or valid.mean()<.65:
        raise ValueError('Heading unavailable: insufficient visible floor depth')
    u,v,z = u[valid],v[valid],z[valid]
    rays = _rays(u,v,intrinsics)
    fx = float(intrinsics[0][0,0])
    matrix = np.column_stack(((u-320.)/fx,rays[:,1],np.ones(len(z))))
    coeff,residual = _robust_fit(matrix,1/z)
    slope = float(coeff[0]/fx)
    mad = float(np.median(abs(residual-np.median(residual))))
    inliers = abs(residual)<=max(.006,3*mad)
    if (mad>.006 or inliers.mean()<.8 or coeff[1]<.2
            or abs(slope)>.0005 or np.ptp(u[inliers])<400 or np.ptp(v[inliers])<80):
        raise ValueError('Heading unavailable: floor reference is not one reliable plane')
    return slope,dict(heading_floor_samples=int(len(z)),heading_floor_mad_inv_m=mad)


def pallet_plane_heading(depth,intrinsics,components,pitch_rad,reference_u=370., *,
                         depth_x_slope=None, floor_diagnostics=None):
    # The independent horizontal floor removes per-frame depth X skew. Zero
    # camera roll is a prerequisite, not inferred from the pallet's angle.
    if depth_x_slope is None:
        slope,diagnostics = floor_depth_skew(depth,intrinsics)
    else:
        slope = float(depth_x_slope)
        if not math.isfinite(slope) or abs(slope) > .0005:
            raise ValueError('Heading unavailable: invalid shared floor depth correction')
        diagnostics = dict(floor_diagnostics or {})
    top = max(c['top'] for c in components)+2
    bottom = min(c['bottom'] for c in components)-2
    points=[];labels=[]
    for index,c in enumerate(components):
        u,v = c['u'],c['v']
        central = ((abs(u-np.median(u))<=max(3,int(c['width']*.23)))&(v>=top)&(v<=bottom))
        u,v = u[central],v[central]
        raw = depth[v,u].astype(float)*.001
        inverse = np.full_like(raw,np.nan)
        valid = raw>0
        inverse[valid] = 1/raw[valid]+slope*(reference_u-u[valid])
        valid &= np.isfinite(inverse)&(inverse>1/3.5)&(inverse<1/.65)
        if valid.sum()<60 or valid.mean()<.65:
            raise ValueError('Heading unavailable: insufficient front-plane samples on a leg')
        rays = _rays(u[valid],v[valid],intrinsics)
        points.append(np.column_stack((rays,inverse[valid])))
        labels.extend([index]*int(valid.sum()))
    points = np.concatenate(points);labels=np.asarray(labels)
    base = np.where(labels==0,1./len(points[labels==0]),1./len(points[labels==1]))
    matrix = np.column_stack((points[:,:2],np.ones(len(points))))
    normal,residual = _robust_fit(matrix,points[:,2],base)
    if float(np.median(abs(residual)))>.006:
        raise ValueError('Heading unavailable: pallet front is not one reliable plane')
    forward_normal = normal[2]*math.cos(pitch_rad)-normal[1]*math.sin(pitch_rad)
    if forward_normal<=0:
        raise ValueError('Heading unavailable: pallet plane faces away from camera')
    face_yaw = math.atan2(normal[0],forward_normal)
    if abs(face_yaw)>math.radians(20):
        raise ValueError('Heading unavailable: pallet-plane yaw outside measurement range')
    diagnostics.update(heading_depth_x_slope=slope,
                       heading_plane_samples=int(len(points)),heading_source='floor_referenced_plane',
                       heading_requires_zero_camera_roll=True,
                       heading_camera_mount_yaw_verified=False)
    return face_yaw,diagnostics
