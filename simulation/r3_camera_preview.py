#!/usr/bin/env python3
"""Read-only r3 stationary-camera planning. No drive/lift publishers or execution.

Inputs are an exported stationary measurement and confirmed calibration/geometry.
Refuses synthetic calibration or missing values; never substitutes fixture.json.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
from r3_core import corrected_base_scalar_target, optical_target, digest
from r3_sim import build_backend, plan_once, plot, write_csv


def locked_snapshot(session, calibration):
    if session.get('provenance') != 'stationary_real_camera':
        raise ValueError('real stationary camera provenance required')
    if not session.get('stationary_verified'):
        raise ValueError('stationary odom/IMU evidence missing')
    if not calibration.get('confirmed_real'):
        raise ValueError('SPEC_INCOMPLETE: real calibration not confirmed')
    if calibration.get('yaw_accuracy_bound_deg') is None or calibration['yaw_accuracy_bound_deg'] > .5:
        raise ValueError('camera absolute yaw accuracy not demonstrated within0.5deg')
    if not calibration.get('accuracy_evidence'):
        raise ValueError('independent camera accuracy evidence required')
    samples = session['post_yolo_off_samples']
    if len(samples) < 5:
        raise ValueError('waiting for5 distinct post-YOLO RGB-D samples; no acquisition deadline')
    stamps = [v['capture_stamp_s'] for v in samples]
    if any(b <= a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('duplicate or nonmonotonic camera captures')
    if stamps[0] <= session['yolo_off_stamp_s']:
        raise ValueError('all locking samples must be after YOLO off acknowledgement')
    if not session.get('pallet_measurement_stopped_after_lock'):
        raise ValueError('pallet measurement/tracking stop acknowledgement missing')
    targets = []
    for sample in samples:
        metadata=dict(feature='front_entry_corridor_center',applied_bias=calibration['applied_bias'],
                      calibration_hash=digest(calibration),capture_stamp_s=sample['capture_stamp_s'])
        if session['adapter'] == 'corrected_base_scalars':
            metadata['source_frame']='base_link_corrected_scalars'
            if not calibration.get('corrected_scalar_adapter_confirmed'):
                raise ValueError('scalar reference has not been mapped to base_link')
            target=corrected_base_scalar_target(odom_at_capture=sample['odom_at_capture'],
                forward_base_m=sample['forward_base_m'],lateral_base_m=sample['lateral_base_m'],
                entry_heading_rad=sample['entry_heading_rad'],metadata=metadata)
        elif session['adapter'] == 'optical_SE3':
            target=optical_target(T_odom_base=sample['T_odom_base_at_capture'],
                T_base_camera_optical=calibration['T_base_camera_optical'],
                T_camera_pallet=sample['T_camera_pallet'],metadata=metadata)
        else:
            raise ValueError('unknown frame adapter')
        targets.append(target)
    # Stability is distinct from the independently measured absolute accuracy.
    poses=[s['pose'] for s in targets]
    from r3_core import wrap
    yaw0=poses[0][2]
    yaw=[wrap(p[2]-yaw0) for p in poses]
    if max(yaw)-min(yaw) > math.radians(.5):
        raise ValueError('waiting for stable RGB-D yaw')
    if any(max(p[k] for p in poses)-min(p[k] for p in poses) > .01 for k in (0,1)):
        raise ValueError('waiting for stable RGB-D position')
    result=dict(targets[-1],pose=[statistics.median(p[0] for p in poses),statistics.median(p[1] for p in poses),wrap(yaw0+statistics.median(yaw))],
                source='stationary_real_RGBD',raw_session_hash=digest(session),yolo_off_ack=True)
    if session['adapter'] == 'corrected_base_scalars':
        offset = calibration['applied_bias']['lateral_bias_m']
        if not math.isclose(offset, .025, abs_tol=1e-9):
            raise ValueError('camera-to-fork lateral offset must be the single accepted +2.5 cm correction')
        raw = [sample['camera_lateral_raw_m'] for sample in samples]
        corrected = [sample['lateral_base_m'] for sample in samples]
        if any(abs(a + offset - b) > 1e-4 for a, b in zip(raw, corrected)):
            raise ValueError('lateral correction was added twice or did not match camera raw samples')
        result['measurement_report'] = dict(
            camera_lateral_raw_m=statistics.median(raw),
            camera_mount_offset_m=offset,
            fork_lateral_corrected_m=statistics.median(corrected),
            forward_base_m=statistics.median(sample['forward_base_m'] for sample in samples),
            entry_heading_deg=math.degrees(result['pose'][2] - session['start_odom'][2]))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurement-json',required=True,type=Path)
    parser.add_argument('--calibration',required=True,type=Path)
    parser.add_argument('--geometry',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    try:
        session=json.loads(args.measurement_json.read_text());cal=json.loads(args.calibration.read_text());g=json.loads(args.geometry.read_text())
        if not g.get('confirmed_real_geometry') or g.get('missing_real'):
            raise ValueError('SPEC_INCOMPLETE: geometry/transforms are not all confirmed')
        if g.get('stop_gap_tolerance') != .005:
            raise ValueError('approved stop gap tolerance is5mm')
        snapshot=locked_snapshot(session,cal)
        plan,phases,length,rotations=plan_once(session['start_odom'],snapshot,g,build_backend(),args.output)
        (args.output/'target_snapshot.json').write_text(json.dumps(snapshot,indent=2))
        rows=[dict(phase=p['type'],x_m=v[0],y_m=v[1],yaw_rad=v[2]) for p in phases for v in p['poses']]
        write_csv(args.output/'path_planned_base.csv',rows)
        from r3_core import fork_for_base
        write_csv(args.output/'path_planned_fork_root.csv',[dict(phase=p['type'],**dict(zip(('x_m','y_m','yaw_rad'),fork_for_base(v,g)))) for p in phases for v in p['poses']])
        # Preview has no odometry run. Never fabricate an executed overlay.
        plot(args.output,phases,[],snapshot['pose'],g,{},preview_only=True)
        result=dict(status='PREVIEW_WAIT',plan_id=plan.plan_id,length_m=length,rotation_phases=rotations,
                    robot_commands=0,requires_operator_start=True)
    except (ValueError,KeyError,OSError) as exc:
        result=dict(status='INCOMPLETE',reason=str(exc),robot_commands=0)
    (args.output/'mission_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
