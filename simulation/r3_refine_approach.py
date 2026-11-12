#!/usr/bin/env python3
"""Try a minimum-jerk A->C refinement of an existing r3 Smac plan.

This is a preview experiment. It preserves C, the exact terminal line, and
the original planner output. It refuses a refinement that fails collision,
forward-motion, curvature, or the existing follower's kinematic simulation.
No ROS or actuator publishers are used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from r3_core import (FrozenPlan, compose, dense, errors, fork_for_base, inverse,
                     overlap, polygons, rectangle, validate_phases, wrap)
from r3_sim import simulate, write_csv


def minimum_jerk_approach(start, end, target, geometry, radius_bound, spacing_m=.01,
                          curvature_cap=.8):
    """Clamped quintic with a bounded shape search for the fork-root trace."""
    origin = (*target[:2], target[2])
    local = inverse(origin)
    a, c = compose(local, start), compose(local, end)
    length = c[0] - a[0]
    if length <= 0 or max(abs(a[2]), abs(c[2])) >= math.pi / 2 - .1:
        raise ValueError('minimum-jerk refinement requires forward X and finite endpoint slopes')
    b1 = length * math.tan(a[2])
    displacement = c[1] - a[1] - b1
    slope_change = length * math.tan(c[2]) - b1
    b3 = 10 * displacement - 4 * slope_change
    b4 = 7 * slope_change - 15 * displacement
    b5 = 6 * displacement - 3 * slope_change
    # These two basis functions and their first two derivatives vanish at both
    # ends. They can reshape A->C without changing A, C, their yaw, or the
    # zero-curvature join. Search for the smallest overshoot of the *fork root*,
    # which moves laterally during chassis yaw because it is ahead of base_link.
    samples = [i / 100 for i in range(101)]
    fx, fy, _ = geometry['base_fork']
    start_root_y = a[1] + fx*math.sin(a[2]) + fy*math.cos(a[2])
    goal_root_y = c[1] + fx*math.sin(c[2]) + fy*math.cos(c[2])
    side = 1 if goal_root_y >= start_root_y else -1
    curvature_limit = min(1 / radius_bound, curvature_cap)

    def evaluate(shape_a, shape_b, make_poses=False):
        poses, root_y = [], []
        max_curvature = 0.
        for u in samples if not make_poses else [i/max(2, math.ceil(length/spacing_m))
                                                 for i in range(max(2, math.ceil(length/spacing_m))+1)]:
            phi = u**3 * (1-u)**3
            phi1 = 3*u*u*(1-u)**3 - 3*u**3*(1-u)**2
            phi2 = 6*u*(1-u)**3 - 18*u*u*(1-u)**2 + 6*u**3*(1-u)
            psi = phi*(2*u-1)
            psi1 = phi1*(2*u-1) + 2*phi
            psi2 = phi2*(2*u-1) + 4*phi1
            y = (a[1] + b1*u + b3*u**3 + b4*u**4 + b5*u**5
                 + shape_a*phi + shape_b*psi)
            slope = (b1 + 3*b3*u*u + 4*b4*u**3 + 5*b5*u**4
                     + shape_a*phi1 + shape_b*psi1) / length
            second = (6*b3*u + 12*b4*u*u + 20*b5*u**3
                      + shape_a*phi2 + shape_b*psi2) / length**2
            max_curvature = max(max_curvature, abs(second) / (1+slope*slope)**1.5)
            root_y.append(y + (fx*slope+fy)/math.sqrt(1+slope*slope))
            if make_poses:
                poses.append(compose(origin, (a[0] + length*u, y, math.atan(slope))))
        overshoot = max(0., max(side*(v-goal_root_y) for v in root_y))
        wrong_way = sum(max(0., -side*(v-u)) for u,v in zip(root_y,root_y[1:]))
        score = overshoot + .15*wrong_way + .004*max_curvature
        return score, max_curvature, overshoot, poses

    choices = []
    for i in range(-30, 31):
        for j in range(-30, 31):
            shape_a, shape_b = i*.05, j*.05
            score, curvature, overshoot, _ = evaluate(shape_a, shape_b)
            if curvature <= curvature_limit:
                choices.append((score, abs(shape_a)+abs(shape_b), shape_a, shape_b))
    if not choices:
        raise ValueError('no bounded smooth A-to-C refinement')
    _, _, shape_a, shape_b = min(choices)
    _, max_curvature, overshoot, poses = evaluate(shape_a, shape_b, make_poses=True)
    poses[0], poses[-1] = tuple(start), tuple(end)
    return poses, max_curvature, overshoot, (shape_a, shape_b)


def refine(source, out):
    if out.exists() and any(out.iterdir()):
        raise ValueError('choose a fresh output directory')
    frozen = json.loads((source / 'frozen_plan.json').read_text())
    snapshot = json.loads(frozen['target_json'])
    geometry = json.loads(frozen['geometry_json'])
    original = json.loads(frozen['phases_json'])
    if [phase['type'] for phase in original] != ['APPROACH', 'STRAIGHT']:
        raise ValueError('this refinement supports one forward approach and one terminal line')
    minimum_radius = json.loads(frozen['config_json'])['parameters']['Constrained.minimum_turning_radius']
    refined, max_curvature, fork_overshoot, shape = minimum_jerk_approach(
        original[0]['poses'][0], original[0]['poses'][-1], snapshot['pose'],
        geometry, minimum_radius)
    if max_curvature > 1 / minimum_radius:
        raise ValueError('refined approach exceeds the configured turning-radius bound')
    candidate = [dict(original[0], poses=refined), original[1]]
    validate_phases(candidate, snapshot['pose'], geometry)

    # Recheck the planner's obstacle rectangles using the full body and tines.
    c_local = compose(inverse(snapshot['pose']), refined[-1])
    requests = list(source.glob('request_*.json'))
    matching = [json.loads(path.read_text()) for path in requests
                if math.dist(json.loads(path.read_text())['goal'][:2], c_local[:2]) < 1e-6]
    if len(matching) != 1:
        raise ValueError('cannot identify the original Nav2 costmap request')
    obstacles = [rectangle((0., 0., 0.), (v[0], v[2], v[1], v[3]))
                 for v in matching[0]['obstacles']]
    for pose in dense(refined):
        local_pose = compose(inverse(snapshot['pose']), pose)
        body, tines, _ = polygons(local_pose, (0., 0., 0.), geometry)
        if any(overlap(part, obstacle) for part in [body, *tines] for obstacle in obstacles):
            raise ValueError('refined approach enters a Nav2 obstacle')

    rows, follower = simulate(candidate, snapshot['pose'], geometry, refined[0])
    if follower.state != 'STOPPED':
        raise ValueError('synthetic Pure Pursuit did not finish: ' + follower.reason)
    final = rows[-1]
    if (abs(final['ey_m']) > geometry['max_lateral'] or
            abs(math.degrees(final['yaw_error_rad'])) > geometry['max_yaw_deg']):
        raise ValueError('synthetic follower missed terminal lateral/yaw limits')
    config = json.loads(frozen['config_json'])
    config['approach_refinement'] = dict(method='bounded fork-root shape search on clamped quintic y(x)',
                                         source_plan_id=frozen['plan_id'],
                                         selected_C_unchanged=True,
                                         terminal_line_unchanged=True,
                                         max_curvature_1pm=max_curvature,
                                         fork_overshoot_m=fork_overshoot,
                                         shape_coefficients=shape)
    config['source_hashes']['simulation/r3_refine_approach.py'] = hashlib.sha256(
        Path(__file__).read_bytes()).hexdigest()
    plan = FrozenPlan.create(snapshot, geometry, candidate, config)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'frozen_plan.json').write_text(json.dumps({
        'target_json': plan.target_json, 'geometry_json': plan.geometry_json,
        'phases_json': plan.phases_json, 'config_json': plan.config_json,
        'plan_id': plan.plan_id}, indent=2))
    (out / 'case.json').write_bytes((source / 'case.json').read_bytes())
    (out / 'target_snapshot.json').write_text(json.dumps(snapshot, indent=2))
    write_csv(out / 'path_planned_base.csv', [
        dict(phase=phase['type'], x_m=pose[0], y_m=pose[1], yaw_rad=pose[2])
        for phase in candidate for pose in phase['poses']])
    write_csv(out / 'path_planned_fork_root.csv', [
        dict(phase=phase['type'], **dict(zip(('x_m', 'y_m', 'yaw_rad'),
             fork_for_base(pose, geometry))))
        for phase in candidate for pose in phase['poses']])
    write_csv(out / 'executed_synthetic_odom_fork_root.csv', [
        dict(time_s=row['time_s'], x_m=row['fork_x_m'], y_m=row['fork_y_m'],
             yaw_rad=row['fork_yaw_rad']) for row in rows])
    result = dict(status='PREVIEW_WAIT', source_plan_id=frozen['plan_id'], plan_id=plan.plan_id,
                  approach_method='Smac-selected C + fork-root-optimized A-to-C refinement',
                  max_curvature_1pm=max_curvature, turning_radius_bound_m=minimum_radius,
                  fork_overshoot_m=fork_overshoot, shape_coefficients=shape,
                  C_gap_m=errors(candidate[1]['poses'][0], snapshot['pose'], geometry)['gap_m'],
                  B_gap_m=errors(candidate[1]['poses'][-1], snapshot['pose'], geometry)['gap_m'],
                  synthetic_follower_state=follower.state,
                  synthetic_final_lateral_m=final['ey_m'],
                  synthetic_final_yaw_deg=math.degrees(final['yaw_error_rad']),
                  synthetic_max_omega_rps=max(abs(row['cmd_w_rps']) for row in rows),
                  real_robot_commands=0, real_geometry_confirmed=False)
    (out / 'prepare_result.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(refine(args.source, args.output), indent=2))


if __name__ == '__main__':
    main()
