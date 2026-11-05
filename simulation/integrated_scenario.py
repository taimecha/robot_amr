#!/usr/bin/env python3
"""Causal offline docking scenarios: quality -> lock -> plan -> shadow follow."""
import json
import math
from pathlib import Path

from docking_preview import PreviewSupervisor, candidate, shadow_replay
from plan_sim import evaluate
from quality_replay import StationaryGate, synthetic_frames


def scenario(calibration_correction_m):
    frames=synthetic_frames()
    for row in frames:
        row['x_m']-=calibration_correction_m
    gate=StationaryGate()
    accepted=[]
    for row in frames:
        if gate.add(row):accepted.append(row)
    pose=gate.pose()
    independent_x_error=abs(pose['x_m']-1.2)
    accuracy_pass=independent_x_error<=.005
    supervisor=PreviewSupervisor(accuracy_validated=accuracy_pass)
    for i in range(131):
        t=i*.1
        # Feed the actual accepted replay samples, offset into the run timeline.
        for row in accepted:
            if abs(t-(8.+row['stamp_s']))<.03:
                supervisor.sample(t,'pallet-A',row['x_m'],row['y_m'],row['yaw_rad'],True)
        supervisor.tick(t,detector_disabled=(supervisor.stop_requested_at is not None
                        and t-supervisor.stop_requested_at>=.4))
    if not accuracy_pass:
        return {'accuracy_pass':False,'absolute_x_error_m':independent_x_error,
                'state':supervisor.state,'motor_command_sent':False,
                'reason':'independent absolute accuracy budget failed'}
    if supervisor.state!='READY':
        return {'accuracy_pass':True,'absolute_x_error_m':independent_x_error,
                'state':supervisor.state,'motor_command_sent':False,
                'reason':'fresh verification evidence incomplete'}
    plan=evaluate(.02,2.,1.2)
    if plan['result']!='ILLUSTRATIVE_FEASIBLE':
        return {'accuracy_pass':True,'state':'REPOSITION_REQUIRED','motor_command_sent':False}
    supervisor.start_motion()
    shadow=shadow_replay([r[:4] for r in candidate(.02,2.,1.2)['rows']])
    return {'accuracy_pass':True,'absolute_x_error_m':independent_x_error,
            'state':shadow[-1]['state'],'plan_result':plan['result'],
            'shadow_duration_s':shadow[-1]['time_s'],'motor_command_sent':False,
            'insertion_verified':False,'lift_command_sent':False}


def main(output=Path('simulation/output')):
    output.mkdir(parents=True,exist_ok=True)
    results={'biased_uncorrected':scenario(0.),'synthetic_corrected':scenario(.01)}
    (output/'integrated_scenarios.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))
    return results


if __name__=='__main__':
    main()
