#!/usr/bin/env python3
"""Offline historical quintic length comparison only; never a planner fallback."""
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from docking_preview import candidate
from r3_core import compose,inverse,validate_phases
out=Path(sys.argv[1])
for file in out.glob('*/frozen_plan.json'):
    p=json.loads(file.read_text());phases=json.loads(p['phases_json']);g=json.loads(p['geometry_json']);target=json.loads(p['target_json'])['pose']
    start=phases[0]['poses'][0];A=phases[-1]['poses'][0]
    local=inverse((start[0],A[1],0.));sl=compose(local,start);al=compose(local,A)
    proposal=candidate(sl[1],math.degrees(sl[2]),al[0])
    poses=[compose((start[0],A[1],0.),v[:3]) for v in proposal['rows']]
    baseline=[dict(type='APPROACH',poses=poses),phases[-1]]
    try:validate_phases(baseline,target,g);valid=True;reason='geometric validation only; no follower acceptance claimed'
    except ValueError as e:valid=False;reason=str(e)
    length=sum(math.dist(a[:2],b[:2]) for ph in baseline for a,b in zip(ph['poses'],ph['poses'][1:]))
    result=dict(mode='offline comparison only, not executable fallback',baseline='historical quintic+straight',
                same_start_and_A_and_goal=True,length_m=length,geometrically_valid=valid,reason=reason,
                lattice_length_m=sum(math.dist(a[:2],b[:2]) for ph in phases for a,b in zip(ph['poses'],ph['poses'][1:])),global_optimality_claim=False)
    (file.parent/'baseline_comparison.json').write_text(json.dumps(result,indent=2))
