"""Unrestricted finite scenario inputs, referenced to the center of fork roots.

Pallet frame: +X into pallet, +Y pallet left. Distance is along pallet normal;
lateral is fork-root center minus opening center. Positive yaw turns left.
"""
import argparse
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
from r3_core import base_for_fork, compose, inverse, digest, wrap

@dataclass(frozen=True)
class ForkCase:
    distance_m: float = 1.5
    lateral_m: float = 0.
    yaw_deg: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Chỉ nhập số hữu hạn; không có giới hạn khoảng cách, ngang hoặc góc.')

    def scenario(self, g):
        pallet_world=(1.8225,.10,0.)
        fork_world=compose(pallet_world,(-self.distance_m,self.lateral_m,wrap(math.radians(self.yaw_deg))))
        base_world=base_for_fork(fork_world,g)
        return dict(input=asdict(self),input_reference='fork_root relative to front_entry_corridor_center',
                    base_world=base_world,fork_world=fork_world,pallet_world=pallet_world,
                    start_odom=[0.,0.,0.],target_odom=compose(inverse(base_world),pallet_world),
                    case_id=digest(asdict(self)),provenance='ENTERED_SIMULATION_CASE_NOT_CAMERA_MEASUREMENT')


def load_case(path):
    return json.loads(Path(path).read_text())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distance',type=float);p.add_argument('--lateral',type=float);p.add_argument('--yaw-deg',type=float)
    p.add_argument('--case',type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--headless',action='store_true')
    a=p.parse_args()
    if a.case:
        inputs=load_case(a.case)['input'];case=ForkCase(**inputs)
    else:
        values=[]
        for key,label,default in [('distance','Gốc càng cách mặt trước pallet dọc pháp tuyến (m)',1.5),
                                  ('lateral','Gốc càng lệch tâm lỗ: + trái pallet, - phải (m)',0.),
                                  ('yaw_deg','Góc càng so pallet: 0 song song, + quay trái (độ)',0.)]:
            value=getattr(a,key)
            if value is None:
                while True:
                    raw=input(f'{label} [{default}]: ').strip()
                    try:
                        value=float(raw) if raw else default
                        if not math.isfinite(value):raise ValueError()
                        break
                    except ValueError:print('Cần một số hữu hạn; không có giới hạn min/max.')
            values.append(value)
        case=ForkCase(*values)
    g=json.loads((Path(__file__).parent/'r3/fixture.json').read_text())
    scenario=case.scenario(g);scenario['headless']=a.headless
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(scenario,indent=2))
    print('Đã lưu tư thế theo tâm gốc càng:',case)
if __name__=='__main__':main()
