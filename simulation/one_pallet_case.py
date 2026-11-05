"""Input pose and odom-frame path for the stationary one-pallet Gazebo demo."""
import argparse
from dataclasses import dataclass
import math
import os

from docking_preview import candidate


PALLET_X = 2.0
PALLET_Y = .10
PALLET_LENGTH = .355
FORK_TIP_FROM_BASE = .7555
FORK_LENGTH = .390
FORK_ROOT_FROM_BASE = FORK_TIP_FROM_BASE-FORK_LENGTH
PRE_DOCK_GAP = .16
FORK_ROOT_CLEARANCE = .012
DEFAULT_DISTANCE = PALLET_X-PALLET_LENGTH/2


@dataclass(frozen=True)
class OnePalletCase:
    # Distance is along the pallet approach axis, from base_link to its front face.
    distance_m: float = DEFAULT_DISTANCE
    lateral_m: float = -PALLET_Y  # positive: robot left of the pallet centerline
    yaw_deg: float = 0.          # positive: robot points left of the approach axis

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Khoảng cách, lệch ngang và góc phải là số hữu hạn')
        if not 1.45 <= self.distance_m <= 3.5:
            raise ValueError('Khoảng cách base_link → mặt trước pallet cần từ 1,45 đến 3,5 m')
        if abs(self.lateral_m) > .35 or abs(self.yaw_deg) > 20:
            raise ValueError('Chỉ hỗ trợ lệch ngang ±0,35 m và góc ±20° trong demo này')

    @property
    def yaw_rad(self):
        return math.radians(self.yaw_deg)

    @property
    def spawn_x(self):
        return PALLET_X-PALLET_LENGTH/2-self.distance_m

    @property
    def spawn_y(self):
        return PALLET_Y+self.lateral_m

    def target_in_odom(self):
        """Expected pallet center if wheel odom begins at the robot's spawn pose."""
        dx,dy=PALLET_X-self.spawn_x,PALLET_Y-self.spawn_y
        c,s=math.cos(self.yaw_rad),math.sin(self.yaw_rad)
        return c*dx+s*dy,-s*dx+c*dy

    def path_in_odom(self, target_x, target_y):
        """Curve from initial robot pose to a straight pre-dock line in odom."""
        # Pallet's world +X axis is -spawn_yaw in the initial odom frame.
        phi=-self.yaw_rad
        c,s=math.cos(phi),math.sin(phi)
        target_forward=c*target_x+s*target_y
        target_left=-s*target_x+c*target_y
        approach=target_forward-PALLET_LENGTH/2-FORK_TIP_FROM_BASE-PRE_DOCK_GAP
        if approach <= .5:
            raise ValueError('Pallet quá gần để lập đường cong pre-dock')
        proposal=candidate(-target_left,self.yaw_deg,approach)
        if not proposal['illustrative_feasible']:
            raise ValueError(f'Không có đường cong phù hợp: curvature={proposal["max_abs_kappa_1pm"]:.2f} 1/m')
        rows=[]
        for x,y,yaw,kappa,_ in proposal['rows']:
            x,y=x,y+target_left
            rows.append((c*x-s*y,s*x+c*y,yaw+phi,kappa))
        return rows

    def insertion_end_in_odom(self, target_x, target_y):
        """Base pose with fork roots 12 mm before the near face of the pallet."""
        phi=-self.yaw_rad
        reach=-PALLET_LENGTH/2-FORK_ROOT_CLEARANCE-FORK_ROOT_FROM_BASE
        return (target_x+reach*math.cos(phi),
                target_y+reach*math.sin(phi),phi)

    @classmethod
    def from_environment(cls):
        return cls(float(os.environ.get('TAI_ONE_PALLET_DISTANCE_M',DEFAULT_DISTANCE)),
                   float(os.environ.get('TAI_ONE_PALLET_LATERAL_M',-PALLET_Y)),
                   float(os.environ.get('TAI_ONE_PALLET_YAW_DEG',0.)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance',type=float,default=DEFAULT_DISTANCE,
                        help='m từ base_link của xe đến mặt trước pallet theo trục tiến; mặc định 1,8225')
    parser.add_argument('--lateral',type=float,default=-PALLET_Y,
                        help='m, xe nằm bên trái (+) hoặc bên phải (-) tâm pallet; mặc định -0,10')
    parser.add_argument('--yaw-deg',type=float,default=0.,
                        help='độ, xe quay về bên trái (+) hoặc bên phải (-); mặc định 0')
    parser.add_argument('--yolo-conf',type=float,default=.25,
                        help='ngưỡng confidence YOLO chỉ cho mô phỏng; mặc định 0,25')
    parser.add_argument('--interactive',action='store_true',help='nhập ba giá trị qua terminal')
    args=parser.parse_args()
    if args.interactive:
        import sys
        def ask(label,default):
            print(f'{label} [{default}]: ',end='',file=sys.stderr,flush=True)
            raw=input().strip()
            return float(raw) if raw else default
        try:
            args.distance=ask('Khoảng cách base_link → mặt pallet (m)',args.distance)
            args.lateral=ask('Xe lệch ngang: trái +, phải - (m)',args.lateral)
            args.yaw_deg=ask('Góc xe: quay trái +, phải - (độ)',args.yaw_deg)
        except (ValueError,EOFError) as exc:
            parser.error(f'Giá trị nhập không hợp lệ: {exc}')
    try:
        case=OnePalletCase(args.distance,args.lateral,args.yaw_deg)
        case.path_in_odom(*case.target_in_odom())
        if not math.isfinite(args.yolo_conf) or not .05 <= args.yolo_conf <= .9:
            raise ValueError('Ngưỡng YOLO cần từ 0,05 đến 0,9')
    except ValueError as exc:
        parser.error(str(exc))
    print(case.distance_m,case.lateral_m,case.yaw_deg,case.spawn_x,case.spawn_y,
          case.yaw_rad,args.yolo_conf)


if __name__=='__main__':
    main()
