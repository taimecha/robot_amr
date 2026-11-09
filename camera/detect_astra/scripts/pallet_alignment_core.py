"""Planar visual alignment; independent of ROS, hardware and neural networks."""
from collections import deque
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, bound):
    return max(-bound, min(bound, value))


def stationary_heading_command(current_yaw, goal_yaw, max_angular_rps):
    """Existing stationary alignment law, shared with fixed-path phase adapter."""
    return clamp(1.2*wrap(goal_yaw-current_yaw), max_angular_rps)


@dataclass(frozen=True)
class AlignmentConfig:
    standoff_m: float = 1.20
    chassis_front_m: float = .3005
    fork_root_m: float = .3655
    distance_tolerance_m: float = .02
    lateral_tolerance_m: float = .02
    heading_tolerance_deg: float = 2.0
    max_linear_mps: float = .10
    max_angular_rps: float = .20
    linear_accel_mps2: float = .08
    angular_accel_rps2: float = .40
    target_timeout_s: float = .25
    odom_timeout_s: float = .20
    settle_s: float = .5
    confirm_samples: int = 3
    max_face_yaw_deg: float = 18.0
    max_bearing_deg: float = 25.0
    max_duration_s: float = 120.0
    allow_reverse: bool = False
    max_reverse_m: float = .40

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'allow_reverse':
                if type(value) is not bool:
                    raise ValueError('allow_reverse must be boolean')
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (self.fork_root_m < self.chassis_front_m or type(self.confirm_samples) is not int
                or self.confirm_samples < 3):
            raise ValueError('Invalid fork origin or insufficient confirmation count')
        if self.max_linear_mps > .10 or self.max_angular_rps > .30:
            raise ValueError('First-trial speed limits exceed 0.10 m/s or 0.30 rad/s')
        if self.max_face_yaw_deg >= 20 or self.heading_tolerance_deg > 2:
            raise ValueError('Heading limits exceed measurement/alignment range')
        if self.lateral_tolerance_m > .02 or self.distance_tolerance_m > .02:
            raise ValueError('Position tolerance exceeds the accepted 2 cm limit')


@dataclass(frozen=True)
class OdomSample:
    stamp_s: float
    x: float
    y: float
    yaw: float
    linear: float = 0.0
    angular: float = 0.0


class OdomHistory:
    def __init__(self):
        self.samples = deque(maxlen=150)

    def add(self, sample):
        if not all(math.isfinite(v) for v in vars(sample).values()):
            raise ValueError('Nonfinite odometry')
        if self.samples and sample.stamp_s <= self.samples[-1].stamp_s:
            if sample.stamp_s < self.samples[-1].stamp_s:
                self.samples.clear()
            else:
                return
        self.samples.append(sample)

    def at(self, stamp_s):
        for a, b in zip(self.samples, list(self.samples)[1:]):
            if a.stamp_s <= stamp_s <= b.stamp_s and b.stamp_s-a.stamp_s <= .15:
                u = (stamp_s-a.stamp_s)/(b.stamp_s-a.stamp_s)
                return OdomSample(stamp_s, a.x+u*(b.x-a.x), a.y+u*(b.y-a.y),
                                  wrap(a.yaw+u*wrap(b.yaw-a.yaw)))
        if self.samples and abs(stamp_s-self.samples[-1].stamp_s) < 1e-6:
            return self.samples[-1]
        return None


class AlignmentController:
    def __init__(self, config=AlignmentConfig()):
        self.config = config
        self.odom = OdomHistory()
        self.target = None
        self.target_stamp = None
        self.target_received = None
        self.last_seen_stamp = None
        self.confirmed = 0
        self.active = False
        self.state = 'IDLE'
        self.reason = 'Chưa bắt đầu căn chỉnh'
        self.v = self.w = 0.0
        self.settle_started = None
        self.plan_yaw = None
        self.goal = None
        self.started_at = None
        self.last_tick = None
        self.backoff_origin = None
        self.reverse_used = 0.0
        self.data_hold_since = None

    def invalidate_target(self, reason):
        self.target = None
        self.target_stamp = self.target_received = None
        self.confirmed = 0
        self.settle_started = None
        self.reason = reason
        self.v = self.w = 0.0
        self.goal = None

    def observe(self, stamp_s, received_s, forward_base_m, left_m, entry_heading_rad):
        """Use odometry AT capture time; filter in odom to avoid turn-induced lag."""
        if not all(math.isfinite(v) for v in
                   (stamp_s, received_s, forward_base_m, left_m, entry_heading_rad)):
            self.invalidate_target('Phép đo không hữu hạn')
            return False
        if self.last_seen_stamp is not None and stamp_s <= self.last_seen_stamp:
            return False
        pose = self.odom.at(stamp_s)
        if pose is None:
            return False  # Caller may retry when the next odometry sample arrives.
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        raw = (pose.x+c*forward_base_m-s*left_m,
               pose.y+s*forward_base_m+c*left_m,
               wrap(pose.yaw+entry_heading_rad))
        dt = stamp_s-self.target_stamp if self.target_stamp is not None else 0
        if self.target is not None and 0 < dt <= self.config.target_timeout_s:
            if math.hypot(raw[0]-self.target[0], raw[1]-self.target[1]) > .10:
                self.invalidate_target('Vị trí pallet thay đổi đột ngột; cần xác nhận lại')
            elif abs(wrap(raw[2]-self.target[2])) > math.radians(6):
                self.invalidate_target('Hướng pallet thay đổi đột ngột; cần xác nhận lại')
        if self.target is None or not 0 < dt <= self.config.target_timeout_s:
            self.target = raw
            self.confirmed = 1
        else:
            a = -math.expm1(-dt/.15)
            dtheta = wrap(raw[2]-self.target[2])
            ah = -math.expm1(-dt/(.15 if abs(dtheta) >= math.radians(2) else .8))
            self.target = (self.target[0]+a*(raw[0]-self.target[0]),
                           self.target[1]+a*(raw[1]-self.target[1]),
                           wrap(self.target[2]+ah*dtheta))
            self.confirmed += 1
        self.target_stamp = self.last_seen_stamp = stamp_s
        self.target_received = received_s
        return True

    def readiness(self, now_s, wall_s):
        c = self.config
        if not self.odom.samples or not -.05 <= now_s-self.odom.samples[-1].stamp_s <= c.odom_timeout_s:
            return False, 'Chưa có odometry mới và đúng hệ tọa độ'
        if (self.target is None or not -.05 <= now_s-self.target_stamp <= c.target_timeout_s
                or wall_s-self.target_received > c.target_timeout_s):
            return False, 'Chưa có phép đo RGB + Depth mới; xe phải dừng'
        if self.confirmed < c.confirm_samples:
            return False, 'Đang xác nhận các phép đo mới'
        return True, ''

    def start(self, now_s, wall_s):
        ready, reason = self.readiness(now_s, wall_s)
        if not ready:
            return False, reason
        self.active = True
        self.started_at = wall_s
        self.plan_yaw = None
        self.settle_started = None
        self.backoff_origin = None
        self.reverse_used = 0.0
        self.state, self.reason = 'ALIGNING', 'Đang căn vị trí và hướng'
        self.data_hold_since = None
        return True, self.reason

    def stop(self, reason='Đã dừng căn chỉnh', state='IDLE'):
        self.active = False
        self.v = self.w = 0.0
        self.settle_started = None
        self.state, self.reason = state, reason

    def errors(self, pose):
        dx, dy = self.target[0]-pose.x, self.target[1]-pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        x, y = c*dx+s*dy, -s*dx+c*dy
        return x-self.config.fork_root_m-self.config.standoff_m, y, wrap(self.target[2]-pose.yaw), x

    def tick(self, now_s, wall_s, interlock=True, interlock_reason=''):
        c = self.config
        dt = min(.1, max(.001, wall_s-self.last_tick)) if self.last_tick is not None else .05
        self.last_tick = wall_s
        ready, reason = self.readiness(now_s, wall_s)
        if not interlock:
            self.stop(interlock_reason or 'Điều kiện chạy xe chưa đạt', 'BLOCKED')
        if not ready:
            self.v = self.w = 0.0
            self.settle_started = None
            self.goal = None
            if self.active:
                self.state, self.reason = 'WAIT_DATA', reason
                if self.data_hold_since is None:
                    self.data_hold_since = wall_s
                elif wall_s-self.data_hold_since > .75:
                    self.stop('Mất dữ liệu quá 0,75 s; cần bấm bắt đầu lại', 'LOST_DATA')
                if wall_s-self.started_at > c.max_duration_s:
                    self.stop('Hết thời gian căn chỉnh', 'TIMEOUT')
            return self.snapshot()
        self.data_hold_since = None
        pose = self.odom.samples[-1]
        ex, ey, heading, forward = self.errors(pose)
        # Keep a heading already within the user's 2-degree tolerance, so a
        # straight, centred approach does not invent an unnecessary yaw move.
        if self.plan_yaw is None or abs(wrap(self.target[2]-self.plan_yaw)) >= math.radians(c.heading_tolerance_deg):
            self.plan_yaw = pose.yaw if abs(heading) < math.radians(c.heading_tolerance_deg) else self.target[2]
        d = c.fork_root_m+c.standoff_m
        self.goal = (self.target[0]-d*math.cos(self.plan_yaw),
                     self.target[1]-d*math.sin(self.plan_yaw), self.plan_yaw)
        if not self.active:
            self.v = self.w = 0.0
            return self.snapshot()
        if wall_s-self.started_at > c.max_duration_s:
            self.stop('Hết thời gian căn chỉnh', 'TIMEOUT')
            return self.snapshot()
        if (abs(heading) > math.radians(c.max_face_yaw_deg)
                or abs(math.atan2(ey, forward)) > math.radians(c.max_bearing_deg)):
            self.stop('Pallet ở ngoài vùng căn đã giới hạn; cần đặt lại xe', 'REPOSITION_REQUIRED')
            return self.snapshot()
        aligned = (abs(ex) <= c.distance_tolerance_m and abs(ey) < c.lateral_tolerance_m
                   and abs(heading) < math.radians(c.heading_tolerance_deg))
        if aligned:
            self.v = self.w = 0.0
            stopped = abs(pose.linear) <= .015 and abs(pose.angular) <= .035
            if stopped:
                if self.settle_started is None:
                    self.settle_started = wall_s
                if wall_s-self.settle_started >= c.settle_s:
                    self.stop('Đã căn đúng tâm và hướng tại khoảng cách chờ', 'ALIGNED')
                    return self.snapshot()
            else:
                self.settle_started = None
            self.state, self.reason = 'SETTLING', 'Đang dừng và kiểm tra vị trí ổn định'
            return self.snapshot()
        self.settle_started = None
        dx, dy = self.goal[0]-pose.x, self.goal[1]-pose.y
        gx = math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
        gy = -math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
        rho = math.hypot(gx, gy)
        if self.backoff_origin is not None:
            bx, by, byaw = self.backoff_origin
            travelled = -(pose.x-bx)*math.cos(byaw)-(pose.y-by)*math.sin(byaw)
            if travelled < .22:
                v, w = -min(.04, c.max_linear_mps), clamp(1.2*wrap(byaw-pose.yaw), c.max_angular_rps)
                self.state, self.reason = 'BACKING', 'Lùi có giới hạn để có khoảng căn ngang'
                return self._command(v, w, dt)
            self.reverse_used += travelled
            self.backoff_origin = None
        if gx < .10 and abs(gy) > c.lateral_tolerance_m:
            if not c.allow_reverse or self.reverse_used+.22 > c.max_reverse_m:
                self.stop('Cần khoảng lùi để căn ngang; chưa cho phép lùi tự động', 'REPOSITION_REQUIRED')
                return self.snapshot()
            self.backoff_origin = (pose.x, pose.y, pose.yaw)
            self.v = self.w = 0.0
            return self.snapshot()
        if rho < .02:
            v = 0.0
            w = stationary_heading_command(pose.yaw, self.plan_yaw, c.max_angular_rps)
        else:
            direction = -1 if gx < -.02 else 1
            if direction < 0 and not c.allow_reverse:
                self.stop('Đã qua vị trí chờ; cần lùi nhưng chưa được cho phép', 'REPOSITION_REQUIRED')
                return self.snapshot()
            alpha = wrap(math.atan2(gy, gx)-(math.pi if direction < 0 else 0))
            beta = wrap(self.plan_yaw-pose.yaw-alpha)
            v = direction*min(c.max_linear_mps, .5*rho)*max(0., math.cos(alpha))
            w = 1.8*alpha-.65*beta
        # Predict the relative face angle after a short turn; do not steer the
        # pallet outside the vision algorithm's measured yaw range.
        margin = math.radians(c.max_face_yaw_deg)
        w = max((heading-margin)/.5, min((heading+margin)/.5, w))
        self.state, self.reason = 'ALIGNING', 'Đang căn tâm càng, khoảng cách và hướng'
        return self._command(v, clamp(w, c.max_angular_rps), dt)

    def _command(self, v, w, dt):
        if v*self.v < 0 and self.odom.samples and abs(self.odom.samples[-1].linear) > .015:
            self.v = self.w = 0.0
        else:
            self.v += clamp(v-self.v, self.config.linear_accel_mps2*dt)
            self.w += clamp(w-self.w, self.config.angular_accel_rps2*dt)
        return self.snapshot()

    def snapshot(self):
        errors = None
        if self.target is not None and self.odom.samples:
            ex, ey, heading, _ = self.errors(self.odom.samples[-1])
            errors = dict(distance_m=ex, lateral_m=ey, heading_deg=math.degrees(heading))
        return dict(state=self.state, reason=self.reason, active=self.active,
                    linear_mps=self.v, angular_rps=self.w, errors=errors,
                    goal=self.goal, standoff_m=self.config.standoff_m,
                    confirmed=self.confirmed)
