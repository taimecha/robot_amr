"""Stationary RGB-D qualification after YOLO has completely stopped."""
from collections import deque
import math


class RGBDHealth:
    WINDOW_S = 5.0
    MIN_SAMPLES = 6

    def __init__(self):
        self.reset()

    def reset(self):
        self.started = None
        self.last_stamp = None
        self.rows = deque()

    def sample(self, stamp, wall, ros, valid, x, y, heading):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if self.started is None:
            self.started = wall
        valid = (valid and -.05 <= ros-stamp <= .25
                 and all(math.isfinite(v) for v in (x, y, heading)))
        self.rows.append((wall, stamp, bool(valid), x, y, heading))
        while self.rows and wall-self.rows[0][0] > self.WINDOW_S:
            self.rows.popleft()

    def snapshot(self, wall, ros):
        rows = [r for r in self.rows if wall-r[0] <= self.WINDOW_S]
        good = [r for r in rows if r[2]]
        span = rows[-1][0]-rows[0][0] if len(rows) > 1 else 0.
        fps = (len(good)-1)/span if span > 0 and good else 0.
        ratio = len(good)/len(rows) if rows else 0.
        gap = max([b[0]-a[0] for a,b in zip(good,good[1:])]
                  + ([wall-good[-1][0], good[0][0]-rows[0][0]] if good else [self.WINDOW_S]))
        spread = [max(r[i] for r in good)-min(r[i] for r in good)
                  if good else None for i in (3,4)]
        yaw_ref = good[-1][5] if good else 0.
        yaw_offsets = [math.atan2(math.sin(r[5]-yaw_ref),math.cos(r[5]-yaw_ref))
                       for r in good]
        spread.append(max(yaw_offsets)-min(yaw_offsets) if good else None)
        fresh = bool(rows and rows[-1][2] and wall-rows[-1][0] <= .25
                     and -.05 <= ros-rows[-1][1] <= .25)
        # Stationary acquisition favors quality over frame rate. Freshness still
        # uses the independent 250 ms deadline at the time of readiness.
        stable = (fresh and span >= self.WINDOW_S-.15 and len(good) >= self.MIN_SAMPLES
                  and ratio >= .98 and spread[0] <= .03 and spread[1] <= .02
                  and spread[2] <= math.radians(2))
        return dict(stable=bool(stable), window_s=self.WINDOW_S,
                    observed_s=span, samples=len(rows), valid_samples=len(good),
                    valid_ratio=ratio, valid_fps=fps, max_gap_s=gap,
                    forward_range_m=spread[0], lateral_range_m=spread[1],
                    heading_range_deg=math.degrees(spread[2]) if good else None,
                    fresh=fresh)
