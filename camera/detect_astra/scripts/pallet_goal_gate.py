"""One-shot observation of a recognition goal; never sends navigation commands."""


class RecognitionGoalGate:
    def __init__(self, now_ros):
        self.navigation_active = False
        self.arm(now_ros)

    def arm(self, now_ros):
        self.armed = True
        self.armed_at = now_ros
        self.goal_id = None
        self.goal_stamp = None
        self.phase = 'WAITING_GOAL'

    def update(self, entries):
        """Entries are (UUID hex, ROS goal creation seconds, action status).

        Require an accepted/executing goal created after arming. Historical
        terminal results, canceled/aborted goals and repeats cannot trigger YOLO.
        The caller pauses inference whenever navigation_active is true.
        """
        active = [e for e in entries if e[2] in (1, 2, 3)]
        self.navigation_active = bool(active)
        if not self.armed:
            if active:
                self.phase = 'WAITING_ARM'
            return False
        candidates = [e for e in active if e[1] >= self.armed_at and e[2] in (1, 2)]
        if candidates:
            newest = max(candidates, key=lambda e: (e[1], e[0]))
            if self.goal_stamp is None or (newest[1], newest[0]) > (self.goal_stamp, self.goal_id):
                self.goal_id, self.goal_stamp = newest[:2]
                self.phase = 'NAVIGATING'
        result = next((e[2] for e in entries if e[0] == self.goal_id), None)
        if result in (5, 6):
            self.armed = False
            self.phase = 'GOAL_FAILED'
        elif result == 4 and not active:
            self.armed = False
            self.phase = 'GOAL_SUCCEEDED'
            return True
        return False

    def snapshot(self):
        return dict(armed=self.armed, phase=self.phase, goal_id=self.goal_id,
                    navigation_active=self.navigation_active)
