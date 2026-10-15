"""Convert the saved left-arm calibration to the policy's 8-value state layout.

The left arm contributes seven joint angles in radians and one gripper width.
The conversion preserves the existing rig's asymmetric joint limits.
"""
import numpy as np
from .settings import read_json

ROLES = ("left_follower",)
STATE_DIM = 8
JOINT_INDICES = np.arange(7)
GRIPPER_INDICES = np.array([7])


class Calibration:
    def __init__(self, cfg):
        self.robot = read_json(cfg["robot_file"])
        # The archived calibration contains other arms. Only load the active
        # left follower; the right arm is never needed or opened.
        buses = read_json(cfg["calibration_file"])["buses"]
        self.saved = {role: buses[role] for role in ROLES}
        rig_map = read_json(cfg["rig_map_file"])
        for key in ("joint_lower_rad", "joint_upper_rad", "home_joints_rad"):
            if len(self.robot[key]) != 7 or not np.all(np.isfinite(self.robot[key])):
                raise ValueError(f"{key} must contain seven finite values")
        lower = self.robot["joint_lower_rad"] + [0.0]
        upper = self.robot["joint_upper_rad"] + [1.0]
        self.lower = np.array(lower, dtype=float)
        self.upper = np.array(upper, dtype=float)
        if not np.all(np.isfinite([self.lower, self.upper])) or np.any(self.upper <= self.lower):
            raise ValueError("Invalid robot limits")
        home = np.array(self.robot["home_joints_rad"])
        if np.any(home < self.lower[:7]) or np.any(home > self.upper[:7]):
            raise ValueError("Mock home pose exceeds joint limits")
        self.minimum, self.maximum = {}, {}
        for role in ROLES:
            bus = self.saved[role]
            if bus["serial"] != rig_map[role]["serial"]:
                raise ValueError(f"Calibration / rig-map serial mismatch for {role}")
            motors = [bus["motors"][str(i)] for i in range(1, 9)]
            if not all(m["range_freezed"] for m in motors):
                raise ValueError(f"Unfrozen calibration for {role}")
            lo = np.array([m["range_min"] for m in motors])
            hi = np.array([m["range_max"] for m in motors])
            if np.any(lo < 0) or np.any(hi > 4095) or np.any(hi <= lo):
                raise ValueError(f"Invalid encoder ranges for {role}")
            self.minimum[role], self.maximum[role] = lo, hi

    def decode(self, counts):
        """Measured left motor counts -> [seven left joints, left gripper]."""
        arms = []
        for role in ROLES:
            raw = np.array([counts[role][m] for m in range(1, 9)], dtype=float)
            fraction = (raw - self.minimum[role]) / (self.maximum[role] - self.minimum[role])
            # Leave joints unclipped: a bad measurement must remain visible.
            values = self.lower[:8] + fraction * (self.upper[:8] - self.lower[:8])
            values[7] = np.clip(values[7], 0, 1)
            arms.append(values)
        return np.concatenate(arms)

    def encode(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (STATE_DIM,) or not np.all(np.isfinite(state)):
            raise ValueError("Motor target must contain 8 finite values for the left arm")
        if np.any(state < self.lower) or np.any(state > self.upper):
            raise ValueError("Motor target exceeds joint/gripper limits")
        goals = {}
        for arm, role in enumerate(ROLES):
            part = state[arm * 8:(arm + 1) * 8]
            fraction = (part - self.lower[:8]) / (self.upper[:8] - self.lower[:8])
            counts = np.rint(self.minimum[role] + fraction * (self.maximum[role] - self.minimum[role]))
            goals[role] = {i + 1: int(v) for i, v in enumerate(counts)}
        return goals
