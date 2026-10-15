"""Pure checks: validate entire chunks, then check each command against feedback."""
import logging
import numpy as np
from .calibration import JOINT_INDICES, GRIPPER_INDICES, STATE_DIM


class SafetyStop(RuntimeError):
    pass


class TrackingError(SafetyStop):
    """A joint failed to follow: do not start another automatic motion on exit."""


class Safety:
    def __init__(self, cfg, calibration):
        self.cfg, self.calibration = cfg, calibration
        self.bad_tracking_ticks = 0

    def chunk(self, reply, reference):
        actions = np.array(reply[self.cfg["action_key"]], dtype=float, copy=True)
        if actions.ndim != 2 or actions.shape[1] != STATE_DIM or not 1 <= len(actions) <= self.cfg["max_chunk_horizon"]:
            raise SafetyStop(f"Expected left-arm actions (H,8), got {actions.shape}")
        if not np.all(np.isfinite(actions)):
            raise SafetyStop("Non-finite action in chunk")
        lo, hi = self.calibration.lower, self.calibration.upper
        # Saturate finite gripper overshoots before checking joint tolerance
        # or per-tick changes. The server's input array stays untouched.
        if np.any(actions[:, GRIPPER_INDICES] < 0) or np.any(actions[:, GRIPPER_INDICES] > 1):
            logging.warning("Clipping gripper targets to [0,1] in action chunk")
        actions[:, GRIPPER_INDICES] = np.clip(actions[:, GRIPPER_INDICES], 0.0, 1.0)
        overshoot = np.maximum(lo - actions, actions - hi)
        if np.any(overshoot > self.cfg["clip_tolerance_rad"]):
            raise SafetyStop("Action exceeds joint limits")
        if np.any(overshoot > 0):
            logging.warning("Clipping small joint-limit overshoot in action chunk")
        actions = np.clip(actions, lo, hi)
        # Include entry and all trailing steps, even if chunk_steps is shorter.
        previous = np.vstack([reference, actions[:-1]])
        delta = np.abs(actions - previous)
        if np.any(delta[:, JOINT_INDICES] > self.cfg["max_joint_step_rad"] + 1e-9):
            raise SafetyStop("Joint jump too large; first target must be near the held pose")
        if np.any(delta[:, GRIPPER_INDICES] > self.cfg["max_gripper_step"] + 1e-9):
            raise SafetyStop("Gripper jump too large")
        return actions[:self.cfg["chunk_steps"]]

    def tracking(self, goal, measured):
        measured = np.asarray(measured)
        if measured.shape != (STATE_DIM,) or not np.all(np.isfinite(measured)):
            raise SafetyStop("Invalid motor feedback")
        error = np.max(np.abs(goal[JOINT_INDICES] - measured[JOINT_INDICES]))
        self.bad_tracking_ticks = self.bad_tracking_ticks + 1 if error > self.cfg["max_tracking_error_rad"] else 0
        if self.bad_tracking_ticks >= self.cfg["tracking_failure_ticks"]:
            raise TrackingError(f"Tracking error {error:.3f} rad exceeded limit")

    def before_write(self, target, measured):
        # Prevent issuing another far-away goal even during the multi-tick
        # tracking debounce. Reject instead of accumulating unreachable goals.
        error = np.max(np.abs(target[JOINT_INDICES] - measured[JOINT_INDICES]))
        if error > self.cfg["max_tracking_error_rad"] + self.cfg["max_joint_step_rad"]:
            raise SafetyStop("Next target is too far from the measured joint pose")
