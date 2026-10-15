"""Gradual left-arm homing, independent of cameras and policy inference.

This interpolates in joint space. It does not plan around obstacles. The
gripper keeps its width from arming, and exit keeps the final servo goals.
"""
import logging
import math
import time
import numpy as np
from .safety import Safety, SafetyStop

logger = logging.getLogger(__name__)


class HomeMotion:
    """Generate small synchronized joint steps and check measured progress."""
    def __init__(self, cfg, calibration, start):
        self.cfg = dict(cfg)
        speed = cfg.get("home_speed_rad_s", 0.2)
        self.tolerance = cfg.get("home_tolerance_rad", 0.02)
        tracking = cfg.get("home_max_tracking_error_rad", 0.15)
        self.timeout = cfg.get("home_timeout_s", 30.0)
        for value in (speed, self.tolerance, tracking, self.timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Home speed, tolerance, tracking limit and timeout must be finite and positive")
        # Homing stays gradual even if policy step/tracking limits were raised.
        self.cfg["max_joint_step_rad"] = min(cfg["max_joint_step_rad"], 0.05)
        self.cfg["max_tracking_error_rad"] = min(cfg["max_tracking_error_rad"], tracking)
        self.step_size = min(speed / cfg["hz"], self.cfg["max_joint_step_rad"])
        self.last_goal = np.array(start, dtype=float, copy=True)
        self.target = self.last_goal.copy()
        self.target[:7] = calibration.robot["home_joints_rad"]
        calibration.encode(self.last_goal)
        calibration.encode(self.target)
        self.safety = Safety(self.cfg, calibration)
        self.settled_ticks = 0

    def step(self, measured):
        """Return the next goal, or None after measured home is confirmed."""
        measured = np.asarray(measured)
        self.safety.tracking(self.last_goal, measured)
        at_target = np.max(np.abs(self.last_goal[:7] - self.target[:7])) < 1e-9
        arrived = np.max(np.abs(measured[:7] - self.target[:7])) <= self.tolerance
        self.settled_ticks = self.settled_ticks + 1 if at_target and arrived else 0
        if self.settled_ticks >= 3:
            return None
        # Do not keep advancing when a joint falls behind its goal. The
        # tracking debounce above turns sustained lag into a latched hold.
        lag = np.max(np.abs(measured[:7] - self.last_goal[:7]))
        if lag > self.cfg["max_tracking_error_rad"]:
            return self.last_goal.copy()
        delta = self.target[:7] - self.last_goal[:7]
        largest = np.max(np.abs(delta))
        goal = self.last_goal.copy()
        if largest > 0:
            goal[:7] += delta * min(1.0, self.step_size / largest)
        goal = self.safety.chunk({self.cfg["action_key"]: goal[None]}, self.last_goal)[0]
        self.safety.before_write(goal, measured)
        self.last_goal = goal.copy()
        return goal


def run_home(robot, cfg, calibration, keyboard, recorder, *, mode, steps,
             start_immediately=False, return_on_hold=False):
    """Space starts/pauses homing; shutdown can start without another arm key."""
    # Validate all home settings and the destination before torque can change.
    initial = robot.read()
    HomeMotion(cfg, calibration, initial)
    logger.info("Home joints (radians): %s", calibration.robot["home_joints_rad"])
    logger.info("Home mode: joint interpolation; gripper width retained. Space: start/hold, q: exit.")
    state, motion, started = "WAITING", None, None
    if mode == "dry-run" or start_immediately:
        motion = HomeMotion(cfg, calibration, robot.engage())
        state, started = "RUNNING", time.monotonic()
    period = 1 / cfg["hz"]
    tick = 0
    try:
        while not steps or tick < steps:
            tick_started = time.monotonic()
            key = keyboard.poll()
            if key == "q":
                logger.info("Home session cancelled; last goal and torque state retained")
                state = "STOPPED"
                return state
            measured = robot.read()
            if key == " " and mode == "hardware":
                if state == "RUNNING":
                    state, motion = "HOLD", None
                    recorder.event("hold", reason="Operator paused homing")
                    logger.info("HOLD: homing paused; Space to resume, q to exit")
                    if return_on_hold:
                        state = "STOPPED"
                        return state
                else:
                    motion = HomeMotion(cfg, calibration, robot.engage())
                    state, started = "RUNNING", time.monotonic()
                    recorder.event("home_armed", goal=motion.target.tolist())
                    logger.info("ARMED: moving gradually toward home")
            if state == "RUNNING":
                try:
                    if time.monotonic() - started > motion.timeout:
                        raise SafetyStop("Homing timed out before measured home was reached")
                    target = motion.step(measured)
                    if target is None:
                        recorder.event("home_reached", measured=measured.tolist())
                        logger.info("Home reached and confirmed by motor feedback; holding position")
                        state = "HOME"
                        return state
                    robot.command(target)
                except Exception as exc:
                    state, motion = "HOLD", None
                    recorder.event("hold", reason=f"Homing stopped: {exc}")
                    logger.warning("HOLD: homing stopped: %s. Last goal and torque retained.", exc)
                    if return_on_hold:
                        return state
            recorder.event("home_tick", tick=tick, state=state, measured=measured.tolist(),
                           goal=None if motion is None else motion.last_goal.tolist())
            tick += 1
            time.sleep(max(0.0, period - (time.monotonic() - tick_started)))
    finally:
        recorder.event("home_stopped", state=state, ticks=tick)
    return "HOLD" if state == "RUNNING" else state
