"""Homing regression tests with simulated feedback; never touch real motors."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller.calibration import Calibration
from controller.hardware import MockRobot
from controller.homing import HomeMotion, run_home
from controller.safety import SafetyStop
from controller.settings import load
from run import main


class Recorder:
    folder = "mock-homing-log"
    def __init__(self, *args):
        self.events = []
    def event(self, kind, **values):
        self.events.append((kind, values))
    def close(self):
        pass


class HomingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load()
        self.cfg.update(home_speed_rad_s=0.2, home_tolerance_rad=0.02,
                        home_max_tracking_error_rad=0.15, home_timeout_s=30.0)
        self.cal = Calibration(self.cfg)
        self.robot = MockRobot(self.cfg, self.cal)
        self.robot.state[:2] += [0.4, 0.2]
        self.robot.state[7] = 0.6
        self.robot.last_goal = self.robot.state.copy()

    def test_home_reached_gradually_and_gripper_retained(self):
        self.robot.engage()
        motion = HomeMotion(self.cfg, self.cal, self.robot.read())
        for tick in range(1000):
            before = self.robot.last_goal.copy()
            target = motion.step(self.robot.read())
            if target is None:
                break
            self.assertLessEqual(np.max(np.abs(target[:7] - before[:7])), 0.2 / self.cfg["hz"] + 1e-9)
            self.assertEqual(target[7], 0.6)
            self.robot.command(target)
        else:
            self.fail("Mock robot never reached home")
        self.assertGreater(tick, 30)
        np.testing.assert_allclose(self.robot.read()[:7], self.cal.robot["home_joints_rad"], atol=0.02)

    def test_large_policy_step_does_not_make_homing_fast(self):
        self.cfg["max_joint_step_rad"] = 0.3
        motion = HomeMotion(self.cfg, self.cal, self.robot.read())
        target = motion.step(self.robot.read())
        self.assertLessEqual(np.max(np.abs(target - self.robot.state)), 0.2 / self.cfg["hz"] + 1e-9)

    def test_stalled_arm_is_rejected(self):
        measured = self.robot.read()
        motion = HomeMotion(self.cfg, self.cal, measured)
        with self.assertRaises(SafetyStop):
            for _ in range(1000):
                motion.step(measured)  # Feedback never moves.

    def test_nonfinite_feedback_rejected(self):
        motion = HomeMotion(self.cfg, self.cal, self.robot.read())
        measured = self.robot.read()
        measured[0] = np.nan
        with self.assertRaises(SafetyStop):
            motion.step(measured)

    def test_invalid_home_settings_rejected(self):
        for key, value in (("home_speed_rad_s", 0), ("home_timeout_s", float("inf"))):
            cfg = dict(self.cfg, **{key: value})
            with self.assertRaises(ValueError):
                HomeMotion(cfg, self.cal, self.robot.read())

    def test_quit_before_arming_never_engages(self):
        self.robot.engage = lambda: self.fail("Torque was enabled before Space")
        keys = type("Keys", (), {"poll": lambda _: "q"})()
        state = run_home(self.robot, self.cfg, self.cal, keys, Recorder(), mode="hardware", steps=1)
        self.assertEqual(state, "STOPPED")

    def test_timeout_latches_hold(self):
        self.cfg["home_timeout_s"] = 0.000001
        keys = type("Keys", (), {"poll": lambda _: ""})()
        recorder = Recorder()
        state = run_home(self.robot, self.cfg, self.cal, keys, recorder, mode="dry-run", steps=2)
        self.assertEqual(state, "HOLD")
        self.assertTrue(any(kind == "hold" for kind, _ in recorder.events))

    def test_home_cli_never_creates_cameras_or_policy(self):
        with patch("controller.cameras.MockCameras") as cameras, \
             patch("controller.transport.PolicyClient") as client, \
             patch("controller.transport.connection_settings", side_effect=AssertionError("Unexpected connection config access")), \
             patch("controller.recording.Recorder", Recorder):
            self.assertEqual(main(["--home", "--mode", "dry-run", "--policy", "websocket", "--steps", "8"]), 0)
            cameras.assert_not_called()
            client.assert_not_called()


if __name__ == "__main__":
    unittest.main()
