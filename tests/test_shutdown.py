"""Verify homing precedes cleanup, with no physical hardware or remote calls."""
from contextlib import nullcontext
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller.settings import load
from controller.calibration import Calibration
from controller.hardware import MockRobot
from controller.shutdown import finish_session
from controller.pipeline import run
from controller.cameras import MockCameras
from controller.transport import MockPolicy
from run import main


class Recorder:
    folder = "mock-shutdown-log"
    def __init__(self, *args):
        self.events = []
    def event(self, kind, **values):
        self.events.append((kind, values))
    def close(self):
        self.events.append(("recorder_closed", {}))


class NoKeys:
    def poll(self):
        return ""


class ShutdownTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load()
        self.cfg.update(return_home_on_exit=True, home_speed_rad_s=0.2,
                        home_tolerance_rad=0.02, home_timeout_s=30.0,
                        home_max_tracking_error_rad=0.15, max_joint_step_rad=0.05,
                        max_tracking_error_rad=0.15, tracking_failure_ticks=3)
        self.cal = Calibration(self.cfg)
        self.robot = MockRobot(self.cfg, self.cal)
        self.robot.state[:2] += [0.2, 0.1]
        self.robot.last_goal = self.robot.state.copy()
        self.recorder = Recorder()

    def finish(self, **kwargs):
        with patch("controller.shutdown.Keyboard", return_value=nullcontext(NoKeys())), \
             patch("controller.homing.time.sleep"):
            return finish_session(self.robot, self.cfg, self.cal, self.recorder,
                                  mode="hardware", **kwargs)

    def test_armed_robot_returns_home_without_another_space(self):
        self.robot.engage()
        self.assertEqual(self.finish(), "HOME")
        np.testing.assert_allclose(self.robot.read()[:7], self.cal.robot["home_joints_rad"], atol=0.02)
        self.assertTrue(self.robot.engaged)  # Never drop torque at home.
        self.assertTrue(any(k == "home_reached" for k, _ in self.recorder.events))

    def test_never_armed_session_does_not_move(self):
        self.robot.engage = lambda: self.fail("Unarmed robot unexpectedly engaged")
        self.assertEqual(self.finish(), "SKIPPED")

    def test_read_only_session_does_not_move(self):
        self.robot.engage()
        with patch("controller.shutdown.run_home") as home:
            self.assertEqual(finish_session(self.robot, self.cfg, self.cal, self.recorder, mode="observe"), "SKIPPED")
            home.assert_not_called()

    def test_exit_setting_can_disable_return(self):
        self.robot.engage()
        self.cfg["return_home_on_exit"] = False
        self.assertEqual(self.finish(), "SKIPPED")

    def test_exception_or_latched_tracking_fault_skips_return(self):
        self.robot.engage()
        self.assertEqual(self.finish(failed=True), "SKIPPED")
        self.robot.shutdown_home_blocked = True
        self.assertEqual(self.finish(), "SKIPPED")

    def test_user_can_cancel_automatic_return(self):
        self.robot.engage()
        for key in ("q", " "):
            with self.subTest(key=key):
                keys = type("Keys", (), {"poll": lambda _, value=key: value})()
                with patch("controller.shutdown.Keyboard", return_value=nullcontext(keys)):
                    self.assertEqual(finish_session(self.robot, self.cfg, self.cal, self.recorder, mode="hardware"), "CANCELLED")

    def test_homing_failure_exits_instead_of_waiting_forever(self):
        self.robot.engage()
        with patch("controller.shutdown.run_home", return_value="HOLD") as home:
            self.assertEqual(self.finish(), "FAILED")
            self.assertTrue(home.call_args.kwargs["return_on_hold"])
            self.assertTrue(home.call_args.kwargs["start_immediately"])

    def test_second_interrupt_cancels_return(self):
        self.robot.engage()
        with patch("controller.shutdown.run_home", side_effect=KeyboardInterrupt):
            self.assertEqual(self.finish(), "CANCELLED")

    def test_bus_failure_during_homing_is_reported(self):
        self.robot.engage()
        with patch("controller.shutdown.run_home", side_effect=OSError("fake bus error")):
            self.assertEqual(self.finish(), "FAILED")

    def exercise_main(self, *, interrupted=False, failure=False, disabled=False):
        def policy_loop(robot, *args, **kwargs):
            robot.engage()
            self.recorder.event("policy_finished")
            if interrupted:
                raise KeyboardInterrupt
            if failure:
                raise OSError("fake sensor failure")
            return "RUNNING"
        def close_robot():
            self.recorder.event("robot_closed", measured=self.robot.read().tolist())
        self.robot.close = close_robot
        args = ["--mode", "hardware", "--steps", "1"]
        if disabled:
            args.append("--no-home-on-exit")
        with patch("run.sys.stdin.isatty", return_value=True), \
             patch("run.load", return_value=dict(self.cfg)), \
             patch("controller.hardware.Robot", return_value=self.robot), \
             patch("controller.cameras.Cameras"), \
             patch("controller.recording.Recorder", return_value=self.recorder), \
             patch("controller.pipeline.run", side_effect=policy_loop), \
             patch("controller.keyboard.Keyboard", return_value=nullcontext(NoKeys())), \
             patch("controller.shutdown.Keyboard", return_value=nullcontext(NoKeys())), \
             patch("controller.homing.time.sleep"):
            return main(args)

    def test_main_homes_before_serial_close_on_normal_exit(self):
        self.assertEqual(self.exercise_main(), 0)
        kinds = [kind for kind, _ in self.recorder.events]
        self.assertLess(kinds.index("policy_finished"), kinds.index("home_reached"))
        self.assertLess(kinds.index("home_reached"), kinds.index("robot_closed"))
        self.assertLess(kinds.index("robot_closed"), kinds.index("recorder_closed"))

    def test_main_homes_after_first_ctrl_c(self):
        self.assertEqual(self.exercise_main(interrupted=True), 130)
        self.assertTrue(any(k == "home_reached" for k, _ in self.recorder.events))

    def test_main_skips_home_after_fatal_sensor_failure(self):
        self.assertEqual(self.exercise_main(failure=True), 1)
        self.assertFalse(any(k == "shutdown_home_started" for k, _ in self.recorder.events))

    def test_main_exit_opt_out_preserves_pose(self):
        before = self.robot.state.copy()
        self.assertEqual(self.exercise_main(disabled=True), 0)
        np.testing.assert_array_equal(self.robot.state, before)

    def test_tracking_fault_is_latched_for_shutdown(self):
        cfg = dict(self.cfg, max_tracking_error_rad=0.001, tracking_failure_ticks=1)
        # Emulate a stuck motor: commands change the goal but not the reading.
        self.robot.read = lambda: self.robot.state.copy()
        policy = MockPolicy(cfg)
        def moving(obs):
            chunk = np.repeat(obs["observation.state"][None], 3, axis=0)
            chunk[:, 0] += 0.005
            return {"actions": chunk}
        policy.infer = moving
        run(self.robot, MockCameras(cfg), policy, cfg, self.cal, NoKeys(),
            self.recorder, mode="dry-run", steps=8, instruction="move")
        self.assertTrue(self.robot.shutdown_home_blocked)


if __name__ == "__main__":
    unittest.main()
