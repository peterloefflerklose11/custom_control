"""Dropped frames/reconnected devices must not send stale actions or end the loop."""
import copy
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller.settings import load
from controller.calibration import Calibration
from controller.hardware import MockRobot
from controller.cameras import Cameras, MockCameras, CameraUnavailableError
from controller.pipeline import run


class Recorder:
    def __init__(self):
        self.events = []
    def event(self, kind, **values):
        self.events.append({"kind": kind, **values})


class Keys:
    def poll(self):
        return ""


class GapCamera(MockCameras):
    def __init__(self, cfg, missing):
        super().__init__(cfg)
        self.tick = 0
        self.missing = set(missing)
        self.unavailable = False
    def frames(self):
        self.tick += 1
        self.unavailable = self.tick in self.missing
        if self.unavailable:
            raise CameraUnavailableError("Camera overhead missing or stale")
        return super().frames()


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load()
        self.cfg.update(max_joint_step_rad=0.05, max_tracking_error_rad=0.15,
                        tracking_failure_ticks=3, camera_max_age_s=0.5,
                        camera_retry_interval_s=0.001)
        self.cal = Calibration(self.cfg)
        self.robot = MockRobot(self.cfg, self.cal)
        self.recorder = Recorder()

    def exercise(self, cameras, *, mode="dry-run", keys=None, steps=8, delayed=False):
        self.submissions = []
        outer = self
        class Worker:
            def __init__(inner, client):
                inner.result = None
            def submit(inner, token, obs):
                outer.assertFalse(cameras.unavailable)
                outer.submissions.append((cameras.tick, token))
                actions = np.repeat(obs["observation.state"][None], 3, axis=0)
                actions[:, 0] += 0.005
                inner.result = (token, {"actions": actions}, None)
            def poll(inner):
                if delayed and cameras.tick < 4:
                    return None
                result, inner.result = inner.result, None
                return result
            def close(inner):
                pass
        command = self.robot.command
        def guarded(target):
            self.assertFalse(cameras.unavailable, "A target was written while the camera was unavailable")
            command(target)
        self.robot.command = guarded
        with patch("controller.pipeline.InferenceWorker", Worker):
            return run(self.robot, cameras, object(), self.cfg, self.cal, keys or Keys(),
                       self.recorder, mode=mode, steps=steps, instruction="test")

    def test_camera_gap_holds_and_requests_fresh_chunk_after_recovery(self):
        cameras = GapCamera(self.cfg, missing={3, 4})
        state = self.exercise(cameras)
        self.assertEqual(state, "RUNNING")
        kinds = [e["kind"] for e in self.recorder.events]
        self.assertEqual(kinds.count("camera_wait_started"), 1)
        self.assertEqual(kinds.count("camera_wait_finished"), 1)
        self.assertGreaterEqual(len(self.submissions), 2)
        self.assertNotEqual(self.submissions[0][1], self.submissions[1][1])
        self.assertTrue(any(e.get("camera_waiting") for e in self.recorder.events))

    def test_reply_from_before_camera_gap_is_discarded(self):
        cameras = GapCamera(self.cfg, missing={2, 3})
        self.exercise(cameras, delayed=True)
        # Old reply arrives on tick 4, after recovery. It must be discarded;
        # the first accepted chunk is from the fresh request on tick 4.
        chunks = [i for i, e in enumerate(self.recorder.events) if e["kind"] == "chunk"]
        recovered = next(i for i, e in enumerate(self.recorder.events) if e["kind"] == "camera_wait_finished")
        self.assertTrue(chunks)
        self.assertGreater(chunks[0], recovered)
        self.assertEqual(self.submissions[:2], [(1, 0), (4, 1)])
        ticks_before_accept = [e for e in self.recorder.events[:chunks[0]] if e["kind"] == "tick"]
        self.assertTrue(all(e["goal"][0] == ticks_before_accept[0]["goal"][0] for e in ticks_before_accept))

    def test_tracking_hold_survives_camera_gap_and_recovery(self):
        self.cfg.update(max_tracking_error_rad=0.001, tracking_failure_ticks=1)
        self.robot.read = lambda: self.robot.state.copy()  # stuck motor
        state = self.exercise(GapCamera(self.cfg, missing={3, 4}))
        self.assertEqual(state, "HOLD")
        self.assertTrue(self.robot.shutdown_home_blocked)
        self.assertEqual(len(self.submissions), 1)
        self.assertTrue(any(e["kind"] == "camera_wait_finished" for e in self.recorder.events))

    def test_cannot_arm_with_missing_frames(self):
        self.robot.engage = lambda: self.fail("Armed without a valid observation")
        keys = type("Keys", (), {"poll": lambda _: " "})()
        state = self.exercise(GapCamera(self.cfg, missing={1, 2, 3}), mode="hardware", keys=keys, steps=3)
        self.assertEqual(state, "WAITING")

    def test_q_still_exits_while_waiting_for_camera(self):
        class Quit:
            count = 0
            def poll(inner):
                inner.count += 1
                return "q" if inner.count == 3 else ""
        cameras = GapCamera(self.cfg, missing=range(1, 100))
        self.exercise(cameras, keys=Quit(), steps=10)
        self.assertEqual(cameras.tick, 2)

    def test_motor_read_failure_is_still_fatal(self):
        count = 0
        def failed_read():
            nonlocal count
            count += 1
            if count > 1:
                raise OSError("fake motor disconnected")
            return self.robot.state.copy()
        self.robot.read = failed_read
        with self.assertRaisesRegex(OSError, "motor disconnected"):
            self.exercise(GapCamera(self.cfg, missing=set()))

    def test_observe_mode_waits_without_arming_or_inference(self):
        self.robot.engage = lambda: self.fail("Read-only mode engaged robot")
        state = self.exercise(GapCamera(self.cfg, missing={1, 2}), mode="observe", steps=4)
        self.assertEqual(state, "WAITING")
        self.assertEqual(self.submissions, [])

    def capture_test(self, *, misses=0, cannot_open=False):
        cfg = copy.deepcopy(self.cfg)
        cfg["cameras"] = {"overhead": {"device": "/dev/fake-overhead", "width": 4, "height": 2}}
        captures = []
        class Capture:
            def __init__(inner):
                inner.index, inner.reads, inner.releases = len(captures), 0, 0
                captures.append(inner)
            def isOpened(inner):
                return not (cannot_open and inner.index == 0)
            def set(inner, *args):
                return True
            def read(inner):
                inner.reads += 1
                time.sleep(0.001)
                if inner.index == 0 and inner.reads <= misses:
                    return False, None
                return True, np.full((2, 4, 3), [10, 20, 30], dtype=np.uint8)
            def release(inner):
                inner.releases += 1
        cameras = Cameras(cfg)
        with patch("cv2.VideoCapture", side_effect=lambda *args: Capture()):
            try:
                cameras.open()
                frames = cameras.frames()
                np.testing.assert_array_equal(frames["overhead_left"][0, 0], [30, 20, 10])
            finally:
                cameras.close()
        self.assertTrue(all(cap.releases == 1 for cap in captures))
        return captures

    def test_isolated_read_miss_recovers_without_reopening_device(self):
        captures = self.capture_test(misses=1)
        self.assertEqual(len(captures), 1)

    def test_three_failed_reads_reopen_the_camera(self):
        captures = self.capture_test(misses=3)
        self.assertEqual(len(captures), 2)
        self.assertEqual(captures[0].reads, 3)

    def test_device_open_failure_retries_and_recovers(self):
        captures = self.capture_test(cannot_open=True)
        self.assertEqual(len(captures), 2)


if __name__ == "__main__":
    unittest.main()
