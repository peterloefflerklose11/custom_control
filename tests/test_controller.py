"""Hardware-free regression checks, including a captured real servo packet."""
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller.settings import load, local_path
from controller.calibration import Calibration, ROLES
from controller.protocol import (packet, sync_packet, normal_position, ServoBus,
                                 GOAL_POSITION, PRESENT_POSITION, TORQUE_ENABLE, MODE, OFFSET)
from controller.hardware import Robot, MockRobot
from controller.cameras import MockCameras, Cameras
from controller.observation import build
from controller.safety import Safety, SafetyStop
from controller.transport import MockPolicy
from controller.pipeline import run


class MemoryRecorder:
    def __init__(self):
        self.events = []

    def event(self, kind, **values):
        self.events.append({"kind": kind, **values})


class NoKeys:
    def poll(self):
        return ""


class FakePort:
    def __init__(self, response):
        self.response = bytearray(response)
        self.sent = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def read(self, size):
        out = bytes(self.response[:size])
        del self.response[:size]
        return out


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load()
        # Test fixed boundaries independently of the operator's edited limits.
        self.cfg.update(max_joint_step_rad=0.05, max_gripper_step=0.15,
                        max_tracking_error_rad=0.15, tracking_failure_ticks=3,
                        clip_tolerance_rad=0.05, camera_max_age_s=0.5)
        self.cal = Calibration(self.cfg)
        self.robot = MockRobot(self.cfg, self.cal)
        self.pose = self.robot.read()
        self.safety = Safety(self.cfg, self.cal)

    def test_local_paths_cannot_escape(self):
        with self.assertRaises(ValueError):
            local_path("../elrobot_assets/rig/calibration.json")

    def test_calibration_round_trip_and_gripper_direction(self):
        encoded = self.cal.encode(self.pose)
        decoded = self.cal.decode(encoded)
        np.testing.assert_allclose(decoded, self.pose, atol=0.004)
        closed, opened = self.pose.copy(), self.pose.copy()
        closed[7], opened[7] = 0, 1
        for role in ROLES:
            self.assertLess(self.cal.encode(closed)[role][8], self.cal.encode(opened)[role][8])

    def test_reject_nonfinite_and_out_of_range_motor_goals(self):
        for bad in (np.nan, np.inf, 100):
            pose = self.pose.copy()
            pose[0] = bad
            with self.assertRaises(ValueError):
                self.cal.encode(pose)

    def test_captured_syncwrite_packet(self):
        # Copied test vector is independently documented in reference/protocol.
        values = dict(enumerate([247, 718, 2854, 624, 298, 2163, 1742, 3574], 1))
        expected = bytes([255,255,254,28,131,42,2,1,247,0,2,206,2,3,38,11,
                          4,112,2,5,42,1,6,115,8,7,206,6,8,246,13,43])
        self.assertEqual(sync_packet(0x2A, 2, values), expected)

    def test_real_serial_io_over_pseudoterminal(self):
        # Exercise pyserial itself, connected to an emulated servo rather than
        # a physical USB adapter. This catches read/write integration mistakes.
        import os
        import pty
        import select
        import threading
        master, slave = pty.openpty()
        requests, errors = [], []
        def emulate():
            try:
                if not select.select([master], [], [], 2)[0]:
                    raise TimeoutError("No serial read request received")
                data = bytearray()
                while len(data) < 8:
                    if not select.select([master], [], [], 2)[0]:
                        raise TimeoutError("Incomplete request")
                    data.extend(os.read(master, 8 - len(data)))
                requests.append(bytes(data))
                os.write(master, packet(1, 0, b"\x00\x08"))
            except Exception as exc:
                errors.append(exc)
        bus = None
        try:
            bus = ServoBus(os.ttyname(slave), 1000000, 0.2)
            thread = threading.Thread(target=emulate, daemon=True)
            thread.start()
            self.assertEqual(bus.read(1, PRESENT_POSITION, 2), 2048)
            thread.join(timeout=3)
            self.assertFalse(errors)
            self.assertEqual(requests, [packet(1, 2, bytes([PRESENT_POSITION, 2]))])
        finally:
            if bus is not None:
                bus.close()
            os.close(master)
            os.close(slave)

    def test_signed_motor_position(self):
        self.assertEqual(normal_position(0x8001), 4095)
        self.assertEqual(normal_position(2048), 2048)

    def test_serial_response_handles_fragmentation_and_noise(self):
        bus = ServoBus.__new__(ServoBus)
        bus.timeout = 0.01
        bus.port = FakePort(b"noise" + packet(1, 0, (2048).to_bytes(2, "little")))
        self.assertEqual(bus.read(1, PRESENT_POSITION, 2), 2048)
        self.assertEqual(bus.port.sent, [packet(1, 2, bytes([PRESENT_POSITION, 2]))])

    def test_serial_response_rejects_checksum_status_and_wrong_id(self):
        good = packet(1, 0, b"\x00\x08")
        bad = good[:-1] + bytes([good[-1] ^ 1])
        for response in (bad, packet(1, 4, b"\x00\x08"), packet(2, 0, b"\x00\x08")):
            bus = ServoBus.__new__(ServoBus)
            bus.timeout, bus.port = 0.01, FakePort(response)
            with self.assertRaises(OSError):
                bus.read(1, PRESENT_POSITION, 2)

    def test_missing_servo_times_out(self):
        bus = ServoBus.__new__(ServoBus)
        bus.timeout, bus.port = 0.001, FakePort(b"")
        with self.assertRaises(TimeoutError):
            bus.read(8, PRESENT_POSITION, 2)

    def test_whole_chunk_checked_before_shortening(self):
        reply = {"actions": np.repeat(self.pose[None], 40, axis=0)}
        reply["actions"][-1, 2] = np.nan
        with self.assertRaises(SafetyStop):
            self.safety.chunk(reply, self.pose)

    def test_large_entry_and_internal_jump_refused(self):
        for index in (0, 2):
            chunk = np.repeat(self.pose[None], 3, axis=0)
            chunk[index, 0] += 0.2
            with self.assertRaises(SafetyStop):
                self.safety.chunk({"actions": chunk}, self.pose)

    def test_cartesian_shape_and_batched_shape_refused(self):
        for chunk in (np.zeros((3, 14)), np.zeros((3, 16)), np.zeros((1, 3, 8)), np.zeros((0, 8))):
            with self.assertRaises(SafetyStop):
                self.safety.chunk({"actions": chunk}, self.pose)

    def test_small_valid_steps_and_shortening(self):
        chunk = np.repeat(self.pose[None], 40, axis=0)
        chunk[:, 0] += np.arange(40) * 0.01
        result = self.safety.chunk({"actions": chunk}, self.pose)
        self.assertEqual(result.shape, (30, 8))

    def test_gripper_bounds_and_step(self):
        for value in (1.1, -0.1, 0.9):
            chunk = self.pose[None].copy()
            chunk[0, 7] = value
            with self.assertRaises(SafetyStop):
                self.safety.chunk({"actions": chunk}, self.pose)

    def test_gripper_overshoots_are_clipped_before_step_check(self):
        for raw, start, expected in ((1.1, 0.9, 1.0), (-0.1, 0.1, 0.0),
                                     (50.0, 0.9, 1.0), (-50.0, 0.1, 0.0)):
            reference = self.pose.copy()
            reference[7] = start
            chunk = reference[None].copy()
            chunk[0, 7] = raw
            with self.assertLogs(level="WARNING"):
                result = self.safety.chunk({"actions": chunk}, reference)
            self.assertEqual(result[0, 7], expected)
            self.assertEqual(chunk[0, 7], raw)  # Preserve the received array.

    def test_nonfinite_gripper_targets_still_rejected(self):
        for value in (np.nan, np.inf, -np.inf):
            chunk = self.pose[None].copy()
            chunk[0, 7] = value
            with self.assertRaisesRegex(SafetyStop, "Non-finite"):
                self.safety.chunk({"actions": chunk}, self.pose)

    def test_tracking_debounce_and_reset(self):
        bad = self.pose.copy()
        bad[0] += 0.3
        self.safety.tracking(self.pose, bad)
        self.safety.tracking(self.pose, self.pose)
        self.safety.tracking(self.pose, bad)
        self.safety.tracking(self.pose, bad)
        with self.assertRaises(SafetyStop):
            self.safety.tracking(self.pose, bad)

    def test_next_goal_checked_against_measured_pose(self):
        goal = self.pose.copy()
        goal[0] += 0.3
        with self.assertRaises(SafetyStop):
            self.safety.before_write(goal, self.pose)

    def test_observation_keys_shapes_and_stereo_split(self):
        frames = MockCameras(self.cfg).frames()
        obs = build(self.pose, frames, "pick up brick", self.cfg)
        self.assertEqual(set(obs), {"task", "observation.state", "observation.images.overhead",
                                    "observation.images.wrist"})
        self.assertEqual(obs["observation.state"].shape, (8,))
        self.assertEqual(obs["observation.state"].dtype, np.float32)
        self.assertEqual(set(frames), {"overhead_left", "left_wrist"})
        self.assertEqual(frames["overhead_left"].shape, (600, 800, 3))
        self.assertEqual(frames["left_wrist"].shape, (360, 640, 3))
        del frames["overhead_left"]
        with self.assertRaises(KeyError):
            build(self.pose, frames, "", self.cfg)

    def test_stale_camera_is_refused(self):
        cameras = Cameras(self.cfg)
        cameras.latest = {n: (time.monotonic() - 2, np.zeros((2, 2, 3), np.uint8))
                          for n in self.cfg["cameras"]}
        with self.assertRaises(RuntimeError):
            cameras.frames()

    def test_dry_run_pipeline_executes_chunks(self):
        recorder = MemoryRecorder()
        state = run(self.robot, MockCameras(self.cfg), MockPolicy(self.cfg),
                    self.cfg, self.cal, NoKeys(), recorder, mode="dry-run", steps=8, instruction="hold")
        self.assertEqual(state, "RUNNING")
        self.assertTrue(any(e["kind"] == "chunk" for e in recorder.events))

    def test_pipeline_executes_small_motion_and_logs_goal(self):
        policy = MockPolicy(self.cfg)
        def moving_infer(obs):
            chunk = np.repeat(obs["observation.state"][None], 5, axis=0)
            chunk[:, 0] += np.arange(1, 6) * 0.005
            return {"actions": chunk}
        policy.infer = moving_infer
        recorder = MemoryRecorder()
        run(self.robot, MockCameras(self.cfg), policy, self.cfg, self.cal,
            NoKeys(), recorder, mode="dry-run", steps=12, instruction="move")
        self.assertGreater(self.robot.last_goal[0], self.pose[0] + 0.02)
        self.assertTrue(any(e["kind"] == "tick" and e["goal"][0] > self.pose[0] + 0.02
                            for e in recorder.events))

    def test_bad_policy_latches_hold_without_writes(self):
        policy = MockPolicy(self.cfg)
        policy.infer = lambda obs: {"actions": np.full((3, 8), np.nan)}
        self.robot.command = lambda _: self.fail("Rejected chunk reached motors")
        recorder = MemoryRecorder()
        state = run(self.robot, MockCameras(self.cfg), policy, self.cfg, self.cal,
                    NoKeys(), recorder, mode="dry-run", steps=6, instruction="hold")
        self.assertEqual(state, "HOLD")

    def test_observe_never_engages_commands_or_infers(self):
        self.robot.engage = lambda: self.fail("Observe mode engaged torque")
        self.robot.command = lambda _: self.fail("Observe mode wrote a goal")
        policy = MockPolicy(self.cfg)
        policy.infer = lambda _: self.fail("Observe mode called policy")
        run(self.robot, MockCameras(self.cfg), policy, self.cfg, self.cal, NoKeys(),
            MemoryRecorder(), mode="observe", steps=2, instruction="")

    def test_keyboard_hold_discards_late_reply(self):
        import threading
        started, release = threading.Event(), threading.Event()
        policy = MockPolicy(self.cfg)
        def slow_infer(obs):
            started.set()
            release.wait(timeout=1)
            return {"actions": np.repeat(self.pose[None], 3, axis=0)}
        policy.infer = slow_infer
        class Keys:
            count = 0
            def poll(inner):
                inner.count += 1
                if inner.count == 1:
                    return " "  # arm
                if started.is_set() and inner.count >= 2 and not release.is_set():
                    release.set()
                    return " "  # hold while request is in flight
                return ""
        self.robot.command = lambda _: self.fail("Late reply executed after hold")
        recorder = MemoryRecorder()
        state = run(self.robot, MockCameras(self.cfg), policy, self.cfg, self.cal, Keys(),
                    recorder, mode="hardware", steps=8, instruction="")
        self.assertEqual(state, "HOLD")

    def test_engage_writes_left_hold_goals_before_torque(self):
        history = []
        counts = self.cal.encode(self.pose)
        class FakeBus:
            def __init__(inner, role):
                inner.role, inner.registers = role, {}
            def read(inner, motor, addr, width):
                if addr == PRESENT_POSITION:
                    return counts[inner.role][motor]
                return inner.registers[(addr, motor)]
            def write(inner, addr, width, values):
                history.append((inner.role, addr))
                inner.registers.update({(addr, m): v for m, v in values.items()})
        robot = Robot(self.cfg, self.cal)
        robot.buses = {r: FakeBus(r) for r in ROLES}
        pose = robot.engage()
        self.assertTrue(robot.engaged)
        goals = [i for i, (_, a) in enumerate(history) if a == GOAL_POSITION]
        torque = [i for i, (_, a) in enumerate(history) if a == TORQUE_ENABLE]
        self.assertLess(max(goals), min(torque))
        with self.assertRaises(ValueError):
            robot.command(pose + 0.3)
        self.assertTrue(all(a >= TORQUE_ENABLE for _, a in history))  # No EEPROM writes.
        self.assertEqual({role for role, _ in history}, {"left_follower"})

    def test_only_left_arm_opened_read_and_commanded(self):
        counts = self.cal.encode(self.pose)["left_follower"]
        history = []
        saved = self.cal.saved["left_follower"]
        class LeftBus:
            def __init__(inner, path, baudrate, timeout):
                history.append(("open", path))
                inner.registers = {}
            def read(inner, motor, addr, width):
                history.append(("read", motor, addr))
                if addr == MODE:
                    return 0
                if addr == OFFSET:
                    return saved["motors"][str(motor)]["offset"] & 0xFFFF
                if addr == PRESENT_POSITION:
                    return counts[motor]
                return inner.registers[(addr, motor)]
            def write(inner, addr, width, values):
                history.append(("write", addr, dict(values)))
                inner.registers.update({(addr, m): v for m, v in values.items()})
            def close(inner):
                history.append(("close",))
        # No right-arm device is even present in enumeration.
        ports = [SimpleNamespace(device=self.cfg["ports"]["left_follower"], serial_number=saved["serial"])]
        robot = Robot(self.cfg, self.cal)
        with patch("serial.tools.list_ports.comports", return_value=ports), patch("controller.hardware.ServoBus", LeftBus):
            robot.open()
            self.assertEqual(set(robot.buses), {"left_follower"})
            self.assertEqual(set(self.cal.saved), {"left_follower"})
            target = robot.engage()
            target[0] += 0.01
            robot.command(target)
            robot.close()
        self.assertEqual([e for e in history if e[0] == "open"],
                         [("open", self.cfg["ports"]["left_follower"])])
        self.assertTrue(any(e[0] == "write" for e in history))

    def test_top_view_uses_left_half_of_stereo_frame(self):
        cameras = Cameras(self.cfg)
        image = np.zeros((600, 1600, 3), dtype=np.uint8)
        image[:, :800] = [10, 20, 30]
        image[:, 800:] = [200, 210, 220]
        cameras.latest = {"overhead": (time.monotonic(), image)}
        wrist = np.full((360, 640, 3), 42, dtype=np.uint8)
        cameras.latest["left_wrist"] = (time.monotonic(), wrist)
        frames = cameras.frames()
        self.assertEqual(set(frames), {"overhead_left", "left_wrist"})
        np.testing.assert_array_equal(frames["overhead_left"], image[:, :800])
        np.testing.assert_array_equal(frames["left_wrist"], wrist)

    def test_camera_capture_opens_overhead_and_left_wrist_devices(self):
        import cv2
        class Capture:
            def __init__(inner, spec):
                inner.spec = spec
            def isOpened(inner):
                return True
            def set(inner, *args):
                return True
            def read(inner):
                time.sleep(0.005)
                return True, np.zeros((inner.spec["height"], inner.spec["width"], 3), dtype=np.uint8)
            def release(inner):
                pass
        cameras = Cameras(self.cfg)
        specs = {spec["device"]: spec for spec in self.cfg["cameras"].values()}
        with patch("cv2.VideoCapture", side_effect=lambda path, backend: Capture(specs[path])) as capture:
            try:
                cameras.open()
                self.assertEqual(set(cameras.frames()), {"overhead_left", "left_wrist"})
                self.assertEqual(capture.call_count, 2)
                self.assertEqual({call.args for call in capture.call_args_list},
                                 {(path, cv2.CAP_V4L2) for path in specs})
            finally:
                cameras.close()

    def test_config_rejects_extra_arm_or_right_wrist_camera(self):
        import copy
        for extra in ("arm", "camera"):
            cfg = copy.deepcopy(self.cfg)
            if extra == "arm":
                cfg["ports"]["right_follower"] = "/dev/elrobot/right_follower"
            else:
                cfg["cameras"]["right_wrist"] = dict(cfg["cameras"]["overhead"])
            with patch("controller.settings.read_json", return_value=cfg):
                with self.assertRaises(ValueError):
                    load()

    def test_left_wrist_missing_or_stale_is_refused(self):
        cameras = Cameras(self.cfg)
        overhead = np.zeros((600, 1600, 3), dtype=np.uint8)
        cameras.latest = {"overhead": (time.monotonic(), overhead)}
        with self.assertRaisesRegex(RuntimeError, "left_wrist"):
            cameras.frames()
        cameras.latest["left_wrist"] = (time.monotonic() - 2, np.zeros((360, 640, 3), dtype=np.uint8))
        with self.assertRaisesRegex(RuntimeError, "left_wrist"):
            cameras.frames()

    def test_hardware_identity_mismatch_opens_no_bus(self):
        robot = Robot(self.cfg, self.cal)
        fake_ports = [SimpleNamespace(device=p, serial_number="WRONG") for p in self.cfg["ports"].values()]
        with patch("serial.tools.list_ports.comports", return_value=fake_ports), patch("controller.hardware.ServoBus") as bus:
            with self.assertRaises(RuntimeError):
                robot.open()
            bus.assert_not_called()

    def test_copied_sources_match_manifest(self):
        # Verify local copies using only their local hashes: original repo is unnecessary.
        import hashlib
        import json
        manifest = json.loads(local_path("reference/COPIED_FILES.json").read_text())
        for entry in manifest:
            self.assertEqual(hashlib.sha256(local_path(entry["local_copy"]).read_bytes()).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    unittest.main()
