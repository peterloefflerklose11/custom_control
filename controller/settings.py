"""Load configuration relative to this folder, never relative to another repo."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def local_path(name):
    """Reject paths or symlinks that escape the standalone folder."""
    path = (ROOT / name).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f"Path must stay inside {ROOT}: {name}")
    return path


def read_json(name):
    return json.loads(local_path(name).read_text())


def load(name="config/runtime.json"):
    cfg = read_json(name)
    # Fail during startup, before opening hardware, for invalid timing/limits.
    import math
    for key in ("hz", "serial_timeout_s", "camera_max_age_s", "inference_timeout_s",
                "max_joint_step_rad", "max_gripper_step", "max_tracking_error_rad"):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    for key in ("chunk_steps", "max_chunk_horizon", "tracking_failure_ticks", "max_attempts"):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    for key in ("home_speed_rad_s", "home_tolerance_rad", "home_max_tracking_error_rad", "home_timeout_s", "camera_retry_interval_s"):
        if key in cfg and (not math.isfinite(cfg[key]) or cfg[key] <= 0):
            raise ValueError(f"{key} must be finite and positive")
    if "return_home_on_exit" in cfg and type(cfg["return_home_on_exit"]) is not bool:
        raise ValueError("return_home_on_exit must be true or false")
    for key, upper in (("servo_speed", 4095), ("servo_acceleration", 50), ("servo_torque_limit", 1000)):
        if type(cfg[key]) is not int or not 0 <= cfg[key] <= upper:
            raise ValueError(f"{key} must be an integer in [0,{upper}]")
    if cfg["servo_torque_limit"] == 0:
        raise ValueError("servo_torque_limit must be positive")
    if cfg["action_mode"] != "absolute_joints":
        raise ValueError("This pipeline accepts absolute joint targets only")
    if type(cfg["hz"]) is not int or type(cfg["baudrate"]) is not int or cfg["baudrate"] <= 0:
        raise ValueError("hz and baudrate must be positive integers")
    if type(cfg["max_message_bytes"]) is not int or cfg["max_message_bytes"] <= 0:
        raise ValueError("max_message_bytes must be a positive integer")
    if not cfg["server_url"].startswith(("ws://", "wss://")):
        raise ValueError("server_url must start with ws:// or wss://")
    for name, camera in cfg["cameras"].items():
        for key in ("width", "height"):
            if type(camera[key]) is not int or camera[key] <= 0:
                raise ValueError(f"Invalid {key} for {name}")
        if name == "overhead" and camera["width"] % 2:
            raise ValueError("Stereo camera width must be even")
    if set(cfg["cameras"]) != {"overhead", "left_wrist"}:
        raise ValueError("Configure the overhead and left wrist cameras")
    if len({camera["device"] for camera in cfg["cameras"].values()}) != 2:
        raise ValueError("Overhead and left wrist cameras must use distinct devices")
    schema = cfg["observation"]
    if set(schema["camera_keys"]) != {"overhead_left", "left_wrist"}:
        raise ValueError("Configure the overhead_left and left_wrist policy images")
    keys = [schema["state_key"], schema["prompt_key"], *schema["camera_keys"].values()]
    if not all(isinstance(k, str) and k for k in keys) or len(set(keys)) != len(keys):
        raise ValueError("Observation keys must be distinct nonempty strings")
    if set(cfg["ports"]) != {"left_follower"}:
        raise ValueError("Configure only the left_follower serial port")
    if cfg["clip_tolerance_rad"] < 0 or not math.isfinite(cfg["clip_tolerance_rad"]):
        raise ValueError("clip_tolerance_rad must be finite and nonnegative")
    for key in ("calibration_file", "robot_file", "rig_map_file"):
        local_path(cfg[key])
    return cfg
