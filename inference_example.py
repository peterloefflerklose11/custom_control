#!/usr/bin/env python3
"""Run SO-101 inference against the remote OpenPI websocket policy server.

The loop is:

1. Read the follower state and camera observation.
2. Send the observation to the cloud policy server.
3. Receive an action chunk.
4. Execute each action in the chunk on the follower.
5. Repeat until Ctrl-C, --max-steps, or --max-chunks stops the program.
"""

from __future__ import annotations

import argparse
import os
import shlex
import signal
import sys
import time
from pathlib import Path
from typing import Any
from urllib.parse import quote

import numpy as np
from openpi_client import websocket_client_policy


SERVER_URL = (
    "wss://port8888-rgrjj05e563nnw4."
    "tunnel.applications.eu-north1.nebius.cloud"
    "/proxy/8000/"
)

MOTOR_KEYS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)


def parse_env_file(path: Path) -> dict[str, str]:
    """Parse the simple KEY=value env files used by the repo scripts."""

    values: dict[str, str] = {}
    if not path.exists():
        return values

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            continue

        try:
            values[key] = shlex.split(value, comments=False, posix=True)[0]
        except IndexError:
            values[key] = ""

    return values


def load_config(station_env: Path | None) -> dict[str, str]:
    so101_root = Path(__file__).resolve().parents[2]
    repo_root = so101_root.parent

    config = dict(os.environ)
    config.update(parse_env_file(repo_root / ".env"))

    station_path = station_env or Path(config.get("STATION_ENV", so101_root / "station.env"))
    if not station_path.exists():
        raise FileNotFoundError(
            f"Missing {station_path}. Copy so101/station.env.example to so101/station.env "
            "and fill in the follower port and camera indexes."
        )
    config.update(parse_env_file(station_path))

    return config


def require_lerobot_imports() -> tuple[Any, Any, Any]:
    try:
        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    except ImportError as exc:
        raise SystemExit(
            "Could not import LeRobot SO-101 support. Run this from so101 after `uv sync`, "
            "for example: `uv run python inference/openpi/inference.py`."
        ) from exc

    return OpenCVCameraConfig, SO101Follower, SO101FollowerConfig


def build_camera_config(config: dict[str, str]) -> dict[str, Any]:
    OpenCVCameraConfig, _, _ = require_lerobot_imports()

    width = int(config.get("CAMERA_WIDTH", "640"))
    height = int(config.get("CAMERA_HEIGHT", "480"))
    fps = int(config.get("CAMERA_FPS", "30"))

    if not config.get("CAMERA_INDEX"):
        raise ValueError("CAMERA_INDEX is required in station.env")

    cameras = {
        "front": OpenCVCameraConfig(
            index_or_path=config["CAMERA_INDEX"],
            width=width,
            height=height,
            fps=fps,
        )
    }

    if config.get("CAMERA_INDEX_TOP"):
        if config["CAMERA_INDEX_TOP"] == config["CAMERA_INDEX"]:
            raise ValueError("CAMERA_INDEX_TOP must differ from CAMERA_INDEX")
        cameras["top"] = OpenCVCameraConfig(
            index_or_path=config["CAMERA_INDEX_TOP"],
            width=width,
            height=height,
            fps=fps,
        )

    return cameras


def connect_robot(config: dict[str, str]) -> Any:
    _, SO101Follower, SO101FollowerConfig = require_lerobot_imports()

    if not config.get("FOLLOWER_PORT"):
        raise ValueError("FOLLOWER_PORT is required in station.env")
    if not config.get("FOLLOWER_ID"):
        raise ValueError("FOLLOWER_ID is required in station.env")

    robot_config = SO101FollowerConfig(
        port=config["FOLLOWER_PORT"],
        id=config["FOLLOWER_ID"],
        cameras=build_camera_config(config),
    )
    robot = SO101Follower(robot_config)
    robot.connect()
    return robot


def normalize_image(image: Any, size: int) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    if array.ndim == 2:
        array = np.repeat(array[..., None], 3, axis=-1)
    if array.shape[-1] == 4:
        array = array[..., :3]
    if array.shape[:2] == (size, size):
        return array

    try:
        from PIL import Image
    except ImportError as exc:
        raise SystemExit(
            "Pillow is required to resize camera frames. Install inference requirements."
        ) from exc

    return np.asarray(Image.fromarray(array).resize((size, size), Image.Resampling.BILINEAR))


def motor_state(observation: dict[str, Any]) -> dict[str, float]:
    missing = [key for key in MOTOR_KEYS if key not in observation]
    if missing:
        raise KeyError(f"Robot observation is missing motor keys: {missing}")
    return {key: float(observation[key]) for key in MOTOR_KEYS}


def create_policy_observation(
    robot_observation: dict[str, Any],
    *,
    prompt: str,
    image_size: int,
    wrist_camera: str,
) -> dict[str, Any]:
    state = motor_state(robot_observation)

    front = normalize_image(robot_observation["front"], image_size)
    wrist_source = wrist_camera if wrist_camera in robot_observation else "front"
    wrist = normalize_image(robot_observation[wrist_source], image_size)

    return {
        "observation/exterior_image_1_left": front,
        "observation/wrist_image_left": wrist,
        "observation.state": np.array([state[key] for key in MOTOR_KEYS], dtype=np.float32),
        "prompt": prompt,
    }


def extract_actions(result: Any) -> np.ndarray:
    if isinstance(result, dict):
        if "actions" not in result:
            raise KeyError(f"Policy response did not include 'actions'. Keys: {list(result)}")
        actions = result["actions"]
    else:
        actions = result

    actions = np.asarray(actions, dtype=np.float32)
    if actions.ndim == 1:
        actions = actions[None, :]
    if actions.ndim != 2:
        raise ValueError(f"Expected action chunk with shape (chunk, dim), got {actions.shape}")
    return actions


def action_row_to_robot_action(row: np.ndarray, layout: str) -> dict[str, float]:
    if layout == "so101":
        if len(row) < len(MOTOR_KEYS):
            raise ValueError(
                f"SO101 action layout needs at least {len(MOTOR_KEYS)} values, got {len(row)}"
            )
        values = row[: len(MOTOR_KEYS)]
    elif layout == "droid":
        body_keys = [key for key in MOTOR_KEYS if key != "gripper.pos"]
        if len(row) < len(body_keys) + 1:
            raise ValueError(
                f"DROID action layout needs at least {len(body_keys) + 1} values, got {len(row)}"
            )
        values = np.array([*row[: len(body_keys)], row[-1]], dtype=np.float32)
    else:
        raise ValueError(f"Unsupported action layout: {layout}")

    return {key: float(value) for key, value in zip(MOTOR_KEYS, values, strict=True)}


def authenticated_url(server_url: str, token_env: str, config: dict[str, str]) -> str:
    token = os.environ.get(token_env) or config.get(token_env)
    if not token:
        raise SystemExit(f"Set {token_env} in the terminal before starting inference.")
    separator = "&" if "?" in server_url else "?"
    return f"{server_url}{separator}token={quote(token, safe='')}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--server-url",
        default=SERVER_URL,
        help="Policy websocket URL without token.",
    )
    parser.add_argument(
        "--token-env",
        default="JUPYTER_TOKEN",
        help="Env var containing the Jupyter token.",
    )
    parser.add_argument("--station-env", type=Path, default=None, help="Path to station.env.")
    parser.add_argument(
        "--task",
        default=None,
        help="Prompt sent to the policy. Defaults to station.env TASK.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=224,
        help="Square image size expected by the policy.",
    )
    parser.add_argument(
        "--wrist-camera",
        default="top",
        help="Camera key to use as wrist_image_left.",
    )
    parser.add_argument("--action-layout", choices=("so101", "droid"), default="so101")
    parser.add_argument(
        "--control-fps",
        type=float,
        default=None,
        help="Action execution rate. Defaults to CAMERA_FPS.",
    )
    parser.add_argument(
        "--max-chunks",
        type=int,
        default=None,
        help="Stop after this many policy chunks.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Stop after this many robot actions.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Connect to policy but print actions instead of moving.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.station_env)
    prompt = args.task or config.get("TASK", "")
    control_fps = args.control_fps or float(config.get("CAMERA_FPS", "30"))
    sleep_s = 1.0 / control_fps if control_fps > 0 else 0.0

    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    client = websocket_client_policy.WebsocketClientPolicy(
        host=authenticated_url(args.server_url, args.token_env, config)
    )
    print("Connected to cloud policy server.")
    print("Server metadata:", client.get_server_metadata())

    robot = None
    chunks = 0
    steps = 0

    try:
        robot = connect_robot(config)
        print("Connected to SO-101 follower.")
        if args.dry_run:
            print(
                "Dry run enabled: observations will be sent, "
                "but motor actions will not be executed."
            )

        while not stop:
            if args.max_chunks is not None and chunks >= args.max_chunks:
                break
            if args.max_steps is not None and steps >= args.max_steps:
                break

            robot_observation = robot.get_observation()
            policy_observation = create_policy_observation(
                robot_observation,
                prompt=prompt,
                image_size=args.image_size,
                wrist_camera=args.wrist_camera,
            )

            start = time.perf_counter()
            result = client.infer(policy_observation)
            actions = extract_actions(result)
            chunks += 1
            latency_ms = (time.perf_counter() - start) * 1000
            print(f"Chunk {chunks}: received {actions.shape[0]} actions in {latency_ms:.1f} ms")

            for row in actions:
                if stop or (args.max_steps is not None and steps >= args.max_steps):
                    break
                action = action_row_to_robot_action(row, args.action_layout)
                sent_action = action if args.dry_run else robot.send_action(action)
                steps += 1
                print(f"  step {steps}: {sent_action}")
                if sleep_s > 0:
                    time.sleep(sleep_s)

    finally:
        if robot is not None and getattr(robot, "is_connected", False):
            robot.disconnect()
            print("Disconnected SO-101 follower.")

    print(f"Stopped after {chunks} chunks and {steps} actions.")
    return 0


if __name__ == "__main__":
    sys.exit(main())