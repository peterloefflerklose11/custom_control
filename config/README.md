# Configuration guide

`runtime.json` is the active configuration. All paths to other JSON files are
relative to the top of `custom_control`, regardless of the shell's directory.
The loader refuses paths and symlinks that escape this folder.

| Setting | Meaning |
|---|---|
| `calibration_file` | Local copy of motor ranges and EEPROM offsets |
| `robot_file` | Local joint limits and mock home pose |
| `rig_map_file` | Local adapter identities and wiring notes |
| `ports` | Exactly one serial device: `left_follower`; the right bus is not opened |
| `baudrate` | Servo bus speed: this rig uses 1,000,000 baud |
| `serial_timeout_s` | Maximum wait for each motor register reply |
| `hz` | Requested control ticks per second; default 30 |
| `home_speed_rad_s` | Maximum commanded homing joint speed, default 0.2 rad/s |
| `home_tolerance_rad` | Measured joint error allowed to confirm home, default 0.02 rad |
| `home_max_tracking_error_rad` | Homing tracking limit, default 0.15 rad; policy tracking limit can make it stricter |
| `home_timeout_s` | Maximum duration per armed homing attempt, default 30 seconds |
| `return_home_on_exit` | Default `true`: return an armed robot home before closing the bus on q/Ctrl-C/step limit; `--no-home-on-exit` disables it |
| `server_url` | Full token-free WebSocket URL, including `/proxy/8000/` for Jupyter; fallback when no URL env var is selected |
| `server_url_env` | Optional URL environment variable name (default `null`), e.g. `POLICY_SERVER_URL`; selected with `--server-url-env` |
| `token_env` | Optional Jupyter token environment variable name (default `null`), e.g. `JUPYTER_TOKEN`; selected with `--token-env` |
| `env_file` | Optional local KEY=value file, e.g. `config/station.env`; selected with `--env-file`. Exported environment values take precedence. |
| `inference_timeout_s` | Maximum connection/receive wait per attempt |
| `max_attempts` | Total attempts for a transient inference failure |
| `max_message_bytes` | Maximum accepted WebSocket message size |
| `instruction` | Task text; overridden by `--instruction` |
| `action_mode` | Must be `absolute_joints`; other modes are rejected |
| `action_key` | Reply dictionary key containing the `(H,8)` left-arm action array |
| `chunk_steps` | Maximum number of actions to execute from an accepted chunk |
| `max_chunk_horizon` | Maximum accepted number of rows before truncation |
| `max_joint_step_rad` | Maximum per-joint change from previous command, also checked for the first target |
| `max_gripper_step` | Maximum change in normalized gripper width per tick |
| `max_tracking_error_rad` | Maximum normal commanded-versus-measured joint difference |
| `tracking_failure_ticks` | Consecutive excessive tracking readings before HOLD |
| `clip_tolerance_rad` | Small allowed joint-limit overshoot; values inside this tolerance are clipped and logged |
| `camera_max_age_s` | Age after which a frame is skipped and policy execution waits for fresh images |
| `camera_retry_interval_s` | Delay between reopening a failed/unavailable camera, default 1.0 second |
| `servo_speed` | GoalSpeed RAM register; 0 means the servo's profile speed limit is disabled |
| `servo_acceleration` | Acc RAM register; this rig's follower firmware accepts up to 50 |
| `servo_torque_limit` | TorqueLimit RAM register, up to 1000; EEPROM MaxTorque still caps it |
| `cameras` | Overhead stereo capture at 1600×600 and left wrist capture at 640×360 |
| `observation.state_key` | Model input key for the fixed 8-value left-arm state |
| `observation.prompt_key` | Model input key for the task text |
| `observation.camera_keys` | `overhead_left` → `observation.images.overhead` (800×600 RGB), and `left_wrist` → `observation.images.wrist` (640×360 RGB) |

The robot and calibration files work together:

```text
joint angle = lower + (count - range_min) / (range_max - range_min) * (upper - lower)
gripper width = (count - range_min) / (range_max - range_min)
```

`lower` and `upper` come from `robot.json`; count ranges come from
`calibration.json`. Gripper widths are normalized to [0,1], with 0 closed.
Offsets are already applied by the motor firmware; this controller checks the
saved offsets against EEPROM and does not add them again in the conversion.

The active layout is `[left joint 1, ..., left joint 7, left gripper]`.
Finite gripper overshoots are always clipped to [0,1] before checking
`max_gripper_step`; no extra clipping parameter is needed. NaN/inf are rejected.
The calibration/device-map copies retain entries for the full original rig,
but only the left follower is loaded or accessed for motor control. The overhead
and left wrist cameras are required; the right arm and right wrist camera are
not opened. Sixteen-value action chunks are rejected. Configure the remote
policy for 8 state/action values and both the overhead and left wrist images.

For Jupyter proxy authentication, export the token-free URL and your token in
the terminal, then run:

```bash
.venv/bin/python run.py --mode hardware --policy websocket --steps 0 \
  --server-url-env POLICY_SERVER_URL --token-env JUPYTER_TOKEN
```

Run this command from the top of `custom_control`. Both arguments are variable
names, not values. The URL order is `--server-url` > selected URL environment
variable > `server_url`. Missing selected variables are errors. The client
URL-encodes and appends `?token=...` (or `&token=...`) only when connecting.
Put only variable names in JSON; keep the token itself out of URLs/config files.
The main README contains the full terminal setup and Jupyter tunnel example.
For the working example's file-based token fallback, fill in
`config/station.env` using the included template and run:

```bash
.venv/bin/python run.py --check-connection \
  --env-file config/station.env --server-url-env POLICY_SERVER_URL
```

This connects and checks metadata without opening cameras or motor buses.
Selecting an env file defaults the token name to `JUPYTER_TOKEN`. An already
exported token overrides the file, so check for an old terminal value when
investigating HTTP 403. The token itself is never written to snapshots.

`robot.json` retains the original rig's geometric constants and home pose.
Geometry is not used for collision checking. The mock robot starts at the
saved home. To move the real arm there explicitly, run
`.venv/bin/python run.py --mode hardware --home --steps 0` from the folder root,
then press Space. This uses gradual joint interpolation and retains the gripper
width; ensure the path is clear. It opens no cameras or policy connection.
An armed policy session also returns to this same home before normal exit by
default, without another Space press. Never-armed/read-only sessions and
sensor/tracking/hardware faults skip that move. Torque remains enabled at home.
Temporary camera gaps are recoverable: the loop keeps reading motors and
processing keyboard commands while holding the last goal. Policy execution
resumes with a fresh observation when both images recover, unless a tracking
or operator HOLD was already latched. Camera recovery never clears that HOLD.

Files in `reference/` are archived copies of original station/camera settings.
They are not active controller settings. In particular, changing the archived
station YAML will have no effect on this controller.
