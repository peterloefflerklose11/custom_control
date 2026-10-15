#!/usr/bin/env python3
"""Entry point: parse options and connect the small purpose-specific modules."""
import argparse
import logging
import sys
from controller.settings import load


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/runtime.json", help="JSON path inside this folder")
    parser.add_argument("--mode", choices=("dry-run", "observe", "hardware"), default="dry-run")
    parser.add_argument("--policy", choices=("mock", "websocket"), default="mock")
    parser.add_argument("--server-url", help="Override the saved ws:// or wss:// URL")
    parser.add_argument("--server-url-env", help="Name of the env var holding the token-free WebSocket URL")
    parser.add_argument("--token-env", help="Name of the env var holding the Jupyter token, e.g. JUPYTER_TOKEN")
    parser.add_argument("--env-file", help="Local KEY=value file, e.g. config/station.env; environment values win")
    parser.add_argument("--check-connection", action="store_true", help="Test authentication and metadata only; open no robot/camera devices")
    parser.add_argument("--instruction", help="Override saved task text")
    parser.add_argument("--steps", type=int, default=300, help="Total control ticks; 0 runs until q/Ctrl-C")
    parser.add_argument("--check-config", action="store_true", help="Validate config; open no devices or sockets")
    parser.add_argument("--home", action="store_true", help="Move gradually to saved home pose; no cameras or policy connection")
    parser.add_argument("--no-home-on-exit", action="store_true", help="Skip the default return-home when an armed policy session exits")
    args = parser.parse_args(argv)
    if args.steps < 0:
        parser.error("--steps cannot be negative")
    if args.home and (args.mode == "observe" or args.check_connection):
        parser.error("--home cannot be combined with observe mode or --check-connection")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    cfg = load(args.config)
    from controller.transport import connection_settings, authenticated_url
    try:
        if not args.home:
            cfg = connection_settings(cfg, server_url=args.server_url,
                                      server_url_env=args.server_url_env, token_env=args.token_env,
                                      env_file=args.env_file)
        if not args.home and (args.check_connection or (args.policy == "websocket" and args.mode != "observe")):
            authenticated_url(cfg["server_url"], cfg.get("token_env"), cfg.get("env_file"))
    except ValueError as exc:
        parser.error(str(exc))
    if args.instruction is not None:
        cfg["instruction"] = args.instruction
    cfg["run_mode"], cfg["policy"] = args.mode, args.policy
    cfg["home"] = args.home
    if args.no_home_on_exit:
        cfg["return_home_on_exit"] = False

    if args.check_connection:
        # Like the working example, connect before constructing any robot.
        from controller.transport import PolicyClient
        client = PolicyClient(cfg)
        try:
            client.check_connection()
            print("Connection check passed: authenticated WebSocket and policy metadata received. No robot/cameras opened.")
            return 0
        except Exception as exc:
            logging.error("Connection check failed: %s", exc)
            return 1
        finally:
            client.close()

    from controller.calibration import Calibration
    calibration = Calibration(cfg)
    if args.check_config:
        print("Configuration valid: left follower, 8 state/action values, overhead and left wrist images, local calibration.")
        return 0
    if args.mode == "hardware" and not sys.stdin.isatty():
        parser.error("Hardware control needs a terminal for space/q controls")

    from controller.hardware import Robot, MockRobot
    from controller.cameras import Cameras, MockCameras
    from controller.transport import PolicyClient, MockPolicy
    from controller.keyboard import Keyboard
    from controller.recording import Recorder
    from controller.pipeline import run
    real = args.mode != "dry-run"
    robot = (Robot if real else MockRobot)(cfg, calibration)
    cameras = None if args.home else (Cameras if real else MockCameras)(cfg)
    client = None if args.home else (PolicyClient if args.policy == "websocket" else MockPolicy)(cfg)
    recorder = Recorder(cfg)
    logging.info("Mode=%s policy=%s. Log: %s", args.mode, args.policy, recorder.folder)
    if args.mode == "hardware":
        logging.info("Space: engage/arm or hold. q/Ctrl-C: stop the current control loop.")
        if args.policy == "websocket" and not args.home:
            logging.info("No server connection is attempted until you press Space. Use --check-connection to test it without arming.")
        if not args.home and cfg.get("return_home_on_exit", True):
            logging.info("After an armed session, q/Ctrl-C or the step limit returns the arm home before exit.")
    exit_code = 0
    failed = False
    try:
        robot.open()
        if cameras is not None:
            cameras.open()
        with Keyboard() as keyboard:
            if args.home:
                from controller.homing import run_home
                final_state = run_home(robot, cfg, calibration, keyboard, recorder,
                                       mode=args.mode, steps=args.steps)
            else:
                final_state = run(robot, cameras, client, cfg, calibration, keyboard, recorder,
                                  mode=args.mode, steps=args.steps, instruction=cfg["instruction"])
        exit_code = 2 if final_state == "HOLD" else 0
    except KeyboardInterrupt:
        logging.info("Policy interrupted; entering shutdown cleanup")
        exit_code = 130
    except Exception:
        failed = True
        logging.exception("Stopped. Last hardware goal/torque state retained.")
        exit_code = 1
    finally:
        try:
            if not args.home:
                from controller.shutdown import finish_session
                result = finish_session(robot, cfg, calibration, recorder, mode=args.mode, failed=failed)
                if result in ("FAILED", "CANCELLED") and exit_code == 0:
                    exit_code = 1 if result == "FAILED" else 2
        finally:
            robot.close()
            if cameras is not None:
                cameras.close()
            recorder.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
