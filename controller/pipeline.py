"""Coordinate observation, inference, validation and motor writes.

WAITING: no torque writes yet. Space engages at the current pose.
RUNNING: execute validated targets; hold the last goal during inference.
HOLD: discard queued actions; space explicitly re-arms after a fresh reading.
After the loop exits, run.py returns an armed robot home before closing it.
"""
from collections import deque
import logging
import time
from .inference import InferenceWorker
from .observation import build
from .safety import Safety, TrackingError
from .cameras import CameraUnavailableError

logger = logging.getLogger(__name__)


def run(robot, cameras, client, cfg, calibration, keyboard, recorder,
        *, mode, steps, instruction):
    safety = Safety(cfg, calibration)
    actions = deque()
    state = "WAITING"
    generation = 0
    pending = None
    last_goal = None
    # Observe mode never constructs an inference worker or writes motor RAM.
    worker = None if mode == "observe" else InferenceWorker(client)
    tick = 0
    period = 1 / cfg["hz"]
    next_tick = time.monotonic()
    overrun_reported = False
    camera_waiting = False

    def hold(reason):
        nonlocal state, generation
        state = "HOLD"
        generation += 1  # Ignore a reply that arrives after the hold command.
        actions.clear()
        recorder.event("hold", reason=reason)
        logger.warning("HOLD: %s. Last servo goals remain active. Space to re-arm, q to exit.", reason)

    try:
        if mode == "dry-run":
            last_goal = robot.engage()
            state = "RUNNING"
        while not steps or tick < steps:
            key = keyboard.poll()
            if key == "q":
                break
            try:
                measured = robot.read()
            except Exception as exc:
                # A sensor fault cannot be recovered using stale cached data.
                # End this session; close() leaves the last hardware goal set.
                hold(f"Sensor failure: {exc}")
                raise

            obs = None
            try:
                frames = cameras.frames()
            except CameraUnavailableError as exc:
                if not camera_waiting:
                    camera_waiting = True
                    generation += 1
                    actions.clear()
                    recorder.event("camera_wait_started", reason=str(exc))
                    logger.warning("Waiting for fresh camera frames: %s. Holding last goal; q still exits.", exc)
            else:
                obs = build(measured, frames, instruction, cfg)
                if camera_waiting:
                    camera_waiting = False
                    recorder.event("camera_wait_finished")
                    logger.info("Fresh camera frames received; camera wait ended")

            if key == " " and mode == "hardware":
                if state == "RUNNING":
                    hold("Operator pressed space")
                elif camera_waiting:
                    logger.warning("Cannot arm until both cameras have fresh frames")
                else:
                    last_goal = robot.engage()
                    safety.bad_tracking_ticks = 0
                    state = "RUNNING"
                    generation += 1
                    actions.clear()
                    recorder.event("armed", goal=last_goal.tolist())
                    logger.info("ARMED. First policy target must be close to this pose.")

            result = worker.poll() if worker is not None else None
            if result is not None:
                token, reply, error = result
                pending = None
                if state == "RUNNING" and token == generation and not camera_waiting:
                    try:
                        if error is not None:
                            raise error
                        chunk = safety.chunk(reply, last_goal)
                        actions.extend(chunk)
                        recorder.event("chunk", actions=chunk.tolist())
                    except Exception as exc:
                        hold(f"Inference/chunk rejected: {exc}")

            if state == "RUNNING":
                try:
                    safety.tracking(last_goal, measured)
                    if not camera_waiting and actions:
                        target = actions.popleft()
                        safety.before_write(target, measured)
                        robot.command(target)
                        last_goal = target.copy()
                    elif not camera_waiting and pending is None:
                        worker.submit(generation, obs)
                        pending = generation
                except Exception as exc:
                    if isinstance(exc, (TrackingError, OSError)):
                        robot.shutdown_home_blocked = True
                    hold(f"Execution rejected: {exc}")

            recorder.event("tick", tick=tick, state=state,
                           camera_waiting=camera_waiting,
                           measured=measured.tolist(),
                           goal=None if last_goal is None else last_goal.tolist())
            if tick % cfg["hz"] == 0:
                logger.info("tick=%s state=%s queued=%s inference_pending=%s camera_waiting=%s",
                            tick, state, len(actions), pending is not None, camera_waiting)
            tick += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                if not overrun_reported:
                    logger.warning("Control tick exceeded %.1f ms; actual rate is below requested rate", period * 1000)
                    overrun_reported = True
                # Never send a burst to catch up after an overrun.
                next_tick = time.monotonic()
    finally:
        if worker is not None:
            worker.close()
        recorder.event("stopped", ticks=tick, state=state)
    return state
