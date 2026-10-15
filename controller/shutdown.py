"""Return an armed robot home before releasing its serial connection.

Mirrors the RCS finally-stage ordering, using this folder's gradual homing
instead of RCS's collision planner. Torque stays enabled after arriving.
"""
import logging
from .homing import run_home
from .keyboard import Keyboard

logger = logging.getLogger(__name__)


def finish_session(robot, cfg, calibration, recorder, *, mode, failed=False):
    """Return HOME / SKIPPED / CANCELLED / FAILED; never retry a stalled move."""
    reason = None
    if mode == "observe":
        reason = "read-only session"
    elif not cfg.get("return_home_on_exit", True):
        reason = "return_home_on_exit disabled"
    elif not robot.engaged:
        reason = "robot was never armed"
    elif failed or getattr(robot, "shutdown_home_blocked", False):
        reason = "session had a sensor, tracking or hardware fault"
    if reason:
        logger.info("Skipping shutdown home: %s", reason)
        recorder.event("shutdown_home_skipped", reason=reason)
        return "SKIPPED"

    logger.info("Returning home before exit. Press q, Space or Ctrl-C to cancel this return move.")
    recorder.event("shutdown_home_started")
    try:
        with Keyboard() as keyboard:
            state = run_home(robot, cfg, calibration, keyboard, recorder,
                             mode=mode, steps=0, start_immediately=True, return_on_hold=True)
        result = "HOME" if state == "HOME" else "CANCELLED" if state == "STOPPED" else "FAILED"
    except KeyboardInterrupt:
        result = "CANCELLED"
        logger.warning("Shutdown home cancelled; last goal and torque retained")
    except Exception as exc:
        result = "FAILED"
        logger.warning("Shutdown home failed: %s; last goal and torque retained", exc)
    recorder.event("shutdown_home_finished", result=result)
    return result
