"""Capture overhead and left wrist RGB views; recover from missing frames.

The rig's overhead camera is stereo. Its left half is the top-view image.
The left wrist frame is sent whole; the right wrist camera is not opened.
"""
import threading
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)


class CameraUnavailableError(RuntimeError):
    """No valid fresh camera observation yet; wait rather than end the session."""


class Cameras:
    def __init__(self, cfg):
        self.cfg = cfg
        self.caps, self.threads, self.latest = {}, [], {}
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.errors = {}

    def _open_capture(self, name):
        import cv2
        spec = self.cfg["cameras"][name]
        cap = cv2.VideoCapture(spec["device"], cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise CameraUnavailableError(f"Cannot open camera {name}")
            # MJPEG first: uncompressed streams can saturate the USB bus.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, spec["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, spec["height"])
            cap.set(cv2.CAP_PROP_FPS, self.cfg["hz"])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
            return cap
        except BaseException:
            cap.release()
            raise

    def open(self):
        try:
            for name in self.cfg["cameras"]:
                thread = threading.Thread(target=self._pump, args=(name,), daemon=True)
                self.threads.append(thread)
                thread.start()
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                try:
                    self.frames()
                    return
                except CameraUnavailableError:
                    pass
                time.sleep(0.02)
            # Keep the control loop/keyboard available even if a camera is
            # still missing. Capture threads keep trying in the background.
            logger.warning("Cameras not ready yet; waiting for valid frames in the control loop")
        except BaseException:
            self.close()
            raise

    def _pump(self, name):
        import cv2
        spec = self.cfg["cameras"][name]
        retry = self.cfg.get("camera_retry_interval_s", 1.0)
        while not self.stop.is_set():
            cap = None
            try:
                cap = self._open_capture(name)
                with self.lock:
                    self.caps[name] = cap
                self._capture_frames(name, cap, spec, cv2)
            except Exception as exc:
                with self.lock:
                    first_failure = name not in self.errors
                    self.errors[name] = str(exc)
                    self.latest.pop(name, None)  # Never reuse a failed stream's pixels.
                if first_failure:
                    logger.warning("Camera %s unavailable; will retry capture: %s", name, exc)
            finally:
                # Only this thread owns read/release, including on reconnect.
                if cap is not None:
                    cap.release()
                with self.lock:
                    self.caps.pop(name, None)
            self.stop.wait(retry)

    def _capture_frames(self, name, cap, spec, cv2):
        misses = 0
        while not self.stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                # An isolated miss gets another read; three consecutive misses
                # close/reopen the device. No failed stream image is reused.
                misses += 1
                with self.lock:
                    self.latest.pop(name, None)
                if misses >= 3:
                    raise CameraUnavailableError(f"Camera {name} stopped delivering frames")
                self.stop.wait(0.01)
                continue
            misses = 0
            if frame.shape != (spec["height"], spec["width"], 3):
                raise CameraUnavailableError(f"Camera {name}: unexpected resolution {frame.shape}")
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self.lock:
                recovered = self.errors.pop(name, None) is not None
                self.latest[name] = (time.monotonic(), rgb)
            if recovered:
                logger.info("Camera %s capture recovered", name)

    def frames(self):
        now = time.monotonic()
        with self.lock:
            latest = dict(self.latest)
            errors = dict(self.errors)
        out = {}
        for name, spec in self.cfg["cameras"].items():
            if name not in latest or now - latest[name][0] > self.cfg["camera_max_age_s"]:
                detail = errors.get(name, "missing or stale")
                raise CameraUnavailableError(f"Camera {name} unavailable: {detail}")
            rgb = latest[name][1]
            if name == "overhead":
                half = rgb.shape[1] // 2
                out["overhead_left"] = rgb[:, :half]
            else:
                out[name] = rgb
        return out

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=1)
        # Threads release their own handles. A kernel-blocked read is a daemon
        # so the terminal/control loop can still exit without a read/release race.


class MockCameras:
    """Synthetic RGB data, sized exactly like the configured rig streams."""
    def __init__(self, cfg):
        self.cfg = cfg

    def open(self):
        pass

    def frames(self):
        out = {}
        for name, spec in self.cfg["cameras"].items():
            image = np.zeros((spec["height"], spec["width"], 3), dtype=np.uint8)
            if name == "overhead":
                half = spec["width"] // 2
                out["overhead_left"] = image[:, :half]
            else:
                out[name] = image
        return out

    def close(self):
        pass
