"""Select exactly the observation keys used by the saved joint-policy setup."""
import numpy as np
from .calibration import STATE_DIM


def build(state, frames, instruction, cfg):
    state = np.asarray(state, dtype=np.float32)
    if state.shape != (STATE_DIM,) or not np.all(np.isfinite(state)):
        raise ValueError("Observation requires 8 finite left-arm state values")
    schema = cfg["observation"]
    obs = {schema["state_key"]: state, schema["prompt_key"]: instruction}
    for stream, key in schema["camera_keys"].items():
        image = np.asarray(frames[stream])  # Missing cameras raise immediately.
        if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise ValueError(f"{stream} must be HWC uint8 RGB")
        # No resize or rectification: server preprocessing must match training.
        obs[key] = np.ascontiguousarray(image)
    return obs
