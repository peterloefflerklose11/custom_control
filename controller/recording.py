"""Small JSONL audit trail: measured poses, goals, chunks, and configuration."""
import json
import shutil
from datetime import datetime, timezone
from uuid import uuid4
from .settings import local_path


class Recorder:
    def __init__(self, cfg):
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid4().hex[:8]
        self.folder = local_path("logs/" + name)
        self.folder.mkdir(parents=True)
        (self.folder / "runtime.json").write_text(json.dumps(cfg, indent=2) + "\n")
        for key in ("calibration_file", "robot_file", "rig_map_file"):
            source = local_path(cfg[key])
            shutil.copyfile(source, self.folder / (key + ".json"))
        self.file = (self.folder / "events.jsonl").open("x", buffering=1)

    def event(self, kind, **values):
        self.file.write(json.dumps({"time_utc": datetime.now(timezone.utc).isoformat(),
                                    "kind": kind, **values}) + "\n")

    def close(self):
        self.file.close()
