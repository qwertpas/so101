"""LeRobot calibration, with angles in the official new-calibration URDF frame."""

import json
from pathlib import Path

import numpy as np

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TICK = 2 * np.pi / 4095  # Matches LeRobot's DEGREES normalization.


class Calibration:
    def __init__(self, path: Path):
        self.path = path.resolve()
        self.data = json.loads(path.read_text())
        self.ids = [int(self.data[n]["id"]) for n in NAMES]
        if len(set(self.ids)) != 6 or any(i < 1 or i > 253 for i in self.ids):
            raise ValueError("Calibration must contain six unique servo IDs")
        self.low = np.array([self.data[n]["range_min"] for n in NAMES])
        self.high = np.array([self.data[n]["range_max"] for n in NAMES])
        if np.any(self.low < 0) or np.any(self.high > 4095) or np.any(self.high <= self.low):
            raise ValueError("Invalid calibrated encoder ranges")
        if any(self.data[n]["drive_mode"] != 0 for n in NAMES):
            raise ValueError("This SO101 model requires drive_mode=0")
        self.zero = (self.low + self.high) / 2
        self.zero[-1] = self.low[-1]  # Closed jaw is zero, not the midpoint.

    def angles(self, raw):
        return (np.asarray(raw) - self.zero) * TICK

    def ticks(self, angles):
        angles = np.asarray(angles)
        if not np.all(np.isfinite(angles)):
            raise ValueError("Joint targets must be finite")
        return np.clip(np.rint(angles / TICK + self.zero), self.low, self.high).astype(int)

    def check(self, rows):
        for name, row in zip(NAMES, rows, strict=True):
            saved = self.data[name]
            for key, register in (
                ("homing_offset", "Homing offset"),
                ("range_min", "Min position limit"),
                ("range_max", "Max position limit"),
            ):
                if saved[key] != row[register]:
                    raise ValueError(f"{name}: {register} does not match {self.path.name}")
            if row["Operating mode"] != 0 or row["Model number"] != 777:
                raise ValueError(f"{name}: expected STS3215 in position mode")


def find_calibration(path: str | None = None) -> Path:
    if path:
        return Path(path).expanduser()
    root = Path.home() / ".cache/huggingface/lerobot/calibration/robots"
    paths = sorted(root.glob("so*_follower/*.json"))
    if len(paths) != 1:
        raise ValueError(f"Found {len(paths)} follower calibrations; select one with --calibration PATH")
    return paths[0]
