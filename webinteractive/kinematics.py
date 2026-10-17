"""Bounded five-joint pose IK using the same URDF as the viewer."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, least_squares, minimize
from scipy.spatial.transform import Rotation

from calibration import NAMES, Calibration

MODEL = Path(__file__).parent / "model/so101.urdf"


def origin(element):
    pose = np.eye(4)
    node = element.find("origin")
    if node is not None:
        pose[:3, 3] = np.fromstring(node.get("xyz", "0 0 0"), sep=" ")
        pose[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(node.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return pose


@dataclass
class Solution:
    angles: np.ndarray
    position_error: float
    rotation_error: float
    reachable: bool


class Kinematics:
    def __init__(self, calibration: Calibration):
        joints = {j.get("name"): j for j in ET.parse(MODEL).iter("joint") if j.get("type")}
        self.origins = [origin(joints[name]) for name in NAMES[:5]]
        self.axes = [np.fromstring(joints[name].find("axis").get("xyz"), sep=" ") for name in NAMES[:5]]
        self.tool = origin(joints["gripper_frame_joint"])
        # Measured mechanical endpoints are authoritative for this calibrated arm.
        # Generic URDF limits would clip real resting poses (e.g. shoulder lift).
        self.low = calibration.angles(calibration.low)
        self.high = calibration.angles(calibration.high)

    def pose(self, angles):
        pose = np.eye(4)
        for q, base, axis in zip(angles[:5], self.origins, self.axes, strict=True):
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
            pose = pose @ base @ turn
        return pose @ self.tool

    def rotation_axes(self, angles):
        """World axes of tool tilt and twist at the given joint pose."""
        original = self.pose(angles)[:3, :3]
        axes = []
        for index in (3, 4):
            moved = np.array(angles, dtype=float)
            moved[index] += 1e-5
            rotated = self.pose(moved)[:3, :3]
            axis = Rotation.from_matrix(rotated @ original.T).as_rotvec() / 1e-5
            axes.append(axis / np.linalg.norm(axis))
        return axes

    def solve(self, target, seed, match_rotation=True):
        seed = np.clip(np.asarray(seed), self.low, self.high)
        if not np.all(np.isfinite(target)):
            raise ValueError("Target pose must be finite")

        def residual(q):
            pose = self.pose(q)
            position = pose[:3, 3] - target[:3, 3]
            rotation = Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()
            return np.r_[position, rotation * (0.025 if match_rotation else 0), (q - seed[:5]) * 0.0001]

        result = least_squares(
            residual,
            seed[:5],
            bounds=(self.low[:5], self.high[:5]),
            max_nfev=45,
            ftol=1e-5,
            xtol=1e-5,
            gtol=1e-7,
        )
        angles = np.r_[result.x, seed[5]]
        pose = self.pose(angles)
        pe = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
        re = float(np.linalg.norm(Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()))
        return Solution(angles, pe, re, bool(pe < 0.005 and (not match_rotation or re < np.deg2rad(15))))

    def solve_with_tilt(self, position, tilt, twist, seed):
        """Keep tool position fixed while setting total bend and wrist twist."""
        position = np.asarray(position)
        seed = np.clip(np.asarray(seed), self.low, self.high)
        if not np.all(np.isfinite(position)) or not np.isfinite(tilt) or not np.isfinite(twist):
            raise ValueError("Target must be finite")

        def angles(x):
            return np.array([x[0], x[1], x[2], tilt - x[1] - x[2], twist, seed[5]])

        def error(x):
            distance = (self.pose(angles(x))[:3, 3] - position) * 1000
            return float(distance @ distance + 1e-5 * np.sum((x - seed[:3]) ** 2))

        total_low = max(self.low[1] + self.low[2], tilt - self.high[3])
        total_high = min(self.high[1] + self.high[2], tilt - self.low[3])
        if total_low > total_high or not self.low[4] <= twist <= self.high[4]:
            raise ValueError("Tilt or twist is outside calibrated joint limits")

        def feasible_start(start):
            total = np.clip(start[1] + start[2], total_low, total_high)
            shoulder = np.clip(
                start[1], max(self.low[1], total - self.high[2]), min(self.high[1], total - self.low[2])
            )
            return np.array([np.clip(start[0], self.low[0], self.high[0]), shoulder, total - shoulder])

        bend = LinearConstraint([[0, 1, 1]], tilt - self.high[3], tilt - self.low[3])
        starts = (feasible_start(seed[:3]), feasible_start([seed[0], -seed[1], -seed[2]]))
        best = None
        for start in starts:
            result = minimize(
                error,
                start,
                method="SLSQP",
                bounds=Bounds(self.low[:3], self.high[:3]),
                constraints=[bend],
                options={"ftol": 1e-9, "maxiter": 70},
            )
            q = angles(result.x)
            if np.any(q < self.low - 1e-6) or np.any(q > self.high + 1e-6):
                continue
            distance = float(np.linalg.norm(self.pose(q)[:3, 3] - position))
            if best is None or distance < best.position_error:
                best = Solution(q, distance, 0.0, distance < 0.005)
            if distance < 1e-4:
                break
        if best is None:
            q = angles(starts[0])
            best = Solution(q, float(np.linalg.norm(self.pose(q)[:3, 3] - position)), 0.0, False)
        return best

    def solve_position_first(self, target, seed):
        """Reach the target point before reducing any orientation error."""
        position = self.solve(target, seed, match_rotation=False)
        if not position.reachable:
            return position

        def residual(q):
            pose = self.pose(q)
            distance = pose[:3, 3] - target[:3, 3]
            rotation = Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()
            return np.r_[distance, rotation * 0.0005, (q - position.angles[:5]) * 0.000001]

        result = least_squares(
            residual,
            position.angles[:5],
            bounds=(self.low[:5], self.high[:5]),
            max_nfev=70,
        )
        angles = np.r_[result.x, seed[5]]
        pose = self.pose(angles)
        pe = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
        re = float(np.linalg.norm(Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()))
        if pe >= 0.005:
            return position
        return Solution(angles, pe, re, True)
