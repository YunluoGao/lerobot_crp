"""
SO101 leader -> CRP GP geometry.

Units:
- FK: meters + radians
- CRP GP: millimeters + degrees
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np

try:
    from scipy.spatial.transform import Rotation as ScipyRotation
except Exception:  # pragma: no cover - optional runtime dependency guard
    ScipyRotation = None

from lerobot.robots.crp_arm_dual.crp_ui import GRIPPER_UI_OPEN

WRIST_ROLL_ACTION_KEY = "wrist_roll.pos"
WRIST_FLEX_ACTION_KEY = "wrist_flex.pos"
WRIST_FLEX_SCIPY_AXIS = "x"

SO101_DEFAULT_URDF_REL = Path("third_party") / "SO101" / "so101_new_calib.urdf"

SO101_ARM_MOTOR_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

DEFAULT_Z_FLOOR_MM = 40.0
DEFAULT_Z_SCALE = 2.0


def so101_gripper_pos_to_crp_ui50(gripper_0_100: float) -> int:
    """SO101 gripper 0-100 (0 closed) -> CRP UI50 0-255 (0 closed, 255 open)."""
    clamped = max(0.0, min(100.0, float(gripper_0_100)))
    return int(round(clamped / 100.0 * 255.0))


def gp_position_step(
    current_pose: Sequence[float],
    target_pose: Sequence[float],
    step_mm: float,
    step_z_mm: float | None = None,
) -> list[float]:
    """
    One step toward target_pose.
    Default: 3D Euclidean cap ``step_mm``.
    Optional: per-axis XY cap + separate Z cap via ``step_z_mm``.
    """
    if len(current_pose) != 6 or len(target_pose) != 6:
        raise ValueError("poses must be length 6.")
    if step_mm < 0:
        raise ValueError("step_mm must be non-negative.")

    cx, cy, cz = (float(current_pose[i]) for i in range(3))
    tx, ty, tz = (float(target_pose[i]) for i in range(3))

    def _step_axis(current: float, target: float, step_cap: float) -> float:
        delta = target - current
        if step_cap <= 0 or abs(delta) <= step_cap:
            return target
        return current + math.copysign(step_cap, delta)

    use_aniso = step_z_mm is not None and step_z_mm > 0 and step_z_mm != step_mm
    if use_aniso:
        xyz = [
            round(_step_axis(cx, tx, step_mm if step_mm > 0 else 0.0), 6),
            round(_step_axis(cy, ty, step_mm if step_mm > 0 else 0.0), 6),
            round(_step_axis(cz, tz, step_z_mm), 6),
        ]
        if xyz == [tx, ty, tz]:
            return [float(v) for v in target_pose]
        return xyz + [float(target_pose[3]), float(target_pose[4]), float(target_pose[5])]

    dx, dy, dz = tx - cx, ty - cy, tz - cz
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= step_mm or dist == 0.0:
        return [float(v) for v in target_pose]

    scale = step_mm / dist
    xyz = [
        round(cx + dx * scale, 6),
        round(cy + dy * scale, 6),
        round(cz + dz * scale, 6),
    ]
    return xyz + [float(target_pose[3]), float(target_pose[4]), float(target_pose[5])]


class TrajectoryProcessor:
    """Back-compat alias for ``gp_position_step``."""

    def trajectory_differential(
        self,
        current_pose: Sequence[float],
        target_pose: Sequence[float],
        step_length: float = 0.1,
    ) -> list[float]:
        return gp_position_step(current_pose, target_pose, step_length)


def pose_to_crp_gp(pose6: list[float]) -> list[float]:
    x, y, z, roll, pitch, yaw = (float(v) for v in pose6)
    return [
        round(x * 1000.0, 6),
        round(y * 1000.0, 6),
        round(z * 1000.0, 6),
        round(float(np.degrees(roll)), 6),
        round(float(np.degrees(pitch)), 6),
        round(float(np.degrees(yaw)), 6),
    ]


def resolve_so101_urdf_path(urdf_path: str | None = None) -> str:
    if urdf_path:
        resolved = Path(urdf_path).expanduser().resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"Can not find URDF: {resolved}")
        return str(resolved)

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / SO101_DEFAULT_URDF_REL
        if candidate.is_file():
            return str(candidate.resolve())

    raise FileNotFoundError(f"Can not find {SO101_DEFAULT_URDF_REL}")


def leader_action_to_joint_degrees(
    action: dict[str, float],
    motor_names: list[str] | tuple[str, ...] = SO101_ARM_MOTOR_NAMES,
) -> np.ndarray:
    missing = [f"{m}.pos" for m in motor_names if f"{m}.pos" not in action]
    if missing:
        raise KeyError(f"Lack of joint keys: {missing}")
    return np.array([float(action[f"{m}.pos"]) for m in motor_names], dtype=float)


def _joint_deg(action: dict[str, float], key: str) -> float:
    if key not in action:
        raise KeyError(f"Lack of joint key: {key}")
    return float(action[key])


def get_wrist_roll_deg(action: dict[str, float]) -> float:
    return _joint_deg(action, WRIST_ROLL_ACTION_KEY)


def get_wrist_flex_deg(action: dict[str, float]) -> float:
    return _joint_deg(action, WRIST_FLEX_ACTION_KEY)


def _rpy_deg_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    if ScipyRotation is None:
        raise ImportError("scipy is required for CRP kinematics")
    return ScipyRotation.from_euler(
        "xyz",
        [float(roll_deg), float(pitch_deg), float(yaw_deg)],
        degrees=True,
    ).as_matrix()


def _matrix_to_rpy_deg(rm: np.ndarray) -> tuple[float, float, float]:
    if ScipyRotation is None:
        raise ImportError("scipy is required for CRP kinematics")
    roll, pitch, yaw = ScipyRotation.from_matrix(rm).as_euler("xyz", degrees=True)
    return float(roll), float(pitch), float(yaw)


def _matrix_to_rpy_rad(rm: np.ndarray) -> tuple[float, float, float]:
    if ScipyRotation is None:
        raise ImportError("scipy is required for CRP kinematics")
    roll, pitch, yaw = ScipyRotation.from_matrix(rm).as_euler("xyz", degrees=False)
    return float(roll), float(pitch), float(yaw)


def homogeneous_to_pose6(t: np.ndarray) -> list[float]:
    x, y, z = float(t[0, 3]), float(t[1, 3]), float(t[2, 3])
    roll, pitch, yaw = _matrix_to_rpy_rad(t[:3, :3])
    return [x, y, z, roll, pitch, yaw]


def get_endpose2Crp_urdf(
    action: dict[str, float],
    kinematics: "RobotKinematics",
    *,
    motor_names: list[str] | tuple[str, ...] = SO101_ARM_MOTOR_NAMES,
) -> list[float]:
    joint_deg = leader_action_to_joint_degrees(action, motor_names)
    t = kinematics.forward_kinematics(joint_deg)
    return pose_to_crp_gp(homogeneous_to_pose6(t))


@dataclass
class CrpGPAlignState:
    offset_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    fk_z_ref_mm: float | None = None
    z_floor_mm: float = DEFAULT_Z_FLOOR_MM
    z_scale: float = DEFAULT_Z_SCALE
    z_floor_clamp: bool = True
    locked_rpy: list[float] | None = None
    last_gp: list[float] | None = None
    wrist_roll_ref_deg: float | None = None
    last_wrist_roll_delta_deg: float | None = None
    wrist_flex_ref_deg: float | None = None
    last_wrist_flex_delta_deg: float | None = None
    sign_roll: float = 1.0
    sign_flex: float = 1.0

    def align_from_tcp(self, tcp_crp: list[float], fk_seed: list[float]) -> None:
        self.offset_xyz = [
            float(tcp_crp[0]) - float(fk_seed[0]),
            float(tcp_crp[1]) - float(fk_seed[1]),
            float(tcp_crp[2]) - float(fk_seed[2]),
        ]
        self.fk_z_ref_mm = float(fk_seed[2])
        self.locked_rpy = [float(tcp_crp[3]), float(tcp_crp[4]), float(tcp_crp[5])]

    def align_from_fk_fallback(self, fk_gp: list[float]) -> None:
        if len(fk_gp) != 6:
            raise ValueError("fk_gp must be length 6")
        self.offset_xyz = [0.0, 0.0, 0.0]
        self.fk_z_ref_mm = float(fk_gp[2])
        self.locked_rpy = [float(fk_gp[3]), float(fk_gp[4]), float(fk_gp[5])]

    def set_wrist_references(self, wrist_roll_deg: float, wrist_flex_deg: float) -> None:
        self.wrist_roll_ref_deg = float(wrist_roll_deg)
        self.wrist_flex_ref_deg = float(wrist_flex_deg)
        # Episode realign reuses CrpGPAlignState; stale deltas would clip the first GP tick.
        self.last_wrist_roll_delta_deg = None
        self.last_wrist_flex_delta_deg = None

    @staticmethod
    def _clip_wrist_delta_deg(
        delta_deg: float,
        last_delta_deg: float | None,
        max_step_deg: float | None,
    ) -> float:
        if max_step_deg is not None and max_step_deg > 0 and last_delta_deg is not None:
            return float(
                np.clip(
                    delta_deg,
                    last_delta_deg - max_step_deg,
                    last_delta_deg + max_step_deg,
                )
            )
        return delta_deg

    def apply_with_wrist_joints(
        self,
        fk_gp: list[float],
        wrist_roll_deg: float,
        wrist_flex_deg: float,
        *,
        max_step_roll_deg: float | None = None,
        max_step_flex_deg: float | None = None,
    ) -> list[float]:
        if self.locked_rpy is None:
            raise RuntimeError("locked_rpy is required; call align_from_tcp at startup")
        if self.fk_z_ref_mm is None:
            raise RuntimeError("fk_z_ref_mm is required; call align_from_tcp/align_from_fk_fallback")
        if self.wrist_roll_ref_deg is None or self.wrist_flex_ref_deg is None:
            raise RuntimeError("wrist references not set; call set_wrist_references at startup")

        x = float(fk_gp[0]) + float(self.offset_xyz[0])
        y = float(fk_gp[1]) + float(self.offset_xyz[1])
        fk_z = float(fk_gp[2])
        z = fk_z + float(self.offset_xyz[2]) + (float(self.z_scale) - 1.0) * (
            fk_z - float(self.fk_z_ref_mm)
        )
        if self.z_floor_clamp:
            z = max(z, float(self.z_floor_mm))

        r_ref = _rpy_deg_to_matrix(*self.locked_rpy[:3])

        delta_roll = float(self.sign_roll) * (float(wrist_roll_deg) - self.wrist_roll_ref_deg)
        delta_roll = self._clip_wrist_delta_deg(
            delta_roll, self.last_wrist_roll_delta_deg, max_step_roll_deg
        )
        self.last_wrist_roll_delta_deg = delta_roll
        if ScipyRotation is None:
            raise ImportError("scipy is required for CRP kinematics")
        r_roll = ScipyRotation.from_euler("z", np.deg2rad(delta_roll)).as_matrix()

        delta_flex = float(self.sign_flex) * (float(wrist_flex_deg) - self.wrist_flex_ref_deg)
        delta_flex = self._clip_wrist_delta_deg(
            delta_flex, self.last_wrist_flex_delta_deg, max_step_flex_deg
        )
        self.last_wrist_flex_delta_deg = delta_flex
        if ScipyRotation is None:
            raise ImportError("scipy is required for CRP kinematics")
        r_flex = ScipyRotation.from_euler(WRIST_FLEX_SCIPY_AXIS, np.deg2rad(delta_flex)).as_matrix()

        r_cmd = r_ref @ r_roll @ r_flex
        roll_deg, pitch_deg, yaw_deg = _matrix_to_rpy_deg(r_cmd)
        return [
            round(x, 6),
            round(y, 6),
            round(z, 6),
            round(roll_deg, 6),
            round(pitch_deg, 6),
            round(yaw_deg, 6),
        ]

    def step_toward(
        self,
        target: list[float],
        step_mm: float,
        *,
        step_z_mm: float | None = None,
    ) -> list[float]:
        pos = step_mm if step_mm > 0 else 0.0
        sz = step_z_mm if step_z_mm is not None and step_z_mm > 0 else None
        if pos <= 0 and sz is None:
            out = [float(x) for x in target]
        else:
            current = self.last_gp if self.last_gp is not None else target
            out = gp_position_step(current, target, pos, sz)
        self.last_gp = [float(x) for x in out]
        return out


def split_bimanual_so101_action(action: dict[str, float]) -> tuple[dict[str, float], dict[str, float]]:
    left = {k.removeprefix("left_"): v for k, v in action.items() if k.startswith("left_")}
    right = {k.removeprefix("right_"): v for k, v in action.items() if k.startswith("right_")}
    return left, right


__all__ = [
    "DEFAULT_Z_FLOOR_MM",
    "DEFAULT_Z_SCALE",
    "GRIPPER_UI_OPEN",
    "SO101_ARM_MOTOR_NAMES",
    "WRIST_FLEX_ACTION_KEY",
    "WRIST_ROLL_ACTION_KEY",
    "CrpGPAlignState",
    "TrajectoryProcessor",
    "get_endpose2Crp_urdf",
    "get_wrist_flex_deg",
    "get_wrist_roll_deg",
    "gp_position_step",
    "resolve_so101_urdf_path",
    "so101_gripper_pos_to_crp_ui50",
    "split_bimanual_so101_action",
]

