"""Phase-0 probe: dual CRP joint direct control (set_GJs / movej) for deploy path A.

Loads a dataset action frame (or applies a small joint step), sends joint commands,
and reports tracking error plus cross-arm coupling. Intended for safe lab smoke tests
before ``lerobot-crp-replay-dual``.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Literal

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.crp_ui import GRIPPER_UI_OPEN
from lerobot.scripts.crp_gp.config import (
    JOINT_POS_KEYS,
    TeleoperateDualCRPConfig,
    _default_robot_for_teleop,
)
from lerobot.scripts.crp_gp.loop import ensure_gj_replay_ready, lock_gps_to_current_tcp
from lerobot.tools import TrajectoryProcessor
from lerobot.utils.constants import ACTION, HF_LEROBOT_HOME

logger = logging.getLogger(__name__)

ArmLabel = Literal["left", "right"]
ProbeMode = Literal["hold", "step", "dataset"]


class JointWriteMethod(str, Enum):
    SET_GJS = "set_gjs"
    MOVEJ = "movej"


@dataclass
class GjProbeConfig:
    """Configuration for the dual-arm GJ / movej phase-0 probe."""

    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_teleop)
    dataset_repo_id: str = "user/20260615_gyl_1"
    dataset_root: str | Path | None = None
    frame_index: int = 0
    mode: ProbeMode = "step"
    step_arm: ArmLabel = "left"
    step_joint: int = 1
    step_delta_deg: float = 1.0
    max_joint_delta_deg: float = 5.0
    arms: Literal["left", "right", "both"] = "both"
    methods: tuple[JointWriteMethod, ...] = (
        JointWriteMethod.SET_GJS,
        JointWriteMethod.MOVEJ,
    )
    gj_register_left: int = 10
    gj_register_right: int = 20
    gj_trajectory_group_size: int = 5
    settle_s: float = 1.0
    dry_run: bool = False
    test_ui50: bool = True
    ui50_pulse_delta: int = 15
    speed_ratio: int | None = 20
    gp_lock_on_exit: bool = True
    # Seconds to wait before GJ writes (press green START on both teach pendants).
    gp_align_delay_s: float = 5.0


@dataclass
class JointSnapshot:
    left: list[float]
    right: list[float]

    def arm_joints(self, arm: ArmLabel) -> list[float]:
        return self.left if arm == "left" else self.right


@dataclass
class ProbeAttemptResult:
    arm: ArmLabel
    method: JointWriteMethod
    gj_register: int | None
    command_joints: list[float]
    joints_before: JointSnapshot
    joints_after: JointSnapshot
    max_error_deg: float
    per_joint_error_deg: list[float]
    cross_arm_max_delta_deg: float
    physical_motion_deg: float
    sdk_call_ok: bool
    message: str

    @property
    def passed(self) -> bool:
        if not self.sdk_call_ok:
            return False
        # Require the commanded arm to actually move, not just "error == step size".
        moved = self.physical_motion_deg >= 0.3
        tracked = self.max_error_deg <= 3.0
        return moved and tracked


@dataclass
class GjProbeReport:
    sdk_methods: dict[str, bool]
    mode: ProbeMode
    target_left: list[float] | None
    target_right: list[float] | None
    attempts: list[ProbeAttemptResult]
    ui50_ok: bool | None
    ui50_message: str = ""

    def summary_lines(self) -> list[str]:
        lines = [
            "=== CRP dual GJ probe summary ===",
            f"SDK joint-write methods: {self.sdk_methods}",
            f"Mode: {self.mode}",
        ]
        if self.target_left is not None:
            lines.append(f"Target left (deg):  {[round(v, 2) for v in self.target_left]}")
        if self.target_right is not None:
            lines.append(f"Target right (deg): {[round(v, 2) for v in self.target_right]}")
        for attempt in self.attempts:
            status = "PASS" if attempt.passed else "FAIL"
            lines.append(
                f"[{status}] {attempt.arm} {attempt.method.value} "
                f"(GJ={attempt.gj_register}) motion={attempt.physical_motion_deg:.2f}deg "
                f"max_err={attempt.max_error_deg:.2f}deg "
                f"cross={attempt.cross_arm_max_delta_deg:.2f}deg — {attempt.message}"
            )
        if self.ui50_ok is not None:
            ui_status = "PASS" if self.ui50_ok else "FAIL"
            lines.append(f"[{ui_status}] ui50 pulse — {self.ui50_message}")
        return lines


def read_dual_joint_snapshot(robot: CRPArmDual) -> JointSnapshot:
    try:
        raw = robot.read_crp_joints()
    except Exception as exc:
        raise RuntimeError(
            f"read_crp_joints failed: {exc}. "
            "Stop both teach-pendant GJ programs and rerun probe."
        ) from exc
    left = [float(raw[f"left_j{i}.pos"]) for i in range(1, 7)]
    right = [float(raw[f"right_j{i}.pos"]) for i in range(1, 7)]
    return JointSnapshot(left=left, right=right)


def extract_arm_joints_from_action(action: dict[str, float], arm: ArmLabel) -> list[float]:
    prefix = arm
    missing = [f"{prefix}_j{i}.pos" for i in range(1, 7) if f"{prefix}_j{i}.pos" not in action]
    if missing:
        raise KeyError(f"Action missing keys for {arm}: {missing}")
    return [float(action[f"{prefix}_j{i}.pos"]) for i in range(1, 7)]


def clamp_joints_toward(
    current: list[float], target: list[float], max_delta_deg: float
) -> list[float]:
    if max_delta_deg <= 0:
        return [float(v) for v in current]
    out: list[float] = []
    for c, t in zip(current, target, strict=True):
        delta = float(t) - float(c)
        if abs(delta) > max_delta_deg:
            delta = max_delta_deg if delta > 0 else -max_delta_deg
        out.append(float(c) + delta)
    return out


def joint_tracking_error(current: list[float], target: list[float]) -> tuple[float, list[float]]:
    errors = [abs(float(c) - float(t)) for c, t in zip(current, target, strict=True)]
    return max(errors), errors


def physical_arm_motion(before: JointSnapshot, after: JointSnapshot, arm: ArmLabel) -> float:
    b = before.arm_joints(arm)
    a = after.arm_joints(arm)
    return max(abs(x - y) for x, y in zip(b, a, strict=True))


def cross_arm_delta(before: JointSnapshot, after: JointSnapshot, commanded_arm: ArmLabel) -> float:
    other: ArmLabel = "right" if commanded_arm == "left" else "left"
    b = before.arm_joints(other)
    a = after.arm_joints(other)
    return max(abs(x - y) for x, y in zip(b, a, strict=True))


def build_step_target(
    snapshot: JointSnapshot,
    *,
    arm: ArmLabel,
    joint_index: int,
    delta_deg: float,
) -> tuple[list[float] | None, list[float] | None]:
    if joint_index < 1 or joint_index > 6:
        raise ValueError("step_joint must be in 1..6")
    left = snapshot.left[:]
    right = snapshot.right[:]
    idx = joint_index - 1
    if arm == "left":
        left[idx] = float(left[idx]) + float(delta_deg)
        return left, None
    right[idx] = float(right[idx]) + float(delta_deg)
    return None, right


def load_dataset_action_frame(
    repo_id: str,
    frame_index: int,
    *,
    root: Path | None = None,
) -> dict[str, float]:
    dataset_root = root if root is not None else HF_LEROBOT_HOME
    dataset = LeRobotDataset(repo_id, root=dataset_root)
    if frame_index < 0 or frame_index >= len(dataset):
        raise IndexError(f"frame_index {frame_index} out of range for dataset len={len(dataset)}")
    row = dataset[frame_index]
    names = dataset.meta.features[ACTION]["names"]
    values = row[ACTION]
    if hasattr(values, "tolist"):
        values = values.tolist()
    return {name: float(values[i]) for i, name in enumerate(names)}


def inspect_sdk_joint_methods(crp: Any) -> dict[str, bool]:
    names = ("set_GJs", "movej", "set_GJs_second")
    return {name: callable(getattr(crp, name, None)) for name in names}


def _gj_matrix_for_target(joints: list[float], group_size: int) -> list[list[float]]:
    traj = TrajectoryProcessor(max_points=1, max_joints=group_size)
    return traj.init_matrix(joints, group_size)


def _resolve_gj_register(arm: ArmLabel, cfg: GjProbeConfig) -> int:
    return cfg.gj_register_left if arm == "left" else cfg.gj_register_right


def _send_set_gjs_primary(
    crp: Any, gj_register: int, matrix: list[list[float]]
) -> tuple[bool, str]:
    if not hasattr(crp, "set_GJs"):
        return False, "set_GJs missing on CrpRobotPy"
    ok = bool(crp.set_GJs(gj_register, matrix))
    return (
        ok,
        f"set_GJs(GJ{gj_register})" if ok else f"set_GJs(GJ{gj_register}) rejected",
    )


def _send_set_gjs_second(
    crp: Any, gj_register: int, matrix: list[list[float]]
) -> tuple[bool, str]:
    if not hasattr(crp, "set_GJs_second"):
        return False, "set_GJs_second missing — rebuild CrpRobotPy from ~/python_C++/CrpRobotPy"
    try:
        ok = bool(crp.set_GJs_second(gj_register, matrix))
    except Exception as exc:
        return False, f"set_GJs_second exception: {exc}"
    return ok, "set_GJs_second" if ok else "set_GJs_second rejected"


def _send_set_gjs(
    crp: Any,
    arm: ArmLabel,
    gj_register: int,
    joints: list[float],
    group_size: int,
) -> tuple[bool, str]:
    matrix = _gj_matrix_for_target(joints, group_size)
    if arm == "left":
        return _send_set_gjs_primary(crp, gj_register, matrix)
    return _send_set_gjs_second(crp, gj_register, matrix)


def _send_movej(
    crp: Any, arm: ArmLabel, joints: list[float], *, settle_s: float = 1.0
) -> tuple[bool, str]:
    if arm == "left":
        if not hasattr(crp, "movej"):
            return False, "movej missing on CrpRobotPy"
        restored_mode = False
        try:
            from CrpRobotPy import RobotMode  # noqa: PLC0415

            if hasattr(crp, "switch_work_mode"):
                crp.switch_work_mode(RobotMode.Manual)
                restored_mode = True
            crp.movej(joints)
            time.sleep(max(0.0, float(settle_s)))
        except Exception as exc:
            return False, f"movej(primary) failed: {exc}"
        finally:
            if restored_mode and hasattr(crp, "switch_work_mode"):
                crp.switch_work_mode(RobotMode.Auto)
        return True, "movej(primary, Manual→Auto)"

    if hasattr(crp, "movej"):
        crp.movej(joints)
        return True, "movej(primary) fallback for right arm (may move left arm only)"
    return False, "no movej path for right arm"


def _send_joint_command(
    crp: Any,
    arm: ArmLabel,
    method: JointWriteMethod,
    joints: list[float],
    cfg: GjProbeConfig,
) -> tuple[bool, str, int | None]:
    gj_register = _resolve_gj_register(arm, cfg)
    if method is JointWriteMethod.SET_GJS:
        ok, msg = _send_set_gjs(crp, arm, gj_register, joints, cfg.gj_trajectory_group_size)
        return ok, msg, gj_register
    ok, msg = _send_movej(crp, arm, joints, settle_s=cfg.settle_s)
    return ok, msg, None


def _effective_step_arm(cfg: GjProbeConfig) -> ArmLabel:
    """``--arms=left|right`` overrides ``--step_arm`` in step mode."""
    if cfg.arms in ("left", "right"):
        return cfg.arms
    return cfg.step_arm


def resolve_probe_targets(
    cfg: GjProbeConfig,
    snapshot: JointSnapshot,
    action_frame: dict[str, float] | None,
) -> tuple[list[float] | None, list[float] | None]:
    if cfg.mode == "hold":
        return None, None

    if cfg.mode == "step":
        return build_step_target(
            snapshot,
            arm=_effective_step_arm(cfg),
            joint_index=cfg.step_joint,
            delta_deg=cfg.step_delta_deg,
        )

    if action_frame is None:
        raise ValueError("dataset mode requires a loaded action frame")
    left_tgt = extract_arm_joints_from_action(action_frame, "left")
    right_tgt = extract_arm_joints_from_action(action_frame, "right")
    left_cmd = clamp_joints_toward(snapshot.left, left_tgt, cfg.max_joint_delta_deg)
    right_cmd = clamp_joints_toward(snapshot.right, right_tgt, cfg.max_joint_delta_deg)
    return left_cmd, right_cmd


def _arms_to_test(cfg: GjProbeConfig) -> list[ArmLabel]:
    if cfg.arms == "both":
        return ["left", "right"]
    return [cfg.arms]


def run_joint_write_attempt(
    robot: CRPArmDual,
    *,
    arm: ArmLabel,
    method: JointWriteMethod,
    command_joints: list[float],
    cfg: GjProbeConfig,
) -> ProbeAttemptResult:
    crp = robot.crp_arm_robot
    before = read_dual_joint_snapshot(robot)

    if cfg.dry_run:
        return ProbeAttemptResult(
            arm=arm,
            method=method,
            gj_register=_resolve_gj_register(arm, cfg) if method is JointWriteMethod.SET_GJS else None,
            command_joints=command_joints,
            joints_before=before,
            joints_after=before,
            max_error_deg=0.0,
            per_joint_error_deg=[0.0] * 6,
            cross_arm_max_delta_deg=0.0,
            physical_motion_deg=0.0,
            sdk_call_ok=True,
            message="dry-run (no SDK write)",
        )

    ok, msg, gj_register = _send_joint_command(crp, arm, method, command_joints, cfg)
    time.sleep(max(0.0, float(cfg.settle_s)))
    after = read_dual_joint_snapshot(robot)
    tracked = after.arm_joints(arm)
    max_err, per_joint = joint_tracking_error(tracked, command_joints)
    cross = cross_arm_delta(before, after, arm)
    motion = physical_arm_motion(before, after, arm)

    return ProbeAttemptResult(
        arm=arm,
        method=method,
        gj_register=gj_register,
        command_joints=command_joints,
        joints_before=before,
        joints_after=after,
        max_error_deg=max_err,
        per_joint_error_deg=per_joint,
        cross_arm_max_delta_deg=cross,
        physical_motion_deg=motion,
        sdk_call_ok=ok,
        message=msg,
    )


def _pulse_ui50(robot: CRPArmDual, *, delta: int, settle_s: float) -> tuple[bool, str]:
    try:
        left_before = robot.get_ui("first", GRIPPER_UI_OPEN)
        right_before = robot.get_ui("second", GRIPPER_UI_OPEN)
    except Exception as exc:
        return False, f"get_ui failed: {exc}"

    left_cmd = int(max(0, min(255, left_before + delta)))
    right_cmd = int(max(0, min(255, right_before + delta)))
    ok_l = robot.set_gripper_open_first(left_cmd)
    ok_r = robot.set_gripper_open_second(right_cmd)
    time.sleep(max(0.0, settle_s))
    try:
        left_after = robot.get_ui("first", GRIPPER_UI_OPEN)
        right_after = robot.get_ui("second", GRIPPER_UI_OPEN)
    except Exception as exc:
        return False, f"post-command get_ui failed: {exc}"

    moved = (left_after != left_before) or (right_after != right_before)
    ok = ok_l and ok_r and moved
    return ok, (
        f"left {left_before}->{left_after} right {right_before}->{right_after} "
        f"(set_ok L={ok_l} R={ok_r})"
    )


def run_gj_probe(cfg: GjProbeConfig, robot: CRPArmDual) -> GjProbeReport:
    crp = robot.crp_arm_robot
    sdk_methods = inspect_sdk_joint_methods(crp)
    logger.info("SDK joint-write surface: %s", sdk_methods)

    tele_cfg = TeleoperateDualCRPConfig(robot=cfg.robot)
    lock_gps_to_current_tcp(robot, tele_cfg)

    if cfg.speed_ratio is not None and hasattr(crp, "set_speed_ratio"):
        crp.set_speed_ratio(int(cfg.speed_ratio))
        logger.info("speed_ratio set to %s", cfg.speed_ratio)

    snapshot = read_dual_joint_snapshot(robot)
    logger.info("Initial left joints (deg):  %s", [round(v, 2) for v in snapshot.left])
    logger.info("Initial right joints (deg): %s", [round(v, 2) for v in snapshot.right])

    action_frame: dict[str, float] | None = None
    if cfg.mode == "dataset":
        root = Path(cfg.dataset_root) if cfg.dataset_root is not None else HF_LEROBOT_HOME
        action_frame = load_dataset_action_frame(
            cfg.dataset_repo_id, cfg.frame_index, root=root
        )
        logger.info(
            "Loaded dataset action %s frame %d (%d keys)",
            cfg.dataset_repo_id,
            cfg.frame_index,
            len(action_frame),
        )

    left_target, right_target = resolve_probe_targets(cfg, snapshot, action_frame)
    if left_target is not None:
        logger.info("Command target left (deg):  %s", [round(v, 2) for v in left_target])
    if right_target is not None:
        logger.info("Command target right (deg): %s", [round(v, 2) for v in right_target])

    if any(m is JointWriteMethod.SET_GJS for m in cfg.methods) and not cfg.dry_run:
        logger.info(
            "GJ probe sequence: (1) teach programs STOPPED now "
            "(2) PC seeds GJ10/GJ20 (3) press START during countdown "
            "(4) PC writes step target — do not STOP programs before step 4 finishes."
        )
        try:
            seed_left = cfg.arms in ("left", "both")
            seed_right = cfg.arms in ("right", "both")
            ok_l, ok_r = robot.seed_gj_registers_from_current_pose(
                seed_left=seed_left, seed_right=seed_right
            )
            if seed_right and not ok_r:
                logger.error(
                    "Right GJ20 seed failed (set_GJs_second). "
                    "Verify GJ20 on teach pendant and CrpRobotPy.so build."
                )
        except Exception as exc:
            logger.warning("GJ seed before probe failed: %s", exc)
        if not robot.is_connected:
            logger.error(
                "SDK session lost after GJ seed (Disconnected). "
                "STOP both teach programs, re-run probe from a fresh connect."
            )
            return GjProbeReport(
                sdk_methods=sdk_methods,
                mode=cfg.mode,
                target_left=left_target,
                target_right=right_target,
                attempts=[],
                ui50_ok=None,
                ui50_message="skipped (session lost after GJ seed)",
            )
        ensure_gj_replay_ready(robot, delay_s=cfg.gp_align_delay_s)

    attempts: list[ProbeAttemptResult] = []
    for arm in _arms_to_test(cfg):
        command = left_target if arm == "left" else right_target
        if command is None:
            logger.info("Skipping %s arm (no target in this mode)", arm)
            continue
        for method in cfg.methods:
            label = f"{arm} {method.value}"
            logger.info("Attempt %s ...", label)
            result = run_joint_write_attempt(
                robot,
                arm=arm,
                method=method,
                command_joints=command,
                cfg=cfg,
            )
            attempts.append(result)
            logger.info(
                "%s: sdk_ok=%s motion=%.2fdeg max_err=%.2fdeg cross=%.2fdeg — %s",
                label,
                result.sdk_call_ok,
                result.physical_motion_deg,
                result.max_error_deg,
                result.cross_arm_max_delta_deg,
                result.message,
            )
            if result.passed:
                break

    ui50_ok: bool | None = None
    ui50_message = "skipped"
    if cfg.test_ui50 and not cfg.dry_run:
        ui50_ok, ui50_message = _pulse_ui50(robot, delta=cfg.ui50_pulse_delta, settle_s=cfg.settle_s)

    return GjProbeReport(
        sdk_methods=sdk_methods,
        mode=cfg.mode,
        target_left=left_target,
        target_right=right_target,
        attempts=attempts,
        ui50_ok=ui50_ok,
        ui50_message=ui50_message,
    )


def probe_exit_code(report: GjProbeReport) -> int:
    """0 when at least one attempt shows real arm motion + tracking."""
    if not report.attempts:
        return 1
    if any(a.passed for a in report.attempts):
        return 0
    return 1


def validate_action_keys(action: dict[str, float]) -> None:
    missing = [k for k in JOINT_POS_KEYS if k not in action]
    if missing:
        raise KeyError(f"Dataset action missing joint keys: {missing[:4]}...")
