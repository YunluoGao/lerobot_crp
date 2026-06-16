"""Dual CRP joint + ui50 execution (GJ path or GP joint-nudge fallback)."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal

from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.scripts.crp_gp.config import ACTION_UI_KEYS, JOINT_POS_KEYS, TeleoperateDualCRPConfig
from lerobot.scripts.crp_gp.loop import (
    GP_INIT_POINT_COUNT,
    ensure_gj_replay_ready,
    lock_gps_to_current_tcp,
    wait_before_gp_align,
)
from lerobot.tools import TrajectoryProcessor

logger = logging.getLogger(__name__)

ArmLabel = Literal["left", "right"]


class RightJointBackend(str, Enum):
    """How to command the right arm (ip2)."""

    AUTO = "auto"
    SET_GJS_SECOND = "set_gjs_second"
    GP_NUDGE = "gp_nudge"


# Heuristic: joint delta (deg) -> user TCP delta (mm / deg) per joint index j1..j6.
_JOINT_TO_GP_SCALE: tuple[float, float, float, float, float, float] = (
    3.5,
    3.5,
    2.5,
    1.0,
    1.0,
    1.0,
)


@dataclass
class JointSnapshot:
    left: list[float]
    right: list[float]

    def arm_joints(self, arm: ArmLabel) -> list[float]:
        return self.left if arm == "left" else self.right


@dataclass
class CrpDualGjExecutorConfig:
    gj_register_left: int = 10
    gj_register_right: int = 20
    gj_trajectory_group_size: int = 5
    max_joint_delta_deg: float | None = None
    right_joint_backend: RightJointBackend = RightJointBackend.AUTO
    gp_tcp_step_mm: float = 12.0
    gp_tcp_step_deg: float = 6.0
    gp_send_fps: int = 50
    gp_align_delay_s: float = 0.0
    gp_joint_deadband_deg: float = 0.5


@dataclass
class GpTickArm:
    """Minimal handle for ``CRPArmDual.send_gp_tick`` (same shape as teleop loop)."""

    gp_index: int
    command_gp: list[float]


def read_dual_joint_snapshot(robot: CRPArmDual) -> JointSnapshot:
    raw = robot.read_crp_joints()
    left = [float(raw[f"left_j{i}.pos"]) for i in range(1, 7)]
    right = [float(raw[f"right_j{i}.pos"]) for i in range(1, 7)]
    return JointSnapshot(left=left, right=right)


def action_array_to_dict(names: list[str], values: Any) -> dict[str, float]:
    if hasattr(values, "tolist"):
        values = values.tolist()
    return {name: float(values[i]) for i, name in enumerate(names)}


def extract_arm_joints(action: dict[str, float], arm: ArmLabel) -> list[float]:
    prefix = arm
    return [float(action[f"{prefix}_j{i}.pos"]) for i in range(1, 7)]


def clamp_joints_toward(
    current: list[float], target: list[float], max_delta_deg: float
) -> list[float]:
    if max_delta_deg <= 0:
        return [float(v) for v in target]
    out: list[float] = []
    for c, t in zip(current, target, strict=True):
        delta = float(t) - float(c)
        if abs(delta) > max_delta_deg:
            delta = max_delta_deg if delta > 0 else -max_delta_deg
        out.append(float(c) + delta)
    return out


def joint_tracking_error(current: list[float], target: list[float]) -> float:
    return max(abs(float(c) - float(t)) for c, t in zip(current, target, strict=True))


def joints_to_gp_from_baseline(
    baseline_joints: list[float],
    baseline_tcp: list[float],
    command_joints: list[float],
    scales: tuple[float, float, float, float, float, float] = _JOINT_TO_GP_SCALE,
) -> list[float]:
    """Legacy baseline-relative map (kept for tests). Prefer ``joints_to_gp_incremental``."""
    gp = [float(v) for v in baseline_tcp]
    for i, (j0, jt, s) in enumerate(
        zip(baseline_joints, command_joints, scales, strict=True)
    ):
        gp[i] += (float(jt) - float(j0)) * float(s)
    return gp


def joints_to_gp_incremental(
    current_tcp: list[float],
    current_joints: list[float],
    target_joints: list[float],
    scales: tuple[float, float, float, float, float, float] = _JOINT_TO_GP_SCALE,
) -> list[float]:
    """Nudge GP from the current TCP using only this frame's joint delta (approximate IK)."""
    gp = [float(v) for v in current_tcp]
    for i, (cj, tj, s) in enumerate(zip(current_joints, target_joints, scales, strict=True)):
        gp[i] += (float(tj) - float(cj)) * float(s)
    return gp


def clamp_gp_toward(
    current_gp: list[float],
    target_gp: list[float],
    *,
    step_mm: float,
    step_deg: float,
) -> list[float]:
    out: list[float] = []
    for i, (c, t) in enumerate(zip(current_gp, target_gp, strict=True)):
        step = step_deg if i >= 3 else step_mm
        delta = float(t) - float(c)
        if abs(delta) > step:
            delta = step if delta > 0 else -step
        out.append(float(c) + delta)
    return out


def _gj_command_matrix(joints: list[float], group_size: int) -> list[list[float]]:
    """Same layout as gj-probe / crp_arm connect init (``init_matrix`` × group_size)."""
    traj = TrajectoryProcessor(max_points=1, max_joints=group_size)
    return traj.init_matrix(joints, group_size)


def _tele_cfg_for_gj_lock(robot: CRPArmDual, cfg: CrpDualGjExecutorConfig) -> TeleoperateDualCRPConfig:
    tele_cfg = TeleoperateDualCRPConfig(robot=robot.config)
    tele_cfg.left.gp_index = cfg.gj_register_left
    tele_cfg.right.gp_index = cfg.gj_register_right
    return tele_cfg


# Brief gap between left/right set_GJs on one SDK session (controller processes one GJ bank at a time).
_GJ_DUAL_ARM_GAP_S = 0.02


def _resolve_right_backend(robot: CRPArmDual, cfg: CrpDualGjExecutorConfig) -> RightJointBackend:
    pref = cfg.right_joint_backend
    if pref is not RightJointBackend.AUTO:
        return pref

    crp = robot.crp_arm_robot
    if hasattr(crp, "set_GJs_second"):
        logger.info("Right arm: set_GJs_second (ip2).")
        return RightJointBackend.SET_GJS_SECOND
    raise RuntimeError(
        "set_GJs_second missing on CrpRobotPy; rebuild from ~/python_C++/CrpRobotPy "
        "and deploy CrpRobotPy.so to third_party/CrpRobotPy/"
    )


def _read_arm_tcp(robot: CRPArmDual, arm: ArmLabel) -> list[float]:
    if arm == "left":
        return [float(v) for v in robot.read_end_pose_first()]
    return [float(v) for v in robot.read_end_pose_second()]


def _is_dual_gp_mode(backend: RightJointBackend) -> bool:
    return backend is RightJointBackend.GP_NUDGE


def _send_left_gjs(
    robot: CRPArmDual, gj_register: int, matrix: list[list[float]]
) -> tuple[bool, str]:
    ok = robot.set_GJs_first(gj_register, matrix)
    return ok, f"set_GJs(left,GJ{gj_register})"


def _send_right_joints(
    robot: CRPArmDual,
    gj_register: int,
    matrix: list[list[float]],
    backend: RightJointBackend,
) -> tuple[bool, str]:
    if backend is not RightJointBackend.SET_GJS_SECOND:
        raise RuntimeError(f"unsupported right joint backend: {backend.value}")
    ok = robot.set_GJs_second(gj_register, matrix)
    return ok, f"set_GJs_second(GJ{gj_register})"


class CrpDualGjExecutor:
    """Send dataset/policy actions: 12 joint angles + left/right ui50."""

    def __init__(self, robot: CRPArmDual, cfg: CrpDualGjExecutorConfig) -> None:
        self._robot = robot
        self._cfg = cfg
        self._lock = threading.RLock()
        self._right_backend = _resolve_right_backend(robot, cfg)
        self._dual_gp = _is_dual_gp_mode(self._right_backend)
        self._gj_initialized = False
        self._lag_log_interval_s = 1.0
        self._last_lag_log_t = 0.0
        self._baseline_left_j: list[float] = []
        self._baseline_right_j: list[float] = []
        self._baseline_left_tcp: list[float] = []
        self._baseline_right_tcp: list[float] = []
        self._last_left_gp: list[float] = []
        self._last_right_gp: list[float] = []
        self._gp_left: GpTickArm | None = None
        self._gp_right: GpTickArm | None = None
        self._gp_log_interval_s = 2.0
        self._last_gp_log_t = 0.0

        if self._dual_gp:
            logger.info(
                "Dual-arm GP replay @ %d Hz (same joint→GP heuristic for left and right).",
                self._cfg.gp_send_fps,
            )
        elif self._right_backend is RightJointBackend.SET_GJS_SECOND:
            logger.info("Right arm joint path: set_GJs_second (ip2).")
        else:
            logger.info("Right arm joint backend: %s", self._right_backend.value)

    def uses_dual_gp_stream(self) -> bool:
        return self._dual_gp

    def _plan_joint_commands(
        self, action: dict[str, float], snapshot: JointSnapshot
    ) -> tuple[list[float], list[float], dict[str, float]]:
        left_tgt = extract_arm_joints(action, "left")
        right_tgt = extract_arm_joints(action, "right")
        max_delta = self._cfg.max_joint_delta_deg
        if max_delta is not None:
            left_cmd = clamp_joints_toward(snapshot.left, left_tgt, max_delta)
            right_cmd = clamp_joints_toward(snapshot.right, right_tgt, max_delta)
        else:
            left_cmd = left_tgt
            right_cmd = right_tgt
        sent: dict[str, float] = {}
        for i, val in enumerate(left_cmd, start=1):
            sent[f"left_j{i}.pos"] = val
        for i, val in enumerate(right_cmd, start=1):
            sent[f"right_j{i}.pos"] = val
        return left_cmd, right_cmd, sent

    def _send_dual_gj(self, left_cmd: list[float], right_cmd: list[float]) -> tuple[bool, bool, str]:
        gs = self._cfg.gj_trajectory_group_size
        left_mat = _gj_command_matrix(left_cmd, gs)
        right_mat = _gj_command_matrix(right_cmd, gs)
        with self._lock:
            ok_left, left_path = _send_left_gjs(self._robot, self._cfg.gj_register_left, left_mat)
            time.sleep(_GJ_DUAL_ARM_GAP_S)
            ok_right, right_path = _send_right_joints(
                self._robot,
                self._cfg.gj_register_right,
                right_mat,
                self._right_backend,
            )
        return ok_left, ok_right, right_path

    def _update_arm_gp(
        self,
        arm: ArmLabel,
        current_j: list[float],
        command_j: list[float],
        last_gp: list[float],
        gp_tick: GpTickArm,
    ) -> list[float]:
        if joint_tracking_error(current_j, command_j) < self._cfg.gp_joint_deadband_deg:
            return last_gp
        try:
            current_tcp = _read_arm_tcp(self._robot, arm)
        except Exception as exc:
            logger.warning("[%s] TCP read failed, using last GP: %s", arm, exc)
            current_tcp = list(last_gp)
        new_gp = self._command_gp_for_arm(current_tcp, current_j, last_gp, command_j)
        gp_tick.command_gp = new_gp
        return new_gp

    def prepare_gp_replay_session(self, tele_cfg: TeleoperateDualCRPConfig) -> None:
        """Symmetric dual-arm GP: lock + init burst on both arms."""
        if not self._dual_gp:
            raise RuntimeError("prepare_gp_replay_session() requires gp_nudge backend")

        if self._cfg.gp_align_delay_s > 0:
            wait_before_gp_align(delay_s=self._cfg.gp_align_delay_s)
        lock_gps_to_current_tcp(self._robot, tele_cfg)

        snap = read_dual_joint_snapshot(self._robot)
        self._baseline_left_j = list(snap.left)
        self._baseline_right_j = list(snap.right)
        self._baseline_left_tcp = _read_arm_tcp(self._robot, "left")
        self._baseline_right_tcp = _read_arm_tcp(self._robot, "right")
        self._last_left_gp = list(self._baseline_left_tcp)
        self._last_right_gp = list(self._baseline_right_tcp)

        self._gp_left = GpTickArm(
            gp_index=self._cfg.gj_register_left,
            command_gp=list(self._baseline_left_tcp),
        )
        self._gp_right = GpTickArm(
            gp_index=self._cfg.gj_register_right,
            command_gp=list(self._baseline_right_tcp),
        )

        logger.info(
            "GP baseline: left TCP (%.0f, %.0f, %.0f) right TCP (%.0f, %.0f, %.0f)",
            self._baseline_left_tcp[0],
            self._baseline_left_tcp[1],
            self._baseline_left_tcp[2],
            self._baseline_right_tcp[0],
            self._baseline_right_tcp[1],
            self._baseline_right_tcp[2],
        )
        with self._lock:
            ok_left, ok_right = self._robot.send_dual_gp_init(
                left_gp=self._last_left_gp,
                right_gp=self._last_right_gp,
                gp_index_left=self._cfg.gj_register_left,
                gp_index_right=self._cfg.gj_register_right,
                point_count=GP_INIT_POINT_COUNT,
            )
        logger.info(
            "GP init burst (%d points/arm): left=%s right=%s",
            GP_INIT_POINT_COUNT,
            ok_left,
            ok_right,
        )
        self._gj_initialized = True

    def init_gj_registers(self) -> None:
        if self._dual_gp:
            raise RuntimeError("Call prepare_gp_replay_session() for gp_nudge, not init_gj_registers()")
        ensure_gj_replay_ready(self._robot, delay_s=self._cfg.gp_align_delay_s)
        tele_cfg = _tele_cfg_for_gj_lock(self._robot, self._cfg)
        lock_gps_to_current_tcp(self._robot, tele_cfg)
        logger.info("GP locked to current TCP before GJ replay (required for GJ motion on CRP).")
        snap = read_dual_joint_snapshot(self._robot)
        logger.info(
            "GJ init: left=GJ%s right=GJ%s (%s)",
            self._cfg.gj_register_left,
            self._cfg.gj_register_right,
            self._right_backend.value,
        )
        ok_left, ok_right, path = self._send_dual_gj(snap.left, snap.right)
        logger.info("GJ init left: ok=%s | right: ok=%s path=%s", ok_left, ok_right, path)
        self._gj_initialized = True

    def _command_gp_for_arm(
        self,
        current_tcp: list[float],
        current_j: list[float],
        last_gp: list[float],
        command_j: list[float],
    ) -> list[float]:
        target = joints_to_gp_incremental(current_tcp, current_j, command_j)
        return clamp_gp_toward(
            last_gp,
            target,
            step_mm=self._cfg.gp_tcp_step_mm,
            step_deg=self._cfg.gp_tcp_step_deg,
        )

    def update_dataset_frame(self, action: dict[str, float]) -> dict[str, float]:
        """Apply one dataset/policy row (joint targets + ui50)."""
        if not self._gj_initialized:
            raise RuntimeError("Session not initialized")

        snapshot = read_dual_joint_snapshot(self._robot)
        left_cmd, right_cmd, sent = self._plan_joint_commands(action, snapshot)

        if self._dual_gp:
            assert self._gp_left is not None and self._gp_right is not None
            self._last_left_gp = self._update_arm_gp(
                "left", snapshot.left, left_cmd, self._last_left_gp, self._gp_left
            )
            self._last_right_gp = self._update_arm_gp(
                "right", snapshot.right, right_cmd, self._last_right_gp, self._gp_right
            )

        for key in ACTION_UI_KEYS:
            if key in action:
                ui_val = int(max(0, min(255, round(float(action[key])))))
                if key == "left_ui50":
                    self._robot.set_gripper_open_first(ui_val)
                else:
                    self._robot.set_gripper_open_second(ui_val)
                sent[key] = float(ui_val)

        return sent

    def send_gp_tick_once(self) -> tuple[bool | None, bool | None]:
        if not self._dual_gp or self._gp_left is None or self._gp_right is None:
            return None, None
        with self._lock:
            ok_left, ok_right = self._robot.send_gp_tick(self._gp_left, self._gp_right)
        now = time.perf_counter()
        if now - self._last_gp_log_t >= self._gp_log_interval_s:
            self._last_gp_log_t = now
            lg = self._gp_left.command_gp
            rg = self._gp_right.command_gp
            logger.info(
                "GP tick ok L=%s R=%s | cmd L=(%.0f,%.0f,%.0f) R=(%.0f,%.0f,%.0f)",
                ok_left,
                ok_right,
                lg[0],
                lg[1],
                lg[2],
                rg[0],
                rg[1],
                rg[2],
            )
        if ok_left is False or ok_right is False:
            logger.warning("GP tick rejected (left=%s right=%s)", ok_left, ok_right)
        return ok_left, ok_right

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self._gj_initialized:
            raise RuntimeError("Call init_gj_registers() or prepare_gp_replay_session() first.")
        if self._dual_gp:
            return self.update_dataset_frame(action)

        snapshot = read_dual_joint_snapshot(self._robot)
        left_cmd, right_cmd, sent = self._plan_joint_commands(action, snapshot)
        ok_left, ok_right, path = self._send_dual_gj(left_cmd, right_cmd)
        if not ok_left or not ok_right:
            logger.warning(
                "Joint write rejected (left=%s right=%s path=%s)",
                ok_left,
                ok_right,
                path,
            )

        for key in ACTION_UI_KEYS:
            if key in action:
                ui_val = int(max(0, min(255, round(float(action[key])))))
                if key == "left_ui50":
                    self._robot.set_gripper_open_first(ui_val)
                else:
                    self._robot.set_gripper_open_second(ui_val)
                sent[key] = float(ui_val)

        after = read_dual_joint_snapshot(self._robot)
        left_cmd_err = joint_tracking_error(after.left, left_cmd)
        right_cmd_err = joint_tracking_error(after.right, right_cmd)
        if not self._dual_gp and (left_cmd_err > 5.0 or right_cmd_err > 5.0):
            now = time.perf_counter()
            if now - self._last_lag_log_t >= self._lag_log_interval_s:
                self._last_lag_log_t = now
                logger.warning(
                    "Large joint tracking error vs command: left=%.2fdeg right=%.2fdeg",
                    left_cmd_err,
                    right_cmd_err,
                )

        return sent

    def hold_current_pose(self) -> dict[str, float]:
        if not self._gj_initialized:
            if self._dual_gp:
                raise RuntimeError("Call prepare_gp_replay_session() before hold_current_pose()")
            self.init_gj_registers()
            return {}
        snap = read_dual_joint_snapshot(self._robot)
        action = {f"left_j{i}.pos": snap.left[i - 1] for i in range(1, 7)}
        action.update({f"right_j{i}.pos": snap.right[i - 1] for i in range(1, 7)})
        return self.send_action(action)


def validate_dual_action_keys(names: list[str]) -> None:
    missing = [k for k in JOINT_POS_KEYS if k not in names]
    if missing:
        raise ValueError(f"Dataset action missing joint keys: {missing}")
    missing_ui = [k for k in ACTION_UI_KEYS if k not in names]
    if missing_ui:
        logger.warning("Dataset action missing ui50 keys (gripper may not move): %s", missing_ui)
