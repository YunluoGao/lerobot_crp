from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.crp_ui import GRIPPER_UI_OPEN
from lerobot.teleoperators import Teleoperator
from lerobot.utils.robot_utils import precise_sleep

from .config import ArmGPConfig, ArmTeleopState, DualGPTiming, TeleoperateDualCRPConfig
from .mappers.so101_urdf import (
    CrpGPAlignState,
    get_endpose2Crp_urdf,
    get_wrist_flex_deg,
    get_wrist_roll_deg,
    so101_gripper_pos_to_crp_ui50,
    split_bimanual_so101_action,
)
from .ui_probe import GripperUiProbeManager

GP_INIT_POINT_COUNT = 5
TARGET_FRAME_NAME = "gripper_frame_link"
GP_LOG_HEARTBEAT_S = 1.0
# Consecutive GP ticks with is_connected=False before stopping (filters SDK flaps).
CRP_DISCONNECT_STOP_AFTER = 3

logger = logging.getLogger(__name__)


def _interval_due(now: float, last: float, interval_s: float) -> bool:
    return interval_s <= 0.0 or (now - last) >= interval_s


def _read_tcp(robot: CRPArmDual, label: str) -> list[float]:
    if label == "left":
        return robot.read_end_pose_first()
    return robot.read_end_pose_second()


def _per_loop_step(step: float, cfg: TeleoperateDualCRPConfig) -> float:
    """Scale per-frame step so total change between GP sends equals ``step``."""
    if step <= 0:
        return 0.0
    if cfg.gp_send_fps <= 0 or cfg.fps <= 0:
        return step
    return step * float(cfg.gp_send_fps) / float(cfg.fps)


def _command_gp_for_arm(
    arm: ArmTeleopState,
    fk_gp: list[float],
    arm_action: dict[str, float],
    *,
    apply_step_cap: bool,
) -> list[float]:
    wrist_roll = get_wrist_roll_deg(arm_action)
    wrist_flex = get_wrist_flex_deg(arm_action)
    cmd = arm.gp_align.apply_with_wrist_joints(
        fk_gp,
        wrist_roll,
        wrist_flex,
        max_step_roll_deg=arm.max_step_roll_per_loop,
        max_step_flex_deg=arm.max_step_flex_per_loop,
    )
    if apply_step_cap and arm.pos_step_xy_per_loop > 0:
        cmd = arm.gp_align.step_toward(
            cmd,
            arm.pos_step_xy_per_loop,
            step_z_mm=arm.step_z_mm,
        )
    return cmd


def _make_arm_states(
    cfg: TeleoperateDualCRPConfig, robot: CRPArmDual
) -> tuple[ArmTeleopState, ArmTeleopState]:
    specs: tuple[tuple[str, ArmGPConfig, Callable[[int, list], bool], str], ...] = (
        ("left", cfg.left, robot.send_GPs_first, "first"),
        ("right", cfg.right, robot.send_GPs_second, "second"),
    )
    arms: list[ArmTeleopState] = []
    limit_pos = cfg.gp_position_step_mm > 0
    for label, arm_cfg, send_gps_fn, ui_target in specs:
        pos_xy = _per_loop_step(cfg.gp_position_step_mm, cfg) if limit_pos else 0.0
        pos_z = (
            _per_loop_step(cfg.gp_position_step_mm * arm_cfg.z_scale, cfg)
            if limit_pos and arm_cfg.z_scale > 1.0
            else pos_xy
        )
        flex_deg = arm_cfg.wrist_flex_max_step_deg
        arms.append(
            ArmTeleopState(
                label=label,
                gp_align=CrpGPAlignState(
                    z_floor_mm=float(arm_cfg.z_floor_mm),
                    z_scale=float(arm_cfg.z_scale),
                    z_floor_clamp=True,
                    sign_roll=float(arm_cfg.wrist_roll_sign),
                    sign_flex=float(arm_cfg.wrist_flex_sign),
                ),
                gp_index=arm_cfg.gp_index,
                max_step_roll_per_loop=(
                    _per_loop_step(arm_cfg.wrist_roll_max_step_deg, cfg)
                    if arm_cfg.wrist_roll_max_step_deg > 0
                    else None
                ),
                max_step_flex_per_loop=(
                    _per_loop_step(flex_deg, cfg) if flex_deg is not None and flex_deg > 0 else None
                ),
                pos_step_xy_per_loop=pos_xy,
                step_z_mm=pos_z if pos_z > 0 and pos_z != pos_xy else None,
                send_gps_fn=send_gps_fn,
                set_ui_fn=lambda idx, val, t=ui_target, r=robot: r._set_ui(t, idx, val),
            )
        )
    return arms[0], arms[1]


def _align_arms_at_start(
    robot: CRPArmDual,
    arms: tuple[ArmTeleopState, ArmTeleopState],
    fk_gp_seeds: tuple[list[float], list[float]],
) -> None:
    """Align both arms at startup from each controller's TCP."""
    for arm, fk_seed in zip(arms, fk_gp_seeds, strict=True):
        try:
            tcp_pose = _read_tcp(robot, arm.label)
        except Exception as exc:
            logger.warning("[%s] TCP read failed, FK fallback: %s", arm.label, exc)
            arm.gp_align.align_from_fk_fallback(fk_seed)
            continue

        arm.gp_align.align_from_tcp(tcp_pose, fk_seed)
        dx, dy, dz = arm.gp_align.offset_xyz
        dist = math.hypot(dx, dy, dz)
        if dist > 5.0:
            logger.info(
                "[%s] GP align offset vs leader FK: Δxyz=(%.0f, %.0f, %.0f) mm (%.0f mm)",
                arm.label,
                dx,
                dy,
                dz,
                dist,
            )


def wait_before_gp_align(*, delay_s: float) -> None:
    """Count down after servo enable before GP align/init (time to press teach-pendant green)."""
    if delay_s <= 0:
        return
    total = int(math.ceil(delay_s))
    logger.info(
        "Servo enabled; GP alignment in %ds (press green start on teach pendants if needed).",
        total,
    )
    deadline = time.perf_counter() + delay_s
    last_logged: int | None = total
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            break
        secs = max(0, int(math.ceil(remaining)))
        if secs != last_logged:
            logger.info("GP alignment in %ds...", secs)
            last_logged = secs
        time.sleep(min(0.2, remaining))
    logger.info("Starting GP alignment.")


def startup_dual_gp_teleop(
    teleop: Teleoperator,
    robot: CRPArmDual,
    kinematics: RobotKinematics,
    cfg: TeleoperateDualCRPConfig,
    arms: tuple[ArmTeleopState, ArmTeleopState] | None = None,
) -> tuple[tuple[ArmTeleopState, ArmTeleopState], tuple[list[float], list[float]]]:
    """TCP/FK align + GP init burst (reuse ``arms`` when re-aligning between episodes)."""
    arm_actions = split_bimanual_so101_action(teleop.get_action())
    fk_seeds = tuple(get_endpose2Crp_urdf(a, kinematics=kinematics) for a in arm_actions)
    if arms is None:
        arms = _make_arm_states(cfg, robot)
    _align_arms_at_start(robot, arms, fk_seeds)
    init_gps: list[list[float]] = []
    for arm, act, fk in zip(arms, arm_actions, fk_seeds, strict=True):
        arm.gp_align.set_wrist_references(get_wrist_roll_deg(act), get_wrist_flex_deg(act))
        try:
            init_gp = _read_tcp(robot, arm.label)
            logger.info(
                "[%s] GP init from current TCP (%.0f, %.0f, %.0f) — no leader jump",
                arm.label,
                init_gp[0],
                init_gp[1],
                init_gp[2],
            )
        except Exception as exc:
            logger.warning("[%s] TCP read failed for GP init, using leader FK: %s", arm.label, exc)
            init_gp = _command_gp_for_arm(arm, fk, act, apply_step_cap=False)
        arm.gp_align.last_gp = list(init_gp)
        init_gps.append(init_gp)
        arm.last_sent = list(init_gp)
        arm.command_gp = list(init_gp)
        if not arm.send_gps_fn(arm.gp_index, [list(init_gp) for _ in range(GP_INIT_POINT_COUNT)]):
            logger.warning("[GP init] set_GPs (%s) rejected by controller", arm.label)
    return arms, (init_gps[0], init_gps[1])


def lock_gps_to_current_tcp(robot: CRPArmDual, cfg: TeleoperateDualCRPConfig) -> None:
    """Write current TCP to GP registers (hold pose; avoids chasing stale targets on servo/exit)."""
    if not robot.is_connected:
        return

    specs: tuple[tuple[str, int, Callable[[int, list], bool], Callable[[], list[float]]], ...] = (
        ("left", cfg.left.gp_index, robot.send_GPs_first, robot.read_end_pose_first),
        ("right", cfg.right.gp_index, robot.send_GPs_second, robot.read_end_pose_second),
    )
    for label, gp_index, send_fn, read_tcp_fn in specs:
        try:
            gp6 = [float(v) for v in read_tcp_fn()]
        except Exception as exc:
            logger.warning("[%s] GP lock: TCP read failed (%s); skipping", label, exc)
            continue
        points = [gp6 for _ in range(GP_INIT_POINT_COUNT)]
        if not send_fn(gp_index, points):
            logger.warning("[%s] GP lock: set_GPs rejected by controller", label)
            continue
        logger.info(
            "[%s] GP locked to current TCP (%.0f, %.0f, %.0f)",
            label,
            gp6[0],
            gp6[1],
            gp6[2],
        )


def prepare_dual_gp_session(
    robot: CRPArmDual,
    cfg: TeleoperateDualCRPConfig,
    ui_probe: GripperUiProbeManager | None = None,
) -> None:
    """Once per record/teleop session: UI probe, servo enable, gripper UI setup (no GP align)."""
    if ui_probe is not None and ui_probe.enabled:
        ui_probe.start_dual(cfg.robot.ip1, cfg.robot.ip2)
        ui_probe.wait_for_subprocess_ready()

    if robot.config.defer_servo_power_on:
        robot.ensure_servo_power_on()

    robot.setup_gripper_ui()
    lock_gps_to_current_tcp(robot, cfg)


def align_dual_gp_for_episode(
    teleop: Teleoperator,
    robot: CRPArmDual,
    kinematics: RobotKinematics,
    cfg: TeleoperateDualCRPConfig,
    arms: tuple[ArmTeleopState, ArmTeleopState] | None = None,
    *,
    align_delay_s: float | None = None,
) -> tuple[tuple[ArmTeleopState, ArmTeleopState], tuple[list[float], list[float]]]:
    """Per-episode GP realign over the existing SDK session (no probe reconnect / no servo cycle).

    First episode: ``align_delay_s`` defaults to ``cfg.gp_align_delay_s`` (time to green-start).
    Later episodes: default ``0`` (immediate realign only).
    """
    if align_delay_s is None:
        align_delay_s = cfg.gp_align_delay_s if arms is None else 0.0
    if align_delay_s > 0:
        wait_before_gp_align(delay_s=align_delay_s)
    elif arms is not None:
        logger.info("Episode GP realign (servo stays on, no align delay).")
    arms, init_gps = startup_dual_gp_teleop(teleop, robot, kinematics, cfg, arms=arms)
    return arms, init_gps


def _send_gps_if_due(
    robot: CRPArmDual,
    arms: tuple[ArmTeleopState, ArmTeleopState],
    now: float,
    last_send_time: float,
    interval_s: float,
    *,
    disconnect_streak: list[int],
) -> tuple[float, bool, str | None]:
    """Returns ``(updated_last_send_time, should_stop, stop_reason)``."""
    if not _interval_due(now, last_send_time, interval_s):
        return last_send_time, False, None
    left_arm, right_arm = arms
    send_result = robot.send_gp_tick(left_arm, right_arm)
    if send_result == (None, None):
        disconnect_streak[0] += 1
        if disconnect_streak[0] >= CRP_DISCONNECT_STOP_AFTER:
            logger.error(
                "CRP disconnected for %d GP ticks; stopping teleop loop.",
                disconnect_streak[0],
            )
            return last_send_time, True, "crp_disconnected"
        logger.warning(
            "CRP is_connected=False (streak %d/%d); skipping GP send.",
            disconnect_streak[0],
            CRP_DISCONNECT_STOP_AFTER,
        )
        return last_send_time, False, None
    disconnect_streak[0] = 0
    ok_left, ok_right = send_result
    for arm, ok in ((left_arm, ok_left), (right_arm, ok_right)):
        if ok is None:
            continue
        if ok:
            arm.last_sent = [float(x) for x in arm.command_gp]
        else:
            arm.gp_align.last_gp = arm.last_sent[:]
            gp = arm.command_gp
            logger.warning(
                "%s GP rejected (controller / IK); pose=%.1f,%.1f,%.1f",
                arm.label,
                gp[0],
                gp[1],
                gp[2],
            )
    return now, False, None


def _gripper_send_interval_s(cfg: TeleoperateDualCRPConfig) -> float:
    if cfg.gripper_send_fps <= 0:
        return 0.0
    return 1.0 / float(cfg.gripper_send_fps)


def _gripper_pos_changed(arm: ArmTeleopState, cfg: TeleoperateDualCRPConfig) -> bool:
    if arm.gripper_pos is None:
        return False
    if arm.last_gripper_pos is None:
        return True
    return abs(float(arm.gripper_pos) - float(arm.last_gripper_pos)) >= cfg.gripper_min_pos_delta


def _send_gripper_ui_if_due(
    cfg: TeleoperateDualCRPConfig,
    arms: tuple[ArmTeleopState, ArmTeleopState],
    now: float,
    last_send_time: float,
) -> float:
    gr_iv = _gripper_send_interval_s(cfg)
    if gr_iv > 0.0 and not _interval_due(now, last_send_time, gr_iv):
        return last_send_time

    sent = False
    for arm in arms:
        if not _gripper_pos_changed(arm, cfg):
            continue
        ui50 = so101_gripper_pos_to_crp_ui50(arm.gripper_pos)  # type: ignore[arg-type]
        arm.set_ui_fn(GRIPPER_UI_OPEN, ui50)
        arm.last_ui50 = ui50
        arm.last_gripper_pos = float(arm.gripper_pos)
        sent = True

    if gr_iv <= 0.0 or sent:
        return now
    return last_send_time


class TeleopStop(Exception):
    """Leave the teleop loop with a logged reason."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


def advance_dual_gp_teleop(
    teleop: Teleoperator,
    robot: CRPArmDual,
    kinematics: RobotKinematics,
    cfg: TeleoperateDualCRPConfig,
    arms: tuple[ArmTeleopState, ArmTeleopState],
    timing: DualGPTiming,
    *,
    limit_gp_step: bool,
    disconnect_streak: list[int],
) -> None:
    """One scheduler tick: leader FK -> GP/gripper commands."""
    for arm, act in zip(arms, split_bimanual_so101_action(teleop.get_action()), strict=True):
        arm.gripper_pos = act.get("gripper.pos")
        arm.command_gp = _command_gp_for_arm(
            arm,
            get_endpose2Crp_urdf(act, kinematics=kinematics),
            act,
            apply_step_cap=limit_gp_step,
        )

    now = time.perf_counter()
    gp_iv = 1.0 / cfg.gp_send_fps if cfg.gp_send_fps > 0 else 0.0
    timing.last_gp, stop, stop_reason = _send_gps_if_due(
        robot, arms, now, timing.last_gp, gp_iv, disconnect_streak=disconnect_streak
    )
    if stop:
        raise TeleopStop(stop_reason or "crp_disconnected")

    timing.last_gr = _send_gripper_ui_if_due(cfg, arms, now, timing.last_gr)


def _log_teleop_ready(
    cfg: TeleoperateDualCRPConfig,
    init_gps: tuple[list[float], list[float]],
) -> None:
    left_init, right_init = init_gps
    logger.info(
        "Ready: %s / %s | GP#%d/#%d @%dHz loop=%dHz step=%.0fmm | "
        "L_xyz=(%.0f,%.0f,%.0f) R_xyz=(%.0f,%.0f,%.0f)",
        cfg.robot.ip1,
        cfg.robot.ip2,
        cfg.left.gp_index,
        cfg.right.gp_index,
        cfg.gp_send_fps,
        cfg.fps,
        cfg.gp_position_step_mm,
        left_init[0],
        left_init[1],
        left_init[2],
        right_init[0],
        right_init[1],
        right_init[2],
    )


def prepare_dual_gp_teleop_startup(
    teleop: Teleoperator,
    robot: CRPArmDual,
    kinematics: RobotKinematics,
    cfg: TeleoperateDualCRPConfig,
    ui_probe: GripperUiProbeManager | None = None,
) -> tuple[tuple[ArmTeleopState, ArmTeleopState], tuple[list[float], list[float]]]:
    """Single-session teleop startup: probe + servo + delay + GP align/init."""
    prepare_dual_gp_session(robot, cfg, ui_probe=ui_probe)
    return align_dual_gp_for_episode(
        teleop, robot, kinematics, cfg, arms=None, align_delay_s=cfg.gp_align_delay_s
    )


def teleop_loop_crp_dual(
    teleop: Teleoperator,
    robot: CRPArmDual,
    kinematics: RobotKinematics,
    cfg: TeleoperateDualCRPConfig,
    ui_probe: GripperUiProbeManager | None = None,
) -> None:
    """Main loop. Configuration from ``cfg``."""
    limit_gp_step = cfg.gp_position_step_mm > 0
    arms, init_gps = prepare_dual_gp_teleop_startup(
        teleop, robot, kinematics, cfg, ui_probe=ui_probe
    )
    _log_teleop_ready(cfg, init_gps)

    timing = DualGPTiming()
    loop_i = 0
    last_hb = time.perf_counter()
    disconnect_streak = [0]
    stop_reason = "unknown"

    try:
        while True:
            t0 = time.perf_counter()
            try:
                advance_dual_gp_teleop(
                    teleop,
                    robot,
                    kinematics,
                    cfg,
                    arms,
                    timing,
                    limit_gp_step=limit_gp_step,
                    disconnect_streak=disconnect_streak,
                )
            except TeleopStop as exc:
                stop_reason = exc.reason
                break
            except Exception as exc:
                stop_reason = f"{type(exc).__name__}: {exc}"
                logger.exception("Teleop loop error at loop=%d", loop_i + 1)
                break
            loop_i += 1
            precise_sleep(max(1 / cfg.fps - (time.perf_counter() - t0), 0.0))

            if GP_LOG_HEARTBEAT_S > 0:
                now = time.perf_counter()
                if (now - last_hb) >= GP_LOG_HEARTBEAT_S:
                    la, ra = arms
                    if ui_probe is not None and ui_probe.enabled:
                        left_line = ui_probe.format_arm_heartbeat_line(
                            "L", "left", tuple(la.command_gp[:3]), ui50_cmd_fallback=la.last_ui50
                        )
                        right_line = ui_probe.format_arm_heartbeat_line(
                            "R", "right", tuple(ra.command_gp[:3]), ui50_cmd_fallback=ra.last_ui50
                        )
                    else:
                        left_line = (
                            "\tL_GP=[%.0f,%.0f,%.0f] L_UI50=%s, L_POSIT=?, L_SPEED=?, L_TORQUE=?"
                            % (*la.command_gp[:3], la.last_ui50)
                        )
                        right_line = (
                            "\tR_GP=[%.0f,%.0f,%.0f] R_UI50=%s, R_POSIT=?, R_SPEED=?, R_TORQUE=?"
                            % (*ra.command_gp[:3], ra.last_ui50)
                        )
                    logger.info("loop=%d\n%s\n%s", loop_i, left_line, right_line)
                    last_hb = now
    finally:
        logger.info("Teleop loop ended: reason=%s loops=%d", stop_reason, loop_i)

