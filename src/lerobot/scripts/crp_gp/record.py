"""Dual CRP dataset recording on top of the GP teleop loop."""

from __future__ import annotations

import logging
import math
import select
import signal
import sys
import termios
import threading
import time
import tty
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING

from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.scripts.crp_gp.record_utils import build_record_observation, read_ui_for_dataset
from lerobot.scripts.crp_gp.config import (
    ArmTeleopState,
    DualGPTiming,
    JOINT_POS_KEYS,
    RecordDualCRPConfig,
    TeleoperateDualCRPConfig,
)
from lerobot.scripts.crp_gp.loop import (
    GP_LOG_HEARTBEAT_S,
    TeleopStop,
    advance_dual_gp_teleop,
)
from lerobot.scripts.crp_gp.ui_probe import GripperUiProbeManager
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import precise_sleep

if TYPE_CHECKING:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.model.kinematics import RobotKinematics
    from lerobot.processor import RobotProcessorPipeline
    from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
    from lerobot.teleoperators import Teleoperator
    from lerobot.types import RobotObservation

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RecordLoopResult:
    """Outcome of one dataset recording control loop."""

    frames: int
    aborted: bool = False  # True when CRP disconnects (TeleopStop)


@contextmanager
def stdin_episode_hotkeys(events: dict) -> Iterator[None]:
    """Terminal fallback when pynput misses arrow keys; focus this shell."""
    if not sys.stdin.isatty():
        yield
        return

    stop = threading.Event()
    old_term = termios.tcgetattr(sys.stdin)
    pending = ""

    def _drain_stdin() -> str:
        chunks: list[str] = []
        while select.select([sys.stdin], [], [], 0)[0]:
            chunks.append(sys.stdin.read(1))
        return "".join(chunks)

    def _strip_token(token: str) -> None:
        nonlocal pending
        while token in pending:
            idx = pending.index(token)
            pending = pending[:idx] + pending[idx + len(token) :]

    def _consume_pending() -> None:
        nonlocal pending
        while True:
            progress = False
            if "\x1b[C" in pending or "\x1bOC" in pending:
                logging.info("→ : end episode, save, next (also skips reset wait)")
                events["exit_early"] = True
                _strip_token("\x1b[C")
                _strip_token("\x1bOC")
                progress = True
            if "\x1b[D" in pending or "\x1bOD" in pending:
                logging.info("← : discard episode buffer, re-record")
                events["rerecord_episode"] = True
                events["exit_early"] = True
                _strip_token("\x1b[D")
                _strip_token("\x1bOD")
                progress = True
            if pending and pending[0] in ("d", "D"):
                logging.info("d: end episode, save, next (also skips reset wait)")
                events["exit_early"] = True
                pending = pending[1:]
                progress = True
            if pending and pending[0] in ("a", "A"):
                logging.info("a: discard episode buffer, re-record")
                events["rerecord_episode"] = True
                events["exit_early"] = True
                pending = pending[1:]
                progress = True
            if progress:
                continue

            # Drop lone ESC prefix bytes from arrow keys; Esc stop uses pynput only.
            if pending.startswith("\x1b"):
                if len(pending) >= 3 and pending[1] == "[" and pending[2] in "CD":
                    pending = pending[3:]
                    continue
                if len(pending) >= 3 and pending[1] == "O" and pending[2] in "CD":
                    pending = pending[3:]
                    continue
                if len(pending) < 3:
                    break
                pending = pending[1:]
                continue
            if pending:
                pending = pending[1:]
                continue
            break

    def _poll_keys() -> None:
        nonlocal pending
        while not stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                pending += _drain_stdin()
            _consume_pending()

    try:
        tty.setcbreak(sys.stdin.fileno())
        logging.info(
            "Episode keys — click THIS terminal: → or d = save & next; ← or a = re-record; "
            "Esc = stop (pynput). During reset, → or d skips wait and starts next episode."
        )
        threading.Thread(target=_poll_keys, daemon=True, name="stdin_episode_hotkeys").start()
        yield
    finally:
        stop.set()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)


@contextmanager
def ignore_signals_during_cleanup() -> Iterator[None]:
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, signal.SIG_IGN)
        except (ValueError, OSError):
            pass
    yield


@dataclass(frozen=True)
class ResetWaitResult:
    """Outcome of the between-episode reset pause (GP idle, servo on)."""

    aborted: bool = False


def wait_reset_between_episodes(
    robot: CRPArmDual,
    *,
    reset_time_s: float,
    events: dict,
) -> ResetWaitResult:
    """Wait between episodes without sending GP; CRP servo stays on.

    Waits up to ``reset_time_s``. ``→`` / ``d`` (``exit_early``) ends the wait immediately
    so the next episode can realign and record.
    """
    if reset_time_s <= 0:
        return ResetWaitResult()

    logger.info(
        "Reset: GP idle for %.0fs (servo on; reposition scene). "
        "→ or d to start next episode early. Countdown in last 10s.",
        reset_time_s,
    )
    events["rerecord_episode"] = False
    deadline = time.perf_counter() + float(reset_time_s)
    last_countdown_logged: int | None = None
    while time.perf_counter() < deadline:
        if events.get("stop_recording"):
            break
        if not robot.is_connected:
            logger.error("CRP disconnected during reset; stopping session.")
            return ResetWaitResult(aborted=True)
        if events.get("exit_early"):
            events["exit_early"] = False
            events["rerecord_episode"] = False
            logger.info("→ during reset: starting next episode (will realign).")
            break
        remaining = deadline - time.perf_counter()
        secs_left = max(0, int(math.ceil(remaining)))
        if secs_left <= 10 and secs_left != last_countdown_logged:
            logger.info("Reset: %ds...", secs_left)
            last_countdown_logged = secs_left
        precise_sleep(min(0.05, max(remaining, 0.0)))

    return ResetWaitResult()


@safe_stop_image_writer
def record_loop_crp_dual(
    robot: CRPArmDual,
    teleop: Teleoperator,
    kinematics: RobotKinematics,
    cfg: RecordDualCRPConfig,
    tele_cfg: TeleoperateDualCRPConfig,
    arms: tuple[ArmTeleopState, ArmTeleopState],
    timing: DualGPTiming,
    events: dict,
    dataset: LeRobotDataset,
    robot_observation_processor: RobotProcessorPipeline,
    *,
    control_time_s: float,
    single_task: str,
    ui_probe: GripperUiProbeManager | None = None,
    display_data: bool = False,
) -> RecordLoopResult:
    from lerobot.utils.feature_utils import build_dataset_frame
    from lerobot.utils.visualization_utils import log_rerun_data

    limit_gp_step = cfg.gp_position_step_mm > 0
    control_iv = 1.0 / float(cfg.control_fps)
    record_iv = 1.0 / float(cfg.dataset.fps)
    next_record_t = time.perf_counter()

    start_episode_t = time.perf_counter()
    timestamp = 0.0
    last_ui_log = 0.0
    frames_this_sec = 0
    last_fps_log_t = start_episode_t
    total_recorded_frames = 0
    disconnect_streak = [0]

    logger.info(
        "Recording at dataset.fps=%d; GP/UI50 control at %d Hz (mp4 duration = frames / %d)",
        cfg.dataset.fps,
        cfg.control_fps,
        cfg.dataset.fps,
    )

    while timestamp < control_time_s:
        loop_t0 = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        try:
            advance_dual_gp_teleop(
                teleop,
                robot,
                kinematics,
                tele_cfg,
                arms,
                timing,
                limit_gp_step=limit_gp_step,
                disconnect_streak=disconnect_streak,
            )
        except TeleopStop as exc:
            logger.error("CRP disconnected during recording (%s); stopping episode.", exc.reason)
            return RecordLoopResult(total_recorded_frames, aborted=True)

        now = time.perf_counter()
        if now >= next_record_t:
            if not robot.is_connected:
                logger.error("CRP disconnected during recording; stopping episode.")
                return RecordLoopResult(total_recorded_frames, aborted=True)
            t_record = time.perf_counter()
            try:
                obs = build_record_observation(robot)
            except (DeviceNotConnectedError, RuntimeError) as exc:
                logger.error(
                    "CRP observation read failed during recording (%s); stopping episode.",
                    exc,
                )
                return RecordLoopResult(total_recorded_frames, aborted=True)
            obs_ui, action_ui = read_ui_for_dataset(arms, ui_probe=ui_probe)
            obs_with_ui = {**obs, **obs_ui}
            obs_processed = robot_observation_processor(obs_with_ui)

            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)
            joint_values = {k: obs[k] for k in JOINT_POS_KEYS if k in obs}
            action_values = {**joint_values, **action_ui}
            action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)

            total_recorded_frames += 1
            frames_this_sec += 1
            if now - last_fps_log_t >= 1.0:
                window_s = now - last_fps_log_t
                actual_hz = frames_this_sec / window_s
                logger.info(
                    "record fps: %.1f Hz (target %d; set --dataset.fps≈%.0f for normal playback)",
                    actual_hz,
                    cfg.dataset.fps,
                    round(actual_hz),
                )
                frames_this_sec = 0
                last_fps_log_t = now

            if display_data:
                log_rerun_data(observation=obs_processed, action=action_values)

            if (
                ui_probe is not None
                and ui_probe.enabled
                and GP_LOG_HEARTBEAT_S > 0
                and (now - last_ui_log) >= GP_LOG_HEARTBEAT_S
            ):
                la, ra = arms
                logger.info(
                    "record ui\n%s\n%s",
                    ui_probe.format_arm_heartbeat_line(
                        "L", "left", tuple(la.command_gp[:3]), ui50_cmd_fallback=la.last_ui50
                    ),
                    ui_probe.format_arm_heartbeat_line(
                        "R", "right", tuple(ra.command_gp[:3]), ui50_cmd_fallback=ra.last_ui50
                    ),
                )
                last_ui_log = now

            next_record_t += record_iv
            if now - next_record_t > record_iv:
                next_record_t = now + record_iv
            record_ms = (time.perf_counter() - t_record) * 1e3
            if record_ms > control_iv * 1e3 * 2:
                logger.debug("record frame build took %.1fms (target control tick %.1fms)", record_ms, control_iv * 1e3)

        precise_sleep(max(control_iv - (time.perf_counter() - loop_t0), 0.0))
        timestamp = time.perf_counter() - start_episode_t

    if total_recorded_frames > 0:
        wall_s = time.perf_counter() - start_episode_t
        avg_hz = total_recorded_frames / wall_s
        video_s = total_recorded_frames / cfg.dataset.fps
        logger.info(
            "record fps episode summary: %d frames in %.1fs wall → avg %.1f Hz; "
            "mp4 length %.1fs at dataset.fps=%d (suggest --dataset.fps=%.0f)",
            total_recorded_frames,
            wall_s,
            avg_hz,
            video_s,
            cfg.dataset.fps,
            round(avg_hz),
        )

    return RecordLoopResult(total_recorded_frames, aborted=False)
