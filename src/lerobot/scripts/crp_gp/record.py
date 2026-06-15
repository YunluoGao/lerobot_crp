"""Dual CRP dataset recording on top of the GP teleop loop."""

from __future__ import annotations

import logging
import math
import re
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

# CSI/SS3 arrow sequences: \x1b[C, \x1b[ D, \x1b[1;5C, \x1bOC, \x1b[d, etc.
_CSI_ARROW_RE = re.compile(r"\x1b\[[0-9;]*[ ]*([CDcd])")
_SS3_ARROW_RE = re.compile(r"\x1bO([CDcd])")
_ESC_WAIT_S = 0.05


def _open_terminal_input():
    """Return (readable stream, owns_fd) for keyboard input, or (None, False) if unavailable."""
    try:
        return open("/dev/tty", "r+b", buffering=0), True  # noqa: SIM115
    except OSError:
        if sys.stdin.isatty():
            return sys.stdin, False
        return None, False


def _read_terminal_char(stream) -> str:
    chunk = stream.read(1)
    if isinstance(chunk, bytes):
        return chunk.decode("latin-1", errors="replace")
    return chunk


def new_episode_events() -> dict:
    """Fresh event flags for CRP episode / session keyboard control (stdin hotkeys)."""
    return {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
        "hotkeys_paused": False,
    }


@dataclass(frozen=True)
class RecordLoopResult:
    """Outcome of one dataset recording control loop."""

    frames: int
    aborted: bool = False  # True when CRP disconnects (TeleopStop)


@contextmanager
def stdin_episode_hotkeys(events: dict) -> Iterator[None]:
    """Read episode control keys from this terminal (must stay focused)."""
    tty_in, owns_tty = _open_terminal_input()
    if tty_in is None:
        logging.warning(
            "No interactive terminal (/dev/tty): episode hotkeys disabled. "
            "Run in a real shell (→ save, ← re-record, q stop session)."
        )
        yield
        return

    stop = threading.Event()
    old_term = termios.tcgetattr(tty_in)
    pending = ""

    def _drain_input() -> str:
        if stop.is_set():
            return ""
        chunks: list[str] = []
        try:
            while select.select([tty_in], [], [], 0)[0]:
                chunks.append(_read_terminal_char(tty_in))
        except (ValueError, OSError):
            return ""
        return "".join(chunks)

    def _find_arrow() -> re.Match[str] | None:
        return _CSI_ARROW_RE.search(pending) or _SS3_ARROW_RE.search(pending)

    def _apply_arrow_final(final: str) -> None:
        if final in "Cc":
            logging.info("→ (arrow): end episode, save, next (also skips reset wait)")
            events["exit_early"] = True
        elif final in "Dd":
            logging.info("← (arrow): discard episode buffer, re-record (restarts reset countdown if waiting)")
            events["rerecord_episode"] = True
            events["exit_early"] = True

    def _consume_pending() -> None:
        nonlocal pending
        if events.get("hotkeys_paused"):
            pending = ""
            return
        while True:
            match = _find_arrow()
            if match is not None:
                _apply_arrow_final(match.group(1))
                pending = pending[: match.start()] + pending[match.end() :]
                continue
            if pending and pending[0] in ("q", "Q"):
                logging.info("q: stop recording session (skip remaining episodes)")
                events["stop_recording"] = True
                events["exit_early"] = True
                pending = pending[1:]
                continue
            # Hold partial ESC sequences; drop other stray bytes only.
            if pending.startswith("\x1b"):
                break
            if pending:
                pending = pending[1:]
                continue
            break

    def _wait_for_escape_sequence() -> None:
        nonlocal pending
        if not pending.startswith("\x1b"):
            return
        deadline = time.perf_counter() + _ESC_WAIT_S
        while time.perf_counter() < deadline:
            if stop.is_set():
                return
            if _find_arrow() is not None:
                _consume_pending()
                return
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            try:
                more, _, _ = select.select([tty_in], [], [], min(remaining, 0.02))
            except (ValueError, OSError):
                return
            if more:
                pending += _drain_input()
                _consume_pending()
                if not pending.startswith("\x1b"):
                    return
        if pending == "\x1b" or (pending.startswith("\x1b") and _find_arrow() is None):
            logger.debug("Discarding unrecognized stdin escape sequence: %r", pending)
            pending = ""

    def _poll_keys() -> None:
        nonlocal pending
        while not stop.is_set():
            try:
                ready, _, _ = select.select([tty_in], [], [], 0.05)
            except (ValueError, OSError):
                break
            if ready:
                pending += _drain_input()
            if events.get("hotkeys_paused"):
                pending = ""
                continue
            _consume_pending()
            _wait_for_escape_sequence()

    poll_thread: threading.Thread | None = None
    try:
        tty.setcbreak(tty_in.fileno())
        attr = termios.tcgetattr(tty_in)
        attr[3] &= ~termios.ECHO
        termios.tcsetattr(tty_in, termios.TCSADRAIN, attr)
        termios.tcflush(tty_in, termios.TCIFLUSH)
        logging.info(
            "Episode keys — keep THIS terminal focused: → = save & next; ← = re-record; q = stop session. "
            "During reset, ← restarts countdown; → skips wait. Hotkeys are ignored while encoding video."
        )
        poll_thread = threading.Thread(target=_poll_keys, daemon=True, name="stdin_episode_hotkeys")
        poll_thread.start()
        yield
    finally:
        stop.set()
        if poll_thread is not None:
            poll_thread.join(timeout=0.2)
        termios.tcsetattr(tty_in, termios.TCSADRAIN, old_term)
        if owns_tty:
            tty_in.close()


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
    label: str = "Reset",
    dataset: LeRobotDataset | None = None,
) -> ResetWaitResult:
    """Wait between episodes without sending GP; CRP servo stays on.

    Waits up to ``reset_time_s``. ``→`` (``exit_early``) ends the wait immediately
    so the next episode can realign and record. ``←`` during the wait restarts the
    full countdown (and clears any in-progress episode buffer when ``dataset`` is given).
    """
    if reset_time_s <= 0:
        return ResetWaitResult()

    logger.info(
        "%s: GP idle for %.0fs (servo on; reposition scene). "
        "← restarts countdown; → starts next take early. Countdown in last 10s.",
        label,
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
        if events.get("rerecord_episode"):
            events["rerecord_episode"] = False
            events["exit_early"] = False
            if dataset is not None:
                dataset.clear_episode_buffer()
            deadline = time.perf_counter() + float(reset_time_s)
            last_countdown_logged = None
            logger.info(
                "← during %s: buffer cleared, restarting %.0fs countdown.",
                label.lower(),
                reset_time_s,
            )
            continue
        if events.get("exit_early"):
            events["exit_early"] = False
            events["rerecord_episode"] = False
            logger.info("→ during %s: starting next take (will realign).", label.lower())
            break
        remaining = deadline - time.perf_counter()
        secs_left = max(0, int(math.ceil(remaining)))
        if secs_left <= 10 and secs_left != last_countdown_logged:
            logger.info("%s: %ds...", label, secs_left)
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
