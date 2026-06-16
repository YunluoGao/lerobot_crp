"""Dataset episode replay for dual CRP arms (GJ or GP stream)."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.scripts.crp_gp.action_executor import (
    CrpDualGjExecutor,
    action_array_to_dict,
    read_dual_joint_snapshot,
    validate_dual_action_keys,
)
from lerobot.utils.constants import ACTION
from lerobot.utils.robot_utils import precise_sleep

logger = logging.getLogger(__name__)

GP_SETTLE_S = 2.0


@dataclass
class ReplayLoopResult:
    frames: int
    aborted: bool = False


def _check_stationary_motion(
    robot: CRPArmDual,
    start_joints,
    *,
    sent_frames: int,
    motion_check_after: int,
    stationary_warned: bool,
    gp_mode: bool,
) -> bool:
    if stationary_warned or sent_frames < motion_check_after:
        return stationary_warned
    now = read_dual_joint_snapshot(robot)
    left_moved = max(abs(a - b) for a, b in zip(now.left, start_joints.left, strict=True))
    right_moved = max(abs(a - b) for a, b in zip(now.right, start_joints.right, strict=True))
    if left_moved < 1.0 and right_moved < 1.0:
        hint = (
            "GP joint→TCP is heuristic for both arms; prefer default GJ replay (omit gp_nudge)."
            if gp_mode
            else (
                "GJ registers updated but arms did not move — ensure GP was locked at session "
                "start and teach-pendant program is running (solid green)."
            )
        )
        logger.warning(
            "Arms barely moved after %d frames (left Δmax=%.2fdeg right Δmax=%.2fdeg). %s",
            sent_frames,
            left_moved,
            right_moved,
            hint,
        )
    return True


def _replay_joint_loop(
    robot: CRPArmDual,
    dataset: LeRobotDataset,
    executor: CrpDualGjExecutor,
    *,
    fps: float,
) -> ReplayLoopResult:
    executor.init_gj_registers()
    start_joints = read_dual_joint_snapshot(robot)

    interval_s = 1.0 / float(fps)
    sent_frames = 0
    stationary_warned = False
    motion_check_after = max(int(fps * 3), 1)

    for idx in range(dataset.num_frames):
        loop_t0 = time.perf_counter()

        if not robot.is_connected:
            logger.error("CRP disconnected during replay at frame %d; stopping.", idx)
            return ReplayLoopResult(frames=sent_frames, aborted=True)

        action_array = dataset[idx][ACTION]
        action = action_array_to_dict(list(dataset.meta.features[ACTION]["names"]), action_array)
        executor.send_action(action)
        sent_frames += 1

        if idx == 0 or (idx + 1) % max(1, int(fps)) == 0:
            logger.info("Replay frame %d/%d", idx + 1, dataset.num_frames)

        stationary_warned = _check_stationary_motion(
            robot,
            start_joints,
            sent_frames=sent_frames,
            motion_check_after=motion_check_after,
            stationary_warned=stationary_warned,
            gp_mode=False,
        )

        dt_s = time.perf_counter() - loop_t0
        precise_sleep(max(interval_s - dt_s, 0.0))

    logger.info("Replay finished: %d frames sent.", sent_frames)
    return ReplayLoopResult(frames=sent_frames)


def _replay_gp_stream_loop(
    robot: CRPArmDual,
    dataset: LeRobotDataset,
    executor: CrpDualGjExecutor,
    *,
    fps: float,
    gp_send_fps: int,
) -> ReplayLoopResult:
    action_names = list(dataset.meta.features[ACTION]["names"])
    start_joints = read_dual_joint_snapshot(robot)

    frame_iv = 1.0 / float(fps)
    gp_iv = 1.0 / float(gp_send_fps)
    t0 = time.perf_counter()
    next_frame = t0
    next_gp = t0
    frame_idx = 0
    sent_frames = 0
    stationary_warned = False
    motion_check_after = max(int(fps * 3), 1)
    settle_until = t0 + (dataset.num_frames / fps) + GP_SETTLE_S

    logger.info(
        "GP stream replay: both arms @ dataset %.3f Hz + GP %d Hz (settle %.1fs)",
        fps,
        gp_send_fps,
        GP_SETTLE_S,
    )

    while True:
        now = time.perf_counter()
        if now >= settle_until and frame_idx >= dataset.num_frames:
            break

        if not robot.is_connected:
            logger.error("CRP disconnected during replay at frame %d; stopping.", frame_idx)
            return ReplayLoopResult(frames=sent_frames, aborted=True)

        if frame_idx < dataset.num_frames and now >= next_frame:
            action_array = dataset[frame_idx][ACTION]
            action = action_array_to_dict(action_names, action_array)
            executor.update_dataset_frame(action)
            sent_frames += 1
            frame_idx += 1

            if frame_idx == 1 or frame_idx % max(1, int(fps)) == 0:
                logger.info("Replay frame %d/%d", frame_idx, dataset.num_frames)

            stationary_warned = _check_stationary_motion(
                robot,
                start_joints,
                sent_frames=sent_frames,
                motion_check_after=motion_check_after,
                stationary_warned=stationary_warned,
                gp_mode=True,
            )
            next_frame += frame_iv

        if now >= next_gp:
            executor.send_gp_tick_once()
            next_gp += gp_iv

        sleep_until = min(
            next_gp,
            next_frame if frame_idx < dataset.num_frames else float("inf"),
            settle_until,
        )
        precise_sleep(max(0.0, sleep_until - time.perf_counter()))

    logger.info("Replay finished: %d frames sent.", sent_frames)
    return ReplayLoopResult(frames=sent_frames)


def replay_episode_crp_dual(
    robot: CRPArmDual,
    dataset: LeRobotDataset,
    executor: CrpDualGjExecutor,
    *,
    fps: float,
    gp_send_fps: int | None = None,
) -> ReplayLoopResult:
    if dataset.num_frames == 0:
        logger.warning("Episode has 0 frames; nothing to replay.")
        return ReplayLoopResult(frames=0)

    action_names = list(dataset.meta.features[ACTION]["names"])
    validate_dual_action_keys(action_names)

    logger.info(
        "Replaying %s episode %s: %d frames @ %.3f Hz",
        dataset.repo_id,
        dataset.episodes,
        dataset.num_frames,
        fps,
    )

    if executor.uses_dual_gp_stream():
        if gp_send_fps is None or gp_send_fps <= 0:
            raise ValueError("gp_send_fps must be > 0 for gp_nudge replay")
        return _replay_gp_stream_loop(
            robot,
            dataset,
            executor,
            fps=fps,
            gp_send_fps=gp_send_fps,
        )

    return _replay_joint_loop(robot, dataset, executor, fps=fps)
