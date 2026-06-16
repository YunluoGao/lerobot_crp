"""Closed-loop policy deploy on dual CRP arms via GJ10/GJ20 (``CrpDualGjExecutor``)."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.train import TRAIN_CONFIG_NAME, TrainPipelineConfig
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import make_robot_action, validate_visual_features_consistency
from lerobot.processor import PolicyAction, PolicyProcessorPipeline, make_default_processors
from lerobot.processor.rename_processor import rename_stats
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.crp_ui import (
    GRIPPER_UI_POSITION,
    GRIPPER_UI_SPEED_OUT,
    GRIPPER_UI_TORQUE_OUT,
)
from lerobot.scripts.crp_gp.action_executor import (
    CrpDualGjExecutor,
    read_dual_joint_snapshot,
    validate_dual_action_keys,
)
from lerobot.scripts.crp_gp.record_utils import build_record_observation
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.device_utils import get_safe_torch_device
from lerobot.utils.feature_utils import build_dataset_frame, dataset_to_policy_features
from lerobot.utils.robot_utils import precise_sleep

logger = logging.getLogger(__name__)

_UI_OBS_INDICES: tuple[tuple[str, int, str], ...] = (
    ("first", GRIPPER_UI_POSITION, "left_ui56"),
    ("first", GRIPPER_UI_SPEED_OUT, "left_ui57"),
    ("first", GRIPPER_UI_TORQUE_OUT, "left_ui58"),
    ("second", GRIPPER_UI_POSITION, "right_ui56"),
    ("second", GRIPPER_UI_SPEED_OUT, "right_ui57"),
    ("second", GRIPPER_UI_TORQUE_OUT, "right_ui58"),
)


@dataclass
class DeployLoopResult:
    ticks: int
    aborted: bool = False


def policy_dir_for_train_config(policy_pretrained: str) -> str:
    p = Path(policy_pretrained).resolve()
    if (p / TRAIN_CONFIG_NAME).is_file():
        return str(p)
    sub = p / "pretrained_model"
    if (sub / TRAIN_CONFIG_NAME).is_file():
        return str(sub)
    raise FileNotFoundError(
        f"Could not find {TRAIN_CONFIG_NAME} under {p} or {sub}. "
        "Use --policy.path=outputs/.../checkpoints/last/pretrained_model"
    )


def training_dataset_meta(policy_pretrained: str) -> LeRobotDatasetMetadata:
    policy_dir = policy_dir_for_train_config(policy_pretrained)
    train_cfg = TrainPipelineConfig.from_pretrained(policy_dir)
    dcfg = train_cfg.dataset
    return LeRobotDatasetMetadata(dcfg.repo_id, root=dcfg.root, revision=dcfg.revision)


def load_policy_bundle(
    policy_pretrained: str,
    *,
    rename_map: dict[str, str] | None = None,
) -> tuple[PreTrainedPolicy, PolicyProcessorPipeline, PolicyProcessorPipeline, LeRobotDatasetMetadata]:
    ckpt = Path(policy_pretrained).resolve()
    policy_dir = policy_dir_for_train_config(str(ckpt))
    policy_cfg = PreTrainedConfig.from_pretrained(policy_dir)
    policy_cfg.pretrained_path = Path(policy_dir)

    ds_meta = training_dataset_meta(str(ckpt))
    rename_map = rename_map or {}
    validate_visual_features_consistency(
        policy_cfg, dataset_to_policy_features(ds_meta.features)
    )

    policy = make_policy(policy_cfg, ds_meta=ds_meta, rename_map=rename_map)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=policy_cfg.pretrained_path,
        dataset_stats=rename_stats(ds_meta.stats, rename_map),
        preprocessor_overrides={
            "device_processor": {"device": policy_cfg.device},
            "rename_observations_processor": {"rename_map": rename_map},
        },
    )
    return policy, preprocessor, postprocessor, ds_meta


def read_deploy_observation(robot: CRPArmDual) -> dict[str, Any]:
    """Joints + cameras + gripper UI56–58 (matches training ``observation.state``)."""
    obs = build_record_observation(robot)
    for arm, ui_idx, key in _UI_OBS_INDICES:
        try:
            obs[key] = float(robot.get_ui(arm, ui_idx))
        except Exception as exc:
            logger.debug("read_deploy_observation %s: %s", key, exc)
            obs[key] = 0.0
    return obs


def run_policy_deploy_crp_dual(
    robot: CRPArmDual,
    executor: CrpDualGjExecutor,
    policy: PreTrainedPolicy,
    preprocessor: PolicyProcessorPipeline,
    postprocessor: PolicyProcessorPipeline,
    ds_meta: LeRobotDatasetMetadata,
    *,
    fps: float,
    duration_s: float | None,
    single_task: str | None,
    events: dict | None,
) -> DeployLoopResult:
    """Fixed-rate loop: obs → policy → ``CrpDualGjExecutor.send_action``."""
    validate_dual_action_keys(list(ds_meta.features[ACTION]["names"]))
    executor.init_gj_registers()

    device = get_safe_torch_device(policy.config.device)
    policy.reset()
    preprocessor.reset()
    postprocessor.reset()

    interval_s = 1.0 / float(fps)
    start_joints = read_dual_joint_snapshot(robot)
    start_t = time.perf_counter()
    ticks = 0
    stationary_warned = False
    motion_check_after = max(int(fps * 3), 1)
    log_interval_s = max(1.0, interval_s * max(1, int(fps)))

    _, _, robot_observation_processor = make_default_processors()

    next_log_t = start_t
    while True:
        if events is not None and events.get("exit_early"):
            logger.info("Exit requested (keyboard).")
            break
        if duration_s is not None and (time.perf_counter() - start_t) >= duration_s:
            logger.info("Duration %.1fs reached.", duration_s)
            break
        if not robot.is_connected:
            logger.error("CRP disconnected during deploy at tick %d.", ticks)
            return DeployLoopResult(ticks=ticks, aborted=True)

        loop_t0 = time.perf_counter()
        try:
            raw_obs = read_deploy_observation(robot)
            obs_processed = robot_observation_processor(raw_obs)
            obs_frame = build_dataset_frame(ds_meta.features, obs_processed, prefix=OBS_STR)
        except Exception as exc:
            logger.error("Observation build failed at tick %d: %s", ticks, exc)
            return DeployLoopResult(ticks=ticks, aborted=True)

        try:
            action_tensor = predict_action(
                observation=obs_frame,
                policy=policy,
                device=device,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                use_amp=policy.config.use_amp,
                task=single_task,
                robot_type=robot.name,
            )
            action_dict = make_robot_action(action_tensor, ds_meta.features)
        except Exception as exc:
            logger.error(
                "Policy inference failed at tick %d: %s (check cameras: %s)",
                ticks,
                exc,
                sorted(robot.cameras),
            )
            return DeployLoopResult(ticks=ticks, aborted=True)

        sent = executor.send_action(action_dict)
        ticks += 1

        now = time.perf_counter()
        if now >= next_log_t:
            snap = read_dual_joint_snapshot(robot)
            left_moved = max(abs(a - b) for a, b in zip(snap.left, start_joints.left, strict=True))
            right_moved = max(
                abs(a - b) for a, b in zip(snap.right, start_joints.right, strict=True)
            )
            logger.info(
                "deploy tick=%d left Δmax=%.2f° right Δmax=%.2f° sent_keys=%d",
                ticks,
                left_moved,
                right_moved,
                len(sent),
            )
            next_log_t = now + log_interval_s

        if not stationary_warned and ticks >= motion_check_after:
            snap = read_dual_joint_snapshot(robot)
            left_moved = max(abs(a - b) for a, b in zip(snap.left, start_joints.left, strict=True))
            right_moved = max(
                abs(a - b) for a, b in zip(snap.right, start_joints.right, strict=True)
            )
            if left_moved < 1.0 and right_moved < 1.0:
                logger.warning(
                    "Arms barely moved after %d ticks (left Δmax=%.2f° right Δmax=%.2f°). "
                    "Ensure teach GJ programs are RUNNING (solid green).",
                    ticks,
                    left_moved,
                    right_moved,
                )
            stationary_warned = True

        dt_s = time.perf_counter() - loop_t0
        precise_sleep(max(interval_s - dt_s, 0.0))

    logger.info("Deploy finished: %d ticks.", ticks)
    return DeployLoopResult(ticks=ticks, aborted=False)
