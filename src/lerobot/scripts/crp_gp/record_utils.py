"""Dataset feature helpers and UI reads for dual CRP recording (no heavy dataset imports)."""

from __future__ import annotations

import logging
import os
import shutil
from pathlib import Path
from typing import TYPE_CHECKING, Any

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.robots.crp_arm_dual.crp_ui import (
    GRIPPER_UI_OPEN,
    GRIPPER_UI_POSITION,
    GRIPPER_UI_SPEED_OUT,
    GRIPPER_UI_TORQUE_OUT,
)
from lerobot.scripts.crp_gp.config import (
    ACTION_UI_KEYS,
    ArmTeleopState,
    JOINT_POS_KEYS,
    OBS_UI_KEYS,
    RecordDualCRPConfig,
)
from lerobot.utils.constants import HF_LEROBOT_HOME

if TYPE_CHECKING:
    from lerobot.scripts.crp_gp.ui_probe import GripperUiProbeManager

logger = logging.getLogger(__name__)

_UI56_MAP = {
    GRIPPER_UI_POSITION: "ui56",
    GRIPPER_UI_SPEED_OUT: "ui57",
    GRIPPER_UI_TORQUE_OUT: "ui58",
}


def dataset_root(cfg: RecordDualCRPConfig) -> Path:
    return Path(cfg.dataset.root) if cfg.dataset.root is not None else HF_LEROBOT_HOME / cfg.dataset.repo_id


def assert_local_dataset_for_resume(cfg: RecordDualCRPConfig) -> Path:
    root = dataset_root(cfg)
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        return root
    raise FileNotFoundError(
        f"Cannot resume: no local dataset at {root} (missing {info_path.name}). "
        "First recording: use --resume=false. After one saved episode, use --resume=true."
    )


def prepare_new_dataset_root(cfg: RecordDualCRPConfig) -> None:
    root = dataset_root(cfg)
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        raise FileExistsError(
            f"Dataset already exists at {root}. Use --resume=true to append, "
            "or pick another --dataset.repo_id."
        )
    if root.exists():
        logger.warning("Removing incomplete dataset directory from a previous failed run: %s", root)
        shutil.rmtree(root)


def resolve_orbbec_top_camera(robot_cfg: CRPArmDualConfig) -> None:
    """Auto-discover Orbbec RGB V4L2 node and apply format/exposure before ``robot.connect()``."""
    top = robot_cfg.cameras.get("top")
    if not isinstance(top, OpenCVCameraConfig):
        return

    from lerobot.scripts.crp_gp.orbbec_rgb_discovery import apply_orbbec_v4l2_tuning, capture_best_orbbec_rgb

    configured = str(top.index_or_path)
    if os.environ.get("ORBBEC_AUTO_DISCOVER", "1").lower() in ("0", "false", "no"):
        if Path(configured).exists():
            apply_orbbec_v4l2_tuning(configured)
            logger.info("Orbbec top: auto-discover disabled; using %s", configured)
        return

    env_path = os.environ.get("ORBBEC_PATH", "")
    prefer = env_path if env_path and Path(env_path).exists() else configured
    logger.info("Orbbec top: auto-discovering RGB node (prefer=%s)...", prefer)
    best = capture_best_orbbec_rgb(prefer=prefer if Path(prefer).exists() else None)
    if best is None:
        if Path(configured).exists():
            logger.warning(
                "Orbbec auto-discovery found no RGB node; falling back to configured path %s",
                configured,
            )
            apply_orbbec_v4l2_tuning(configured)
            return
        raise ConnectionError(
            "Orbbec top camera: no RGB V4L2 node found. Replug USB3 or set ORBBEC_PATH=/dev/videoN"
        )

    if configured != best.path:
        logger.info(
            "Orbbec top: using %s (configured %s, sharpness=%.0f, score=%d)",
            best.path,
            configured,
            best.sharpness,
            best.score,
        )
    else:
        logger.info(
            "Orbbec top: confirmed %s (sharpness=%.0f, score=%d)",
            best.path,
            best.sharpness,
            best.score,
        )
    if best.sharpness > 20_000 or best.frame.shape[0] != 480:
        logger.warning(
            "Orbbec top node may be IR/wrong stream (shape=%s sharp=%.0f); replug USB",
            best.frame.shape,
            best.sharpness,
        )

    top.index_or_path = best.path
    apply_orbbec_v4l2_tuning(best.path)


def crp_dual_hw_observation_features(robot: CRPArmDual) -> dict[str, type | tuple]:
    feats: dict[str, type | tuple] = {k: float for k in JOINT_POS_KEYS}
    feats.update({k: float for k in OBS_UI_KEYS})
    feats.update(robot._cameras_ft)
    return feats


def crp_dual_hw_action_features() -> dict[str, type]:
    feats = {k: float for k in JOINT_POS_KEYS}
    feats.update({k: float for k in ACTION_UI_KEYS})
    return feats


def build_record_observation(robot: CRPArmDual, *, max_camera_age_ms: int = 500) -> dict[str, Any]:
    """Joints + latest camera frames without blocking ``cam.read()`` on the GP thread."""
    obs_dict = robot.read_crp_joints()
    for cam_key, cam in robot.cameras.items():
        try:
            obs_dict[cam_key] = cam.read_latest(max_age_ms=max_camera_age_ms)
        except (TimeoutError, RuntimeError):
            obs_dict[cam_key] = cam.read()
    return obs_dict


def read_ui_for_dataset(
    arms: tuple[ArmTeleopState, ArmTeleopState],
    ui_probe: GripperUiProbeManager | None = None,
) -> tuple[dict[str, float], dict[str, float]]:
    """UI50 from teleop state; UI56–58 from probe cache (no SDK lock on every frame)."""
    obs_ui: dict[str, float] = {}
    action_ui: dict[str, float] = {}

    for label, arm in (("left", arms[0]), ("right", arms[1])):
        latest = ui_probe.latest(label) if ui_probe is not None and ui_probe.enabled else None
        if latest is not None and GRIPPER_UI_OPEN in latest:
            ui50 = float(latest[GRIPPER_UI_OPEN])
        elif arm.last_ui50 is not None:
            ui50 = float(arm.last_ui50)
        else:
            ui50 = 0.0
        action_ui[f"{label}_ui50"] = ui50
        for idx, suffix in _UI56_MAP.items():
            obs_ui[f"{label}_{suffix}"] = float(latest[idx]) if latest is not None and idx in latest else 0.0

    return obs_ui, action_ui
