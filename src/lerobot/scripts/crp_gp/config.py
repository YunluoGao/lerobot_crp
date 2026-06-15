from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs.video import VideoEncoderConfig
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.teleoperators.bi_so_leader.config_bi_so_leader import BiSOLeaderConfig
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from .mappers.so101_urdf import CrpGPAlignState

# GP / UI50 teleop loop rate during record (dataset.fps is the write cadence).
CRP_RECORD_CONTROL_FPS = 80

# Stable camera paths / serials on the lab PC (override via --robot.cameras or env).
# Gemini 335 exposes many V4L2 nodes (depth/IR/metadata/RGB). /dev/videoN changes after USB replug.
# Record entry auto-discovers RGB via resolve_orbbec_top_camera(); this is only a fallback prefer hint.
# Override: ORBBEC_PATH=... ORBBEC_AUTO_DISCOVER=0 to skip auto-discovery.
DEFAULT_ORBBEC_TOP_PATH = "/dev/video6"
DEFAULT_RS_LEFT_SERIAL = "218622273151"
DEFAULT_RS_RIGHT_SERIAL = "218622278121"
DEFAULT_DATASET_REPO_ID = "user/date_name_ordinal"

JOINT_POS_KEYS: tuple[str, ...] = tuple(
    f"{prefix}_j{i}.pos" for prefix in ("left", "right") for i in range(1, 7)
)
OBS_UI_KEYS: tuple[str, ...] = tuple(
    f"{arm}_ui{idx}" for arm in ("left", "right") for idx in (56, 57, 58)
)
ACTION_UI_KEYS: tuple[str, ...] = ("left_ui50", "right_ui50")


def _legacy_so101_calibration_dir():
    legacy = HF_LEROBOT_CALIBRATION / TELEOPERATORS / "so101_leader"
    return legacy if legacy.is_dir() else None


def _default_teleop() -> BiSOLeaderConfig:
    return BiSOLeaderConfig(
        left_arm_config=SOLeaderConfig(port="/dev/ttyACM1", use_degrees=True),
        right_arm_config=SOLeaderConfig(port="/dev/ttyACM0", use_degrees=True),
        left_leader_id="1",
        right_leader_id="2",
        calibration_dir=_legacy_so101_calibration_dir(),
        id="crobotp_leader",
    )


def _default_cameras() -> dict[str, CameraConfig]:
    return {
        "top": OpenCVCameraConfig(
            index_or_path=DEFAULT_ORBBEC_TOP_PATH,
            width=640,
            height=480,
            fps=30,
            fourcc=None,
            warmup_s=2,
        ),
        "left_wrist": RealSenseCameraConfig(
            serial_number_or_name=DEFAULT_RS_LEFT_SERIAL,
            width=848,
            height=480,
            fps=30,
            warmup_s=2,
        ),
        "right_wrist": RealSenseCameraConfig(
            serial_number_or_name=DEFAULT_RS_RIGHT_SERIAL,
            width=848,
            height=480,
            fps=30,
            warmup_s=2,
        ),
    }


def _default_camera_encoder() -> VideoEncoderConfig:
    """H.264 + lower CRF: top JPG was sharp but AV1 crf=30 MP4 looked blocky in viz."""
    return VideoEncoderConfig(vcodec="h264", crf=18)


def _default_robot_for_teleop() -> CRPArmDualConfig:
    """Teleop: SO101 + CRP only (no cameras — faster connect, no USB contention)."""
    return CRPArmDualConfig(
        ip1="192.168.0.100",
        ip2="192.168.0.101",
        id="crobotp",
        cameras={},
    )


def _default_robot_for_record() -> CRPArmDualConfig:
    return CRPArmDualConfig(
        ip1="192.168.0.100",
        ip2="192.168.0.101",
        id="crobotp",
        cameras=_default_cameras(),
    )


@dataclass
class ArmGPConfig:
    """Per-arm GP / wrist / Z parameters (CLI: ``--left.*`` / ``--right.*``)."""

    gp_index: int = 10
    wrist_roll_sign: float = -1.0
    # 0 = do not map SO101 J4 (wrist_flex) to CRP orientation (tool stays vertical at align).
    wrist_flex_sign: float = 0.0
    wrist_roll_max_step_deg: float = 12.0
    wrist_flex_max_step_deg: float | None = 12.0
    z_floor_mm: float = 40.0
    z_scale: float = 2.0


@dataclass
class RightArmGPConfig(ArmGPConfig):
    """Right arm defaults (``gp_index=20`` survives partial ``--right.*`` CLI overrides)."""

    gp_index: int = 20
    wrist_roll_max_step_deg: float = 2.0
    wrist_flex_max_step_deg: float | None = 1.0


@dataclass
class TeleoperateDualCRPConfig:
    """Shared config for dual-arm CRP teleop loop."""

    teleop: BiSOLeaderConfig = field(default_factory=_default_teleop)
    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_teleop)
    left: ArmGPConfig = field(default_factory=ArmGPConfig)
    right: RightArmGPConfig = field(default_factory=RightArmGPConfig)
    fps: int = 80
    gp_position_step_mm: float = 80.0
    gp_send_fps: int = 50
    # 0 = send gripper on every main-loop tick (uses ``fps``); else cap UI50 rate (Hz).
    gripper_send_fps: int = 0
    # Min SO101 gripper.pos change (0–100) before sending UI50 (~0.39 ≈ one UI step).
    gripper_min_pos_delta: float = 0.35
    gripper_ui_probe: bool = True
    gripper_ui_probe_hz: float = 1.0
    # Pause after servo enable before GP align/init (seconds); log countdown in terminal.
    gp_align_delay_s: float = 5.0


@dataclass
class DualGPTiming:
    last_gp: float = 0.0
    last_gr: float = 0.0


@dataclass
class ArmTeleopState:
    """Per-arm teleop context built once at startup (hot loop reads fields only)."""

    label: str
    gp_align: CrpGPAlignState
    gp_index: int
    max_step_roll_per_loop: float | None
    max_step_flex_per_loop: float | None
    send_gps_fn: Callable[[int, list], bool]
    set_ui_fn: Callable[[int, int], None]
    pos_step_xy_per_loop: float = 0.0
    step_z_mm: float | None = None
    last_sent: list[float] = field(default_factory=list)
    command_gp: list[float] = field(default_factory=list)
    gripper_pos: float | None = None
    last_gripper_pos: float | None = None
    last_ui50: int | None = None


@dataclass
class CrpDualDatasetRecordConfig(DatasetRecordConfig):
    """CRP dual record defaults (class fields so partial ``--dataset.*`` CLI keeps fps=16 etc.)."""

    repo_id: str = DEFAULT_DATASET_REPO_ID
    single_task: str = "assemble"
    fps: int = 16
    episode_time_s: int | float = 300
    reset_time_s: int | float = 30
    num_episodes: int = 30
    push_to_hub: bool = False
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 2
    camera_encoder: VideoEncoderConfig = field(default_factory=_default_camera_encoder)


@dataclass
class RecordDualCRPConfig:
    """Dual CRP dataset recording (80Hz GP teleop + dataset.fps write cadence)."""

    teleop: BiSOLeaderConfig = field(default_factory=_default_teleop)
    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_record)
    dataset: CrpDualDatasetRecordConfig = field(default_factory=CrpDualDatasetRecordConfig)
    left: ArmGPConfig = field(default_factory=ArmGPConfig)
    right: RightArmGPConfig = field(default_factory=RightArmGPConfig)
    control_fps: int = CRP_RECORD_CONTROL_FPS
    gp_position_step_mm: float = 80.0
    gp_send_fps: int = 50
    gripper_send_fps: int = 0
    gripper_min_pos_delta: float = 0.35
    gripper_ui_probe: bool = True
    gripper_ui_probe_hz: float = 1.0
    gp_align_delay_s: float = 5.0
    display_data: bool = False
    play_sounds: bool = True
    resume: bool = False


def teleop_cfg_from_record(record_cfg: RecordDualCRPConfig) -> TeleoperateDualCRPConfig:
    """Build teleop loop config: control at ``control_fps``, independent of ``dataset.fps``."""
    return TeleoperateDualCRPConfig(
        robot=record_cfg.robot,
        teleop=record_cfg.teleop,
        left=record_cfg.left,
        right=record_cfg.right,
        fps=record_cfg.control_fps,
        gp_position_step_mm=record_cfg.gp_position_step_mm,
        gp_send_fps=record_cfg.gp_send_fps,
        gripper_send_fps=record_cfg.gripper_send_fps,
        gripper_min_pos_delta=record_cfg.gripper_min_pos_delta,
        gripper_ui_probe=record_cfg.gripper_ui_probe,
        gripper_ui_probe_hz=record_cfg.gripper_ui_probe_hz,
        gp_align_delay_s=record_cfg.gp_align_delay_s,
    )

