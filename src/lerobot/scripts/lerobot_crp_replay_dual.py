# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Replay a LeRobot dataset episode on dual CRP arms.

Default (``--right_joint_backend=auto``): left ``set_GJs(GJ10)`` + right
``set_GJs_second(GJ20)`` (native CrpRobotPy on ip2).

Before motion: GP lock + teach-pendant program running (solid green). If GJ probe
shows motion=0 for both arms, try ``--methods="['movej']"`` or reload the teleop
program on the pendants.

Examples:

```shell
# First smoke: episode 0, safety clip 5 deg/joint/tick
lerobot-crp-replay-dual \\
    --dataset.repo_id=user/20260615_gyl_1 \\
    --dataset.episode=0 \\
    --max_joint_delta_deg=5

# If arms still look frozen, disable per-tick clip and slow the loop
lerobot-crp-replay-dual \\
    --dataset.repo_id=user/20260615_gyl_1 \\
    --dataset.episode=0 \\
    --max_joint_delta_deg=null \\
    --dataset.fps=4

# Merged dataset after training merge
lerobot-crp-replay-dual \\
    --dataset.repo_id=user/20260615_gyl_merged \\
    --dataset.episode=0
```

Stop ``lerobot-crp-tele-dual`` / ``lerobot-crp-record-dual`` before replay.
"""

import logging
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

from lerobot.configs import parser
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.sdk import exit_process_after_dual_disconnect
from lerobot.scripts.crp_gp.action_executor import (
    CrpDualGjExecutor,
    CrpDualGjExecutorConfig,
    RightJointBackend,
    action_array_to_dict,
    read_dual_joint_snapshot,
    validate_dual_action_keys,
)
from lerobot.scripts.crp_gp.config import TeleoperateDualCRPConfig, _default_robot_for_teleop
from lerobot.scripts.crp_gp.loop import lock_gps_to_current_tcp, prepare_dual_gp_session
from lerobot.scripts.crp_gp.replay import replay_episode_crp_dual
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging, log_say

logger = logging.getLogger(__name__)


@dataclass
class CrpDatasetReplayConfig:
    repo_id: str = "user/20260615_gyl_1"
    episode: int = 0
    root: str | Path | None = None
    # None = use dataset meta fps (typically 16 for CRP dual record).
    fps: float | None = None


@dataclass
class ReplayDualCRPConfig:
    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_teleop)
    dataset: CrpDatasetReplayConfig = field(default_factory=CrpDatasetReplayConfig)
    gj_register_left: int = 10
    gj_register_right: int = 20
    gj_trajectory_group_size: int = 5
    max_joint_delta_deg: float | None = 5.0
    speed_ratio: int | None = 20
    gp_lock_on_exit: bool = True
    gp_lock_on_start: bool = False
    right_joint_backend: str = "auto"
    gp_tcp_step_mm: float = 12.0
    gp_tcp_step_deg: float = 6.0
    gp_send_fps: int = 50
    gp_align_delay_s: float = 0.0
    gp_joint_deadband_deg: float = 0.5
    play_sounds: bool = True


def _resolve_fps(dataset: LeRobotDataset, cfg_fps: float | None) -> float:
    if cfg_fps is not None:
        return float(cfg_fps)
    return float(dataset.meta.fps)


@parser.wrap()
def replay_dual_crp(cfg: ReplayDualCRPConfig) -> None:
    init_logging()
    logger.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, CRPArmDual):
        raise TypeError(f"Expected CRPArmDual, got {type(robot)}")

    dataset = LeRobotDataset(
        cfg.dataset.repo_id,
        root=cfg.dataset.root,
        episodes=[cfg.dataset.episode],
    )
    fps = _resolve_fps(dataset, cfg.dataset.fps)

    executor_cfg = CrpDualGjExecutorConfig(
        gj_register_left=cfg.gj_register_left,
        gj_register_right=cfg.gj_register_right,
        gj_trajectory_group_size=cfg.gj_trajectory_group_size,
        max_joint_delta_deg=cfg.max_joint_delta_deg,
        right_joint_backend=RightJointBackend(cfg.right_joint_backend),
        gp_tcp_step_mm=cfg.gp_tcp_step_mm,
        gp_tcp_step_deg=cfg.gp_tcp_step_deg,
        gp_send_fps=cfg.gp_send_fps,
        gp_align_delay_s=cfg.gp_align_delay_s,
        gp_joint_deadband_deg=cfg.gp_joint_deadband_deg,
    )
    executor = CrpDualGjExecutor(robot, executor_cfg)
    gp_stream = executor.uses_dual_gp_stream()
    gp_lock = cfg.gp_lock_on_start and not gp_stream

    exit_code = 0
    try:
        robot.connect(calibrate=False)
        tele_cfg = TeleoperateDualCRPConfig(robot=cfg.robot)
        prepare_dual_gp_session(
            robot, tele_cfg, ui_probe=None, lock_gp=gp_lock
        )

        if gp_stream:
            executor.prepare_gp_replay_session(tele_cfg)

        crp = robot.crp_arm_robot
        if cfg.speed_ratio is not None and hasattr(crp, "set_speed_ratio"):
            crp.set_speed_ratio(int(cfg.speed_ratio))
            logger.info("speed_ratio set to %s", cfg.speed_ratio)

        log_say(f"Replaying episode {cfg.dataset.episode}", cfg.play_sounds, blocking=True)
        result = replay_episode_crp_dual(
            robot,
            dataset,
            executor,
            fps=fps,
            gp_send_fps=cfg.gp_send_fps if gp_stream else None,
        )
        if result.aborted:
            exit_code = 1
    finally:
        if robot.is_connected and cfg.gp_lock_on_exit:
            try:
                lock_gps_to_current_tcp(robot, TeleoperateDualCRPConfig(robot=cfg.robot))
            except Exception as exc:
                logger.warning("GP lock on exit failed: %s", exc)
        try:
            robot.disconnect()
        except Exception as exc:
            logger.warning("disconnect: %s", exc)
        exit_process_after_dual_disconnect(exit_code)

    if exit_code != 0:
        logger.error("Replay aborted.")
    else:
        logger.info("Replay completed successfully.")
    sys.exit(exit_code)


def main() -> None:
    register_third_party_plugins()
    replay_dual_crp()


if __name__ == "__main__":
    main()
