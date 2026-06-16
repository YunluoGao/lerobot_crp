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
Deploy a trained policy on dual CRP arms (GJ10 + ``set_GJs_second`` GJ20).

Left/right joint + ui50 commands go through ``CrpDualGjExecutor`` (same path as
``lerobot-crp-replay-dual``). Observations: CRP joints, gripper UI56–58, and the
cameras configured on ``--robot.cameras`` (must match training).

Examples:

```shell
# Safe first run (slow, clipped, 60s cap)
lerobot-crp-deploy-dual \\
    --policy.path=outputs/train/act_20260615_gyl/checkpoints/last/pretrained_model \\
    --single_task=assemble \\
    --fps=4 \\
    --duration_s=60 \\
    --max_joint_delta_deg=3 \\
    --speed_ratio=10 \\
    --gp_align_delay_s=5

# Longer run after smoke test
lerobot-crp-deploy-dual \\
    --policy.path=outputs/train/act_20260615_gyl/checkpoints/last/pretrained_model \\
    --fps=8 \\
    --duration_s=300 \\
    --max_joint_delta_deg=5 \\
    --speed_ratio=15
```

Before motion: teach programs STOP → connect → seed GJ → countdown → press START on both pendants.
Stop ``lerobot-crp-tele-dual`` / ``lerobot-crp-record-dual`` first. Press the exit key to stop early.
"""

import logging
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.sdk import exit_process_after_dual_disconnect
from lerobot.scripts.crp_gp.action_executor import CrpDualGjExecutor, CrpDualGjExecutorConfig, RightJointBackend
from lerobot.scripts.crp_gp.config import (
    TeleoperateDualCRPConfig,
    _default_robot_for_record,
    ensure_crp_dual_default_cameras,
)
from lerobot.scripts.crp_gp.deploy import load_policy_bundle, run_policy_deploy_crp_dual
from lerobot.scripts.crp_gp.loop import lock_gps_to_current_tcp, prepare_dual_gp_session
from lerobot.scripts.crp_gp.record_utils import resolve_orbbec_top_camera
from lerobot.common.control_utils import init_keyboard_listener
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging, log_say

logger = logging.getLogger(__name__)


@dataclass
class DeployDualCRPConfig:
    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_record)
    policy: PreTrainedConfig | None = None
    single_task: str = "assemble"
    rename_map: dict[str, str] = field(default_factory=dict)
    fps: float = 4.0
    duration_s: float | None = 60.0
    gj_register_left: int = 10
    gj_register_right: int = 20
    gj_trajectory_group_size: int = 5
    max_joint_delta_deg: float | None = 3.0
    speed_ratio: int | None = 10
    gp_lock_on_exit: bool = True
    gp_align_delay_s: float = 5.0
    right_joint_backend: str = "auto"
    play_sounds: bool = True

    def __post_init__(self) -> None:
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = Path(policy_path)
        if self.policy is None:
            raise ValueError(
                "Deploy requires --policy.path=outputs/.../checkpoints/last/pretrained_model"
            )
        ensure_crp_dual_default_cameras(self.robot)

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return ["policy"]


@parser.wrap()
def deploy_dual_crp(cfg: DeployDualCRPConfig) -> None:
    init_logging()
    logger.info(pformat(asdict(cfg)))

    assert cfg.policy is not None and cfg.policy.pretrained_path is not None
    policy_path = str(cfg.policy.pretrained_path)

    policy, preprocessor, postprocessor, ds_meta = load_policy_bundle(
        policy_path,
        rename_map=cfg.rename_map,
    )

    resolve_orbbec_top_camera(cfg.robot)
    required_cams = {
        key.removeprefix("observation.images.")
        for key in ds_meta.features
        if key.startswith("observation.images.")
    }
    missing_cams = required_cams - set(cfg.robot.cameras)
    if missing_cams:
        raise ValueError(
            f"Deploy needs camera keys {sorted(required_cams)} on --robot.cameras; missing {sorted(missing_cams)}. "
            "Defaults are restored automatically unless overridden; check RealSense serials / ORBBEC_PATH."
        )

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, CRPArmDual):
        raise TypeError(f"Expected CRPArmDual, got {type(robot)}")

    executor_cfg = CrpDualGjExecutorConfig(
        gj_register_left=cfg.gj_register_left,
        gj_register_right=cfg.gj_register_right,
        gj_trajectory_group_size=cfg.gj_trajectory_group_size,
        max_joint_delta_deg=cfg.max_joint_delta_deg,
        right_joint_backend=RightJointBackend(cfg.right_joint_backend),
        gp_align_delay_s=cfg.gp_align_delay_s,
    )
    executor = CrpDualGjExecutor(robot, executor_cfg)

    exit_code = 0
    listener = None
    events: dict = {}
    try:
        robot.connect(calibrate=False)
        tele_cfg = TeleoperateDualCRPConfig(robot=cfg.robot)
        prepare_dual_gp_session(robot, tele_cfg, ui_probe=None, lock_gp=False)

        crp = robot.crp_arm_robot
        if cfg.speed_ratio is not None and hasattr(crp, "set_speed_ratio"):
            crp.set_speed_ratio(int(cfg.speed_ratio))
            logger.info("speed_ratio set to %s", cfg.speed_ratio)

        log_say("CRP dual policy deploy: press START on teach pendants during countdown.", cfg.play_sounds)
        listener, events = init_keyboard_listener()

        result = run_policy_deploy_crp_dual(
            robot,
            executor,
            policy,
            preprocessor,
            postprocessor,
            ds_meta,
            fps=cfg.fps,
            duration_s=cfg.duration_s,
            single_task=cfg.single_task,
            events=events,
        )
        if result.aborted:
            exit_code = 1
        elif result.ticks == 0:
            logger.error("Deploy exited with 0 policy ticks (check logs above for inference/obs errors).")
            exit_code = 1
    except Exception as exc:
        logger.exception("Deploy failed: %s", exc)
        exit_code = 1
    finally:
        if listener is not None:
            listener.stop()
        if robot.is_connected and cfg.gp_lock_on_exit:
            try:
                lock_gps_to_current_tcp(robot, TeleoperateDualCRPConfig(robot=cfg.robot))
            except Exception as exc:
                logger.warning("GP lock on exit failed: %s", exc)
        try:
            robot.disconnect()
        except Exception as exc:
            logger.warning("disconnect: %s", exc)
        if exit_code != 0:
            logger.error("Deploy aborted.")
        else:
            logger.info("Deploy completed.")
        exit_process_after_dual_disconnect(exit_code)

    sys.exit(exit_code)


def main() -> None:
    register_third_party_plugins()
    deploy_dual_crp()


if __name__ == "__main__":
    main()
