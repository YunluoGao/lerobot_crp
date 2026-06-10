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
Dual SO leader teleoperation for dual CRP arms (thin CLI wrapper).

Core reusable logic lives in:
  - ``lerobot.scripts.crp_gp.config``
  - ``lerobot.scripts.crp_gp.loop``
  - ``lerobot.scripts.crp_gp.mappers``
  - ``lerobot.scripts.crp_gp.ui_probe``
"""

import logging
from dataclasses import asdict
from pprint import pformat

from lerobot.configs import parser
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots import make_robot_from_config
from lerobot.scripts.crp_gp.config import TeleoperateDualCRPConfig
from lerobot.scripts.crp_gp.loop import TARGET_FRAME_NAME, lock_gps_to_current_tcp, teleop_loop_crp_dual
from lerobot.scripts.crp_gp.mappers.so101_urdf import SO101_ARM_MOTOR_NAMES, resolve_so101_urdf_path
from lerobot.scripts.crp_gp.ui_probe import GripperUiProbeManager, apply_ui_probe_connect_policy
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.robots.crp_arm_dual.sdk import exit_process_after_dual_disconnect
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)


@parser.wrap()
def teleoperate_dual(cfg: TeleoperateDualCRPConfig) -> None:
    init_logging()
    logger.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    ui_probe: GripperUiProbeManager | None = None
    if cfg.gripper_ui_probe:
        ui_probe = GripperUiProbeManager(hz=cfg.gripper_ui_probe_hz)
    apply_ui_probe_connect_policy(robot, ui_probe)

    teleop.connect()
    try:
        robot.connect()
    except Exception:
        teleop.disconnect()
        raise

    urdf_path = resolve_so101_urdf_path()
    kinematics = RobotKinematics(
        urdf_path=urdf_path,
        target_frame_name=TARGET_FRAME_NAME,
        joint_names=list(SO101_ARM_MOTOR_NAMES),
    )
    logger.debug("URDF FK path=%s frame=%s", urdf_path, TARGET_FRAME_NAME)

    try:
        teleop_loop_crp_dual(teleop, robot, kinematics, cfg, ui_probe=ui_probe)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, stopping dual teleoperation.")
    finally:
        if ui_probe is not None:
            ui_probe.stop_all()
        try:
            lock_gps_to_current_tcp(robot, cfg)
        except Exception as exc:
            logger.warning("GP lock on exit failed: %s", exc)
        robot.disconnect()
        teleop.disconnect()
        exit_process_after_dual_disconnect()


def main() -> None:
    register_third_party_plugins()
    teleoperate_dual()


if __name__ == "__main__":
    main()

