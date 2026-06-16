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
Phase-0 probe: dual CRP joint direct control (direction A — set_GJs / movej).

Safe defaults move one arm joint by 1° from the current pose. GP registers are
locked to the current TCP before any joint write. No cameras are opened.

Examples:

```shell
# Safest smoke: left j1 +1°, try set_GJs then movej, no motion if --dry_run=true
lerobot-crp-gj-probe-dual --dry_run=true

# Left arm only, set_GJs only
lerobot-crp-gj-probe-dual --arms=left --methods="['set_gjs']"

# Small step toward dataset frame 0 (clamped to 5° per joint)
lerobot-crp-gj-probe-dual \\
    --mode=dataset \\
    --dataset.repo_id=user/20260615_gyl_1 \\
    --frame_index=0 \\
    --max_joint_delta_deg=5

# Read joints + SDK surface only
lerobot-crp-gj-probe-dual --mode=hold --methods="[]" --test_ui50=false
```

Prerequisites: CRP network, CrpRobotPy with ``set_GJs_second``, servo enable on teach pendants.
Stop ``lerobot-crp-tele-dual`` / ``lerobot-crp-record-dual`` before running.
"""

import logging
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

from lerobot.configs import parser
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.sdk import exit_process_after_dual_disconnect
from lerobot.scripts.crp_gp.config import _default_robot_for_teleop
from lerobot.scripts.crp_gp.gj_probe import (
    GjProbeConfig,
    JointWriteMethod,
    probe_exit_code,
    run_gj_probe,
)
from lerobot.scripts.crp_gp.loop import lock_gps_to_current_tcp
from lerobot.scripts.crp_gp.config import TeleoperateDualCRPConfig
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)


def _parse_methods(raw: list[str] | None) -> tuple[JointWriteMethod, ...]:
    if not raw:
        return ()
    out: list[JointWriteMethod] = []
    for item in raw:
        key = item.strip().lower().replace("-", "_")
        try:
            out.append(JointWriteMethod(key))
        except ValueError as exc:
            raise ValueError(
                f"Unknown probe method {item!r}; use set_gjs and/or movej"
            ) from exc
    return tuple(out)


@dataclass
class DatasetProbeConfig:
    repo_id: str = "user/20260615_gyl_1"
    root: str | Path | None = None
    frame_index: int = 0


@dataclass
class CrpGjProbeDualConfig:
    robot: CRPArmDualConfig = field(default_factory=_default_robot_for_teleop)
    dataset: DatasetProbeConfig = field(default_factory=DatasetProbeConfig)
    mode: str = "step"
    step_arm: str = "left"
    step_joint: int = 1
    step_delta_deg: float = 1.0
    max_joint_delta_deg: float = 5.0
    arms: str = "both"
    methods: list[str] = field(default_factory=lambda: ["set_gjs", "movej"])
    gj_register_left: int = 10
    gj_register_right: int = 20
    gj_trajectory_group_size: int = 5
    settle_s: float = 1.0
    dry_run: bool = False
    test_ui50: bool = True
    ui50_pulse_delta: int = 15
    speed_ratio: int | None = 20
    gp_lock_on_exit: bool = True
    gp_align_delay_s: float = 5.0


@parser.wrap()
def crp_gj_probe_dual(cfg: CrpGjProbeDualConfig) -> None:
    init_logging()
    logger.info(pformat(asdict(cfg)))

    if cfg.mode not in ("hold", "step", "dataset"):
        raise ValueError(f"mode must be hold|step|dataset, got {cfg.mode!r}")
    if cfg.arms not in ("left", "right", "both"):
        raise ValueError(f"arms must be left|right|both, got {cfg.arms!r}")
    if cfg.step_arm not in ("left", "right"):
        raise ValueError(f"step_arm must be left|right, got {cfg.step_arm!r}")

    probe_cfg = GjProbeConfig(
        robot=cfg.robot,
        dataset_repo_id=cfg.dataset.repo_id,
        dataset_root=cfg.dataset.root,
        frame_index=cfg.dataset.frame_index,
        mode=cfg.mode,  # type: ignore[arg-type]
        step_arm=cfg.step_arm,  # type: ignore[arg-type]
        step_joint=cfg.step_joint,
        step_delta_deg=cfg.step_delta_deg,
        max_joint_delta_deg=cfg.max_joint_delta_deg,
        arms=cfg.arms,  # type: ignore[arg-type]
        methods=_parse_methods(cfg.methods),
        gj_register_left=cfg.gj_register_left,
        gj_register_right=cfg.gj_register_right,
        gj_trajectory_group_size=cfg.gj_trajectory_group_size,
        settle_s=cfg.settle_s,
        dry_run=cfg.dry_run,
        test_ui50=cfg.test_ui50,
        ui50_pulse_delta=cfg.ui50_pulse_delta,
        speed_ratio=cfg.speed_ratio,
        gp_lock_on_exit=cfg.gp_lock_on_exit,
        gp_align_delay_s=cfg.gp_align_delay_s,
    )

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, CRPArmDual):
        raise TypeError(f"Expected CRPArmDual, got {type(robot)}")

    # Seed runs inside run_gj_probe; avoid connect-time set_GJs_second before teach START.
    robot.config.init_gj_on_connect = False

    exit_code = 1
    try:
        try:
            robot.connect(calibrate=False)
        except Exception as exc:
            logger.error(
                "Dual connect failed (check ip1/ip2 power, e-stop, teach pendant servo enable): %s",
                exc,
            )
            raise
        report = run_gj_probe(probe_cfg, robot)
        for line in report.summary_lines():
            logger.info(line)
        exit_code = probe_exit_code(report)
    finally:
        if robot.is_connected and cfg.gp_lock_on_exit:
            try:
                tele_cfg = TeleoperateDualCRPConfig(robot=cfg.robot)
                lock_gps_to_current_tcp(robot, tele_cfg)
            except Exception as exc:
                logger.warning("GP lock on exit failed: %s", exc)
        try:
            robot.disconnect()
        except Exception as exc:
            logger.warning("disconnect: %s", exc)
        exit_process_after_dual_disconnect(exit_code)

    if exit_code != 0:
        logger.error("GJ probe finished with failures — review summary above.")
    else:
        logger.info("GJ probe finished OK (at least one joint write path succeeded).")
    sys.exit(exit_code)


def main() -> None:
    register_third_party_plugins()
    crp_gj_probe_dual()


if __name__ == "__main__":
    main()
