#!/usr/bin/env python

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
CLI / config for dual CRP over one SDK session (``robot.type=crp_arm_dual``).

``ip1`` / ``ip2``: left and right controller addresses.
Used by ``lerobot-crp-tele-dual`` and ``lerobot-crp-record-dual``.
"""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("crp_arm_dual")
@dataclass
class CRPArmDualConfig(RobotConfig):
    """
    Dual CRP over one CrpRobotPy session: ``connect`` + ``connect_second``; GP via
    ``set_GPs`` / ``set_GPs_second``.
    """

    ip1: str = "192.168.0.100"  # left / set_GPs
    ip2: str = "192.168.0.101"  # right / set_GPs_second
    retry_times: int = 3
    connect_settle_s: float = 0.25  # pause before connect_second (s)
    # When True, ``connect()`` skips ``servo_power_on`` (single enable after UI probe subprocess).
    defer_servo_power_on: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    # Retries after transient ``cam.read()`` failures (same idea as ``crp_arm``).
    camera_read_retries: int = 2
