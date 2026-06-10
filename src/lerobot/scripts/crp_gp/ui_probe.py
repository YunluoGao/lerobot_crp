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

"""Gripper UI polling via ``crp_getui_probe`` subprocess (one per arm)."""

from __future__ import annotations

import json
import logging
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from lerobot.robots.crp_arm_dual.crp_ui import (
    GRIPPER_UI_OPEN,
    GRIPPER_UI_POSITION,
    GRIPPER_UI_SPEED_OUT,
    GRIPPER_UI_TORQUE_OUT,
)
from lerobot.robots.crp_arm_dual.getui_probe.paths import DEFAULT_GETUI_PROBE_BINARY

if TYPE_CHECKING:
    from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual

logger = logging.getLogger(__name__)

_DEFAULT_UI_INDICES = (
    GRIPPER_UI_POSITION,
    GRIPPER_UI_SPEED_OUT,
    GRIPPER_UI_TORQUE_OUT,
)
_UI_KEY_RE = re.compile(r"^ui(\d+)$")


def default_ui_probe_binary() -> Path:
    return DEFAULT_GETUI_PROBE_BINARY


def parse_probe_json_line(line: str) -> dict[str, object] | None:
    line = line.strip()
    if not line:
        return None
    try:
        payload = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None
    return payload


def ui_values_from_probe_payload(payload: dict[str, object]) -> dict[int, int]:
    out: dict[int, int] = {}
    for key, raw in payload.items():
        if not isinstance(key, str):
            continue
        m = _UI_KEY_RE.match(key)
        if not m or raw is None:
            continue
        ok_key = f"ui{m.group(1)}_ok"
        if ok_key in payload and payload[ok_key] is False:
            continue
        try:
            out[int(m.group(1))] = int(raw)
        except (TypeError, ValueError):
            continue
    return out


def format_probe_ui_for_log(
    latest: dict[int, int] | None,
    indices: tuple[int, ...] = _DEFAULT_UI_INDICES,
) -> str:
    if not latest:
        return "-"
    parts: list[str] = []
    for idx in indices:
        parts.append(str(latest[idx]) if idx in latest else "?")
    return ",".join(parts)


def apply_ui_probe_connect_policy(
    robot: CRPArmDual,
    ui_probe: GripperUiProbeManager | None,
) -> None:
    """Skip ``servo_power_on`` in ``connect()``; probe subprocess connects first."""
    if ui_probe is not None and ui_probe.enabled:
        robot.config.defer_servo_power_on = True


@dataclass
class _SubprocessArmProbe:
    label: str
    ip: str
    process: subprocess.Popen[str] | None = None
    reader_thread: threading.Thread | None = None
    latest: dict[int, int] = field(default_factory=dict)
    _stop_event: threading.Event = field(default_factory=threading.Event)

    def _reader_loop(self) -> None:
        proc = self.process
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            if self._stop_event.is_set():
                break
            payload = parse_probe_json_line(line)
            if payload is None:
                continue
            values = ui_values_from_probe_payload(payload)
            if values:
                self.latest = values

    def start(
        self,
        binary: Path,
        cwd: Path,
        *,
        hz: float,
        indices: str = "50,56,57,58",
    ) -> None:
        if self.process is not None:
            return
        cmd = [
            str(binary),
            self.ip,
            "--json",
            "--hz",
            str(hz),
            "--indices",
            indices,
            "--arm-label",
            self.label,
        ]
        self._stop_event.clear()
        self.process = subprocess.Popen(
            cmd,
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"ui-probe-{self.label}",
            daemon=True,
        )
        self.reader_thread.start()
        logger.info("UI probe started arm=%s ip=%s hz=%.2f", self.label, self.ip, hz)

    def stop(self, timeout_s: float = 3.0) -> None:
        self._stop_event.set()
        proc = self.process
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=timeout_s)
        stderr_tail = ""
        if proc.stderr is not None:
            try:
                stderr_tail = proc.stderr.read() or ""
            except Exception:
                pass
        if proc.returncode not in (0, None, -15, -9) and stderr_tail.strip():
            logger.debug("UI probe arm=%s exit=%s stderr=%s", self.label, proc.returncode, stderr_tail[-500:])
        self.process = None
        self.reader_thread = None


class GripperUiProbeManager:
    """Poll gripper UI via ``crp_getui_probe`` without blocking the teleop SDK session."""

    def __init__(
        self,
        *,
        hz: float = 2.0,
        indices: tuple[int, ...] = _DEFAULT_UI_INDICES,
    ) -> None:
        self.hz = hz
        self.indices = indices
        self.indices_arg = ",".join(str(i) for i in indices)
        self._binary = default_ui_probe_binary()
        self._cwd = self._binary.parent
        self._arms: dict[str, _SubprocessArmProbe] = {}
        self.enabled = self._binary.is_file() and self._cwd.joinpath("libRobotService.so").is_file()
        if not self.enabled:
            logger.warning(
                "gripper_ui_probe disabled: run "
                "bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh"
            )

    def start_dual(self, ip_left: str, ip_right: str) -> None:
        if not self.enabled:
            return
        self.start_arm("left", ip_left)
        self.start_arm("right", ip_right)

    def start_arm(self, label: str, ip: str) -> None:
        if not self.enabled:
            return
        existing = self._arms.get(label)
        if (
            existing is not None
            and existing.process is not None
            and existing.process.poll() is None
        ):
            return
        if existing is not None:
            existing.stop()
        arm = _SubprocessArmProbe(label=label, ip=ip)
        try:
            arm.start(self._binary, self._cwd, hz=self.hz, indices=self.indices_arg)
        except OSError as exc:
            logger.warning("UI probe failed to start arm=%s: %s", label, exc)
            return
        self._arms[label] = arm

    def latest(self, label: str) -> dict[int, int] | None:
        arm = self._arms.get(label)
        if arm is None or not arm.latest:
            return None
        return dict(arm.latest)

    def wait_for_subprocess_ready(
        self,
        *,
        labels: tuple[str, ...] = ("left", "right"),
        timeout_s: float = 15.0,
        poll_s: float = 0.05,
    ) -> bool:
        """Wait until probes finish connect and emit the first UI sample.

        ``ensure_servo_power_on`` must run *after* probe ``connect()`` — otherwise the
        extra SDK session drops teach-pendant servo enable again.
        """
        if not self.enabled:
            return True
        deadline = time.perf_counter() + timeout_s
        pending = set(labels)
        while pending and time.perf_counter() < deadline:
            for label in list(pending):
                if self.latest(label) is not None:
                    pending.discard(label)
            if not pending:
                logger.info("UI probe subprocess ready (%s)", ", ".join(labels))
                return True
            time.sleep(poll_s)
        logger.warning(
            "UI probe subprocess not ready within %.1fs (missing: %s)",
            timeout_s,
            ", ".join(sorted(pending)),
        )
        return False

    def format_arm_heartbeat_line(
        self,
        prefix: str,
        arm_label: str,
        gp_xyz: tuple[float, float, float],
        *,
        ui50_cmd_fallback: int | None,
    ) -> str:
        """Format `L_GP / L_UI50 / L_POSIT / L_SPEED / L_TORQUE` heartbeat line."""
        values = self.latest(arm_label) if self.enabled else None

        def _field(index: int, fallback: int | None = None) -> str:
            if values is not None and index in values:
                return str(values[index])
            if fallback is not None:
                return str(fallback)
            return "?"

        ui50_raw = _field(GRIPPER_UI_OPEN, ui50_cmd_fallback)
        if values is not None and GRIPPER_UI_OPEN in values:
            ui50 = ui50_raw
        elif ui50_cmd_fallback is not None:
            ui50 = f"{ui50_raw}(cmd)"
        else:
            ui50 = ui50_raw
        posit = _field(GRIPPER_UI_POSITION)
        speed = _field(GRIPPER_UI_SPEED_OUT)
        torque = _field(GRIPPER_UI_TORQUE_OUT)
        return (
            f"\t{prefix}_GP=[{gp_xyz[0]:.0f},{gp_xyz[1]:.0f},{gp_xyz[2]:.0f}] "
            f"{prefix}_UI50={ui50}, {prefix}_POSIT={posit}, {prefix}_SPEED={speed}, "
            f"{prefix}_TORQUE={torque}"
        )

    def stop_all(self) -> None:
        for label in list(self._arms):
            try:
                self._arms[label].stop()
            except Exception as exc:
                logger.warning("UI probe stop arm=%s: %s", label, exc)
        self._arms.clear()
