from __future__ import annotations

import pytest

from lerobot.scripts.crp_gp.gj_probe import (
    GjProbeConfig,
    GjProbeReport,
    JointSnapshot,
    JointWriteMethod,
    ProbeAttemptResult,
    build_step_target,
    clamp_joints_toward,
    cross_arm_delta,
    joint_tracking_error,
    probe_exit_code,
    resolve_probe_targets,
)


def test_clamp_joints_toward_limits_delta() -> None:
    current = [0.0, 10.0, -5.0, 0.0, 0.0, 0.0]
    target = [20.0, 10.0, -20.0, 0.0, 0.0, 0.0]
    clamped = clamp_joints_toward(current, target, max_delta_deg=5.0)
    assert clamped == [5.0, 10.0, -10.0, 0.0, 0.0, 0.0]


def test_resolve_probe_targets_arms_right_overrides_step_arm() -> None:
    snap = JointSnapshot(left=[1, 2, 3, 4, 5, 6], right=[10, 20, 30, 40, 50, 60])
    cfg = GjProbeConfig(arms="right", step_arm="left", step_joint=1, step_delta_deg=2.0)
    left, right = resolve_probe_targets(cfg, snap, None)
    assert left is None
    assert right == [12.0, 20, 30, 40, 50, 60]


def test_build_step_target_left_j1() -> None:
    snap = JointSnapshot(left=[1, 2, 3, 4, 5, 6], right=[10, 20, 30, 40, 50, 60])
    left, right = build_step_target(snap, arm="left", joint_index=1, delta_deg=2.5)
    assert left == [3.5, 2, 3, 4, 5, 6]
    assert right is None


def test_cross_arm_delta_detects_uncommanded_motion() -> None:
    before = JointSnapshot(left=[0, 0, 0, 0, 0, 0], right=[1, 1, 1, 1, 1, 1])
    after = JointSnapshot(left=[1, 0, 0, 0, 0, 0], right=[1, 2, 1, 1, 1, 1])
    assert cross_arm_delta(before, after, commanded_arm="left") == 1.0


def test_joint_tracking_error() -> None:
    max_err, per = joint_tracking_error([0, 0, 0, 0, 0, 0], [1, 2, 0, 0, 0, 0])
    assert max_err == 2.0
    assert per == [1.0, 2.0, 0.0, 0.0, 0.0, 0.0]


def test_probe_exit_code_requires_motion() -> None:
    attempt = ProbeAttemptResult(
        arm="right",
        method=JointWriteMethod.SET_GJS,
        gj_register=20,
        command_joints=[0] * 6,
        joints_before=JointSnapshot([0] * 6, [0] * 6),
        joints_after=JointSnapshot([0] * 6, [0] * 6),
        max_error_deg=1.0,
        per_joint_error_deg=[1.0] + [0.0] * 5,
        cross_arm_max_delta_deg=0.0,
        physical_motion_deg=0.0,
        sdk_call_ok=True,
        message="false pass",
    )
    report = GjProbeReport(
        sdk_methods={"set_GJs": True},
        mode="step",
        target_left=None,
        target_right=[1, 0, 0, 0, 0, 0],
        attempts=[attempt],
        ui50_ok=None,
    )
    assert probe_exit_code(report) == 1


def test_probe_exit_code_ok_when_arm_moves() -> None:
    attempt = ProbeAttemptResult(
        arm="left",
        method=JointWriteMethod.SET_GJS,
        gj_register=10,
        command_joints=[1, 0, 0, 0, 0, 0],
        joints_before=JointSnapshot([0] * 6, [0] * 6),
        joints_after=JointSnapshot([1, 0, 0, 0, 0, 0], [0] * 6),
        max_error_deg=0.5,
        per_joint_error_deg=[0.5] + [0.0] * 5,
        cross_arm_max_delta_deg=0.0,
        physical_motion_deg=1.0,
        sdk_call_ok=True,
        message="ok",
    )
    report = GjProbeReport(
        sdk_methods={"set_GJs": True},
        mode="step",
        target_left=[1, 0, 0, 0, 0, 0],
        target_right=None,
        attempts=[attempt],
        ui50_ok=True,
    )
    assert probe_exit_code(report) == 0
