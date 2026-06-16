from __future__ import annotations

from lerobot.scripts.crp_gp.action_executor import (
    action_array_to_dict,
    clamp_gp_toward,
    clamp_joints_toward,
    extract_arm_joints,
    joint_tracking_error,
    joints_to_gp_from_baseline,
    joints_to_gp_incremental,
    validate_dual_action_keys,
)


def test_joints_to_gp_from_baseline() -> None:
    baseline_j = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0]
    baseline_tcp = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
    cmd_j = [10.0, 90.0, 0.0, 0.0, 0.0, 0.0]
    gp = joints_to_gp_from_baseline(baseline_j, baseline_tcp, cmd_j)
    assert gp[0] == 135.0
    assert gp[1] == 200.0


def test_joints_to_gp_incremental() -> None:
    tcp = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
    current_j = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0]
    target_j = [2.0, 90.0, 0.0, 0.0, 0.0, 0.0]
    gp = joints_to_gp_incremental(tcp, current_j, target_j)
    assert gp[0] == 107.0
    assert gp[1] == 200.0


def test_clamp_gp_toward() -> None:
    cur = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tgt = [100.0, 0.0, 0.0, 20.0, 0.0, 0.0]
    out = clamp_gp_toward(cur, tgt, step_mm=12.0, step_deg=6.0)
    assert out == [12.0, 0.0, 0.0, 6.0, 0.0, 0.0]


def test_action_array_to_dict() -> None:
    names = ["left_j1.pos", "right_j1.pos"]
    action = action_array_to_dict(names, [1.5, 2.5])
    assert action == {"left_j1.pos": 1.5, "right_j1.pos": 2.5}


def test_extract_arm_joints() -> None:
    action = {f"left_j{i}.pos": float(i) for i in range(1, 7)}
    action.update({f"right_j{i}.pos": float(10 + i) for i in range(1, 7)})
    assert extract_arm_joints(action, "left") == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_validate_dual_action_keys_ok() -> None:
    names = [f"left_j{i}.pos" for i in range(1, 7)] + [f"right_j{i}.pos" for i in range(1, 7)]
    names += ["left_ui50", "right_ui50"]
    validate_dual_action_keys(names)


def test_joint_tracking_error() -> None:
    assert joint_tracking_error([0, 0, 0, 0, 0, 0], [1, 3, 0, 0, 0, 0]) == 3.0


def test_clamp_replay_safety() -> None:
    current = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    target = [10.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    assert clamp_joints_toward(current, target, 2.0) == [2.0, 10.0, 0.0, 0.0, 0.0, 0.0]
