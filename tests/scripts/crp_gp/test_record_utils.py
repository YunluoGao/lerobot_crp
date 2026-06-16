from __future__ import annotations

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.crp_arm_dual.config_crp_arm_dual import CRPArmDualConfig
from lerobot.scripts.crp_gp.record_utils import resolve_orbbec_top_camera


def test_resolve_orbbec_top_camera_prefers_orbbec_path(monkeypatch, tmp_path) -> None:
    dev = tmp_path / "video7"
    dev.touch()
    monkeypatch.setenv("ORBBEC_PATH", str(dev))

    top = OpenCVCameraConfig(index_or_path="/dev/video6", width=640, height=480, fps=30)
    robot_cfg = CRPArmDualConfig(ip1="127.0.0.1", ip2="127.0.0.1", cameras={"top": top})

    resolve_orbbec_top_camera(robot_cfg)

    assert top.index_or_path == str(dev)
