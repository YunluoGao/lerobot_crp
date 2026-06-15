from __future__ import annotations

import numpy as np
import pytest

from lerobot.scripts.crp_gp.orbbec_rgb_discovery import (
    assert_orbbec_rgb_frame,
    detect_orbbec_frame_glitch,
    score_orbbec_rgb_frame,
)


def _natural_workspace_frame() -> np.ndarray:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :, 0] = 120
    frame[:, :, 1] = 100
    frame[:, :, 2] = 80
    y, x = np.mgrid[0:480, 0:640]
    frame[:, :, 0] = np.clip(frame[:, :, 0] + x // 20, 0, 255).astype(np.uint8)
    frame[:, :, 1] = np.clip(frame[:, :, 1] + y // 25, 0, 255).astype(np.uint8)
    return frame


def _glitch_frame() -> np.ndarray:
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[50:430, 80:280, 1] = 230
    frame[50:430, 80:280, 0] = 20
    frame[50:430, 80:280, 2] = 30
    frame[120:400, 360:560, 0] = 180
    frame[120:400, 360:560, 2] = 200
    frame[120:400, 360:560, 1] = 40
    return frame


def test_detect_orbbec_frame_glitch_rejects_synthetic_glitch() -> None:
    reason = detect_orbbec_frame_glitch(_glitch_frame())
    assert reason is not None
    assert "glitch" in reason


def test_detect_orbbec_frame_glitch_accepts_natural_frame() -> None:
    assert detect_orbbec_frame_glitch(_natural_workspace_frame()) is None


def test_score_orbbec_rgb_frame_penalizes_glitch() -> None:
    good_score, _ = score_orbbec_rgb_frame(_natural_workspace_frame())
    bad_score, _ = score_orbbec_rgb_frame(_glitch_frame())
    assert good_score > bad_score
    assert bad_score < 0


def test_detect_orbbec_frame_glitch_rejects_ir_dots() -> None:
    frame = np.full((480, 640, 3), 128, dtype=np.uint8)
    rng = np.random.default_rng(0)
    noise = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    frame[:, :, 0] = noise
    frame[:, :, 1] = noise
    frame[:, :, 2] = noise
    reason = detect_orbbec_frame_glitch(frame)
    assert reason is not None
    assert "IR" in reason


def test_detect_orbbec_frame_glitch_rejects_grey_ir_stream() -> None:
    """Gemini index2 GREY via OpenCV: BGR channels identical (channel_std ~ 0)."""
    grey = np.full((480, 640), 60, dtype=np.uint8)
    frame = np.stack([grey, grey, grey], axis=2)
    reason = detect_orbbec_frame_glitch(frame)
    assert reason is not None
    assert "greyscale/IR" in reason


def test_assert_orbbec_rgb_frame_raises_on_glitch() -> None:
    with pytest.raises(ConnectionError, match="glitch"):
        assert_orbbec_rgb_frame(_glitch_frame(), path="/dev/video6")


def test_assert_orbbec_rgb_frame_passes_natural_frame() -> None:
    assert_orbbec_rgb_frame(_natural_workspace_frame(), path="/dev/video6")
