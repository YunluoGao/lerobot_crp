"""Discover Orbbec Gemini 335 RGB V4L2 node among many /dev/video* aliases.

Gemini 335 exposes depth/IR/metadata/RGB as separate nodes. IR streams often
return 640x400 with very high Laplacian variance (dot pattern). RGB is typically
640x480 with moderate sharpness. ``/dev/videoN`` indices change after USB replug.
"""

from __future__ import annotations

import glob
import os
import pickle
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

ORBBEC_PROFILE = ("640x480 RGB (record default)", "YUYV", 640, 480, 8)


@dataclass(frozen=True)
class OrbbecCaptureResult:
    path: str
    frame: np.ndarray
    reported: str
    sharpness: float
    score: int
    profile_name: str


def find_orbbec_capture_paths(*, prefer: str | None = None) -> list[str]:
    """Return candidate V4L2 paths for Orbbec capture nodes (deduped, ordered)."""
    paths: list[str] = []
    if prefer and Path(prefer).exists():
        paths.append(prefer)
    env_path = os.environ.get("ORBBEC_PATH", "")
    if env_path and Path(env_path).exists() and env_path not in paths:
        paths.append(env_path)
    for pattern in (
        "/dev/v4l/by-path/*Orbbec*",
        "/dev/v4l/by-id/usb-Orbbec_*",
    ):
        paths.extend(sorted(glob.glob(pattern)))
    for dev in sorted(glob.glob("/dev/video*"), key=lambda p: int(p.replace("/dev/video", "") or "0")):
        try:
            out = subprocess.check_output(
                ["udevadm", "info", "-q", "property", "-n", dev],
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.CalledProcessError):
            continue
        if "Orbbec" not in out and "Gemini" not in out:
            continue
        if ":capture:" not in out:
            continue
        paths.append(dev)
    seen: set[str] = set()
    unique: list[str] = []
    for path in paths:
        if path not in seen:
            seen.add(path)
            unique.append(path)
    return unique


def reset_orbbec_v4l2(path: str, *, width: int = 640, height: int = 480, pixelformat: str = "YUYV") -> None:
    if not Path("/usr/bin/v4l2-ctl").exists() and not Path("/bin/v4l2-ctl").exists():
        return
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            path,
            f"--set-fmt-video=width={width},height={height},pixelformat={pixelformat}",
        ],
        capture_output=True,
        text=True,
        timeout=3,
        check=False,
    )


def apply_orbbec_v4l2_tuning(path: str) -> None:
    """Apply lab Orbbec format + manual exposure (same defaults as setup_orbbec_top_v4l2.sh)."""
    if not Path("/usr/bin/v4l2-ctl").exists() and not Path("/bin/v4l2-ctl").exists():
        reset_orbbec_v4l2(path)
        return
    exposure = os.environ.get("ORBBEC_EXPOSURE", "100")
    gain = os.environ.get("ORBBEC_GAIN", "4")
    brightness = os.environ.get("ORBBEC_BRIGHTNESS", "0")
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            path,
            "--set-fmt-video=width=640,height=480,pixelformat=YUYV",
            "--set-ctrl=auto_exposure=1",
            f"--set-ctrl=exposure_time_absolute={exposure}",
            f"--set-ctrl=gain={gain}",
            f"--set-ctrl=brightness={brightness}",
        ],
        capture_output=True,
        text=True,
        timeout=3,
        check=False,
    )


def score_orbbec_rgb_frame(frame: np.ndarray) -> tuple[int, float]:
    """Higher score = more likely workspace RGB (not IR dot pattern)."""
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    score = 0
    if height == 480 and width == 640:
        score += 100
    if height == 400 and width == 640:
        score -= 80
    if sharpness > 20_000:
        score -= 120
    elif 500 <= sharpness <= 15_000:
        score += 40
    elif sharpness < 300:
        score -= 40
    if float(frame.mean()) < 5:
        score -= 100
    return score, sharpness


def _capture_profile(
    path: str,
    profile_name: str,
    fourcc: str | None,
    width: int | None,
    height: int | None,
    timeout_s: int,
):
    if profile_name.startswith("640"):
        reset_orbbec_v4l2(path, width=width or 640, height=height or 480)
    try:
        proc = subprocess.run(
            [
                sys.executable,
                "-c",
                (
                    "import cv2, pickle, sys\n"
                    "path, fourcc, w, h = sys.argv[1:5]\n"
                    "w = None if w == '-' else int(w)\n"
                    "h = None if h == '-' else int(h)\n"
                    "cap = cv2.VideoCapture(path)\n"
                    "if not cap.isOpened(): raise SystemExit(1)\n"
                    "if fourcc != '-': cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))\n"
                    "if w is not None:\n"
                    "    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, 25)\n"
                    "cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)\n"
                    "frame = None\n"
                    "for _ in range(25):\n"
                    "    ok, f = cap.read()\n"
                    "    if ok and f is not None:\n"
                    "        frame = f; break\n"
                    "reported = f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'\n"
                    "cap.release()\n"
                    "if frame is None: raise SystemExit(2)\n"
                    "pickle.dump((frame, reported), sys.stdout.buffer)\n"
                ),
                path,
                fourcc or "-",
                str(width) if width is not None else "-",
                str(height) if height is not None else "-",
            ],
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return None
    if proc.returncode != 0:
        return None
    return pickle.loads(proc.stdout)


def capture_best_orbbec_rgb(*, prefer: str | None = None) -> OrbbecCaptureResult | None:
    """Try all Orbbec nodes; return the highest-scoring RGB-like capture."""
    profile_name, fourcc, width, height, timeout_s = ORBBEC_PROFILE
    best: OrbbecCaptureResult | None = None
    for path in find_orbbec_capture_paths(prefer=prefer):
        captured = _capture_profile(path, profile_name, fourcc, width, height, timeout_s)
        if captured is None:
            continue
        frame, reported = captured
        score, sharpness = score_orbbec_rgb_frame(frame)
        candidate = OrbbecCaptureResult(
            path=path,
            frame=frame,
            reported=reported,
            sharpness=sharpness,
            score=score,
            profile_name=profile_name,
        )
        if best is None or candidate.score > best.score:
            best = candidate
    if best is None or best.score < 0:
        return None
    return best
