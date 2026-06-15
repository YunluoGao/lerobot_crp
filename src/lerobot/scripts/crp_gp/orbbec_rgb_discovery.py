"""Discover Orbbec Gemini 335 RGB V4L2 node among many /dev/video* aliases.

Gemini 335 exposes depth/IR/metadata/RGB as separate nodes. IR streams often
return 640x400 with very high Laplacian variance (dot pattern). RGB is typically
640x480 with moderate sharpness. ``/dev/videoN`` indices change after USB replug.
"""

from __future__ import annotations

import glob
import os
import pickle
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

if TYPE_CHECKING:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

ORBBEC_PROFILE = ("640x480 OpenCV default", None, 640, 480, 8)
ORBBEC_PROFILES: tuple[tuple[str, str | None, int, int, int], ...] = (
    ORBBEC_PROFILE,
    ("640x480 YUYV", "YUYV", 640, 480, 8),
    ("640x480 MJPG fallback", "MJPG", 640, 480, 8),
)


@dataclass(frozen=True)
class OrbbecCaptureResult:
    path: str
    frame: np.ndarray
    reported: str
    sharpness: float
    score: int
    profile_name: str
    fourcc: str = "YUYV"


def is_orbbec_v4l2_capture_device(path: str | int) -> bool:
    """True when ``path`` is an Orbbec V4L2 *capture* node (may be IR/RGB/depth)."""
    path_str = str(path)
    if not Path(path_str).exists():
        return False
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", path_str],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        return False
    return ("Orbbec" in out or "Gemini" in out) and ":capture:" in out


_RGB_V4L2_PIXEL_FORMATS = frozenset({"YUYV", "MJPG"})


def orbbec_v4l2_node_is_rgb_candidate(path: str) -> bool:
    """True when ``v4l2-ctl`` reports YUYV or MJPG at 640x480 (Gemini 335 RGB UVC node)."""
    path_str = str(path)
    if not Path(path_str).exists() or not is_orbbec_v4l2_capture_device(path_str):
        return False
    if not (Path("/usr/bin/v4l2-ctl").exists() or Path("/bin/v4l2-ctl").exists()):
        return path_str.endswith("video6")  # best-effort fallback
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", path_str, "--list-formats-ext"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False
    current_fmt: str | None = None
    for line in out.splitlines():
        fmt_match = re.match(r"\s*\[\d+\]: '(\w+)'", line)
        if fmt_match:
            current_fmt = fmt_match.group(1).strip()
            continue
        if "Size: Discrete 640x480" in line and current_fmt in _RGB_V4L2_PIXEL_FORMATS:
            return True
    return False


def find_orbbec_capture_paths(*, prefer: str | None = None) -> list[str]:
    """Return Orbbec **RGB** V4L2 nodes only (YUYV/MJPG 640x480).

    Gemini 335 also exposes depth/IR on other ``/dev/video*`` and ``by-id`` symlinks
    (index0–5). Those are excluded here — RGB is often ``/dev/video6`` on a separate
    USB interface, not the by-id index2 IR stream.
    """
    raw_paths: list[str] = []
    if prefer and Path(prefer).exists():
        raw_paths.append(prefer)
    env_path = os.environ.get("ORBBEC_PATH", "")
    if env_path and Path(env_path).exists() and env_path not in raw_paths:
        raw_paths.append(env_path)
    for pattern in (
        "/dev/v4l/by-path/*Orbbec*",
        "/dev/v4l/by-id/usb-Orbbec_*",
    ):
        raw_paths.extend(sorted(glob.glob(pattern)))
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
        raw_paths.append(dev)

    seen: set[str] = set()
    rgb_paths: list[str] = []
    for path in raw_paths:
        if path in seen:
            continue
        seen.add(path)
        if orbbec_v4l2_node_is_rgb_candidate(path):
            rgb_paths.append(path)
    return rgb_paths


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


def apply_orbbec_v4l2_tuning(path: str, *, pixelformat: str | None = None) -> None:
    """Optional V4L2 tweaks for Orbbec top RGB.

    Default (``ORBBEC_MANUAL_EXPOSURE=0``): **auto exposure only** — matches the
    working ``VideoCapture`` path and avoids forcing YUYV/manual gain that can
    switch Gemini 335 onto IR/degraded streams.

    Set ``ORBBEC_MANUAL_EXPOSURE=1`` to apply lab manual exposure (legacy).
    """
    if not Path("/usr/bin/v4l2-ctl").exists() and not Path("/bin/v4l2-ctl").exists():
        if pixelformat is not None:
            reset_orbbec_v4l2(path, pixelformat=pixelformat)
        return

    manual = os.environ.get("ORBBEC_MANUAL_EXPOSURE", "0").lower() in ("1", "true", "yes")
    if not manual:
        subprocess.run(
            ["v4l2-ctl", "-d", path, "--set-ctrl=auto_exposure=3"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        return

    fmt = pixelformat or "YUYV"
    exposure = os.environ.get("ORBBEC_EXPOSURE", "100")
    gain = os.environ.get("ORBBEC_GAIN", "4")
    brightness = os.environ.get("ORBBEC_BRIGHTNESS", "0")
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            path,
            f"--set-fmt-video=width=640,height=480,pixelformat={fmt}",
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


def detect_orbbec_frame_glitch(frame: np.ndarray) -> str | None:
    """Return a reason string when the frame looks like a corrupted V4L2 decode; else ``None``."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        return "not a 3-channel BGR frame"

    height, width = frame.shape[:2]
    if height == 400 and width == 640:
        return "glitch: IR stream (640x400)"

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    _, saturation, _value = cv2.split(hsv)
    sat_mean = float(saturation.mean())

    # Gemini IR (GREY) opened via OpenCV: identical B=G=R channels (std ~ 0).
    channel_std = float(frame.std(axis=2).mean())
    if channel_std < 2.0:
        return f"glitch: greyscale/IR stream (channel_std={channel_std:.2f})"

    # Gemini IR dot pattern at high sharpness (synthetic noise or strong projector).
    if sharpness > 8_000 and sat_mean < 45 and channel_std < 8:
        return f"glitch: IR dot pattern (sharp={sharpness:.0f})"

    high_sat_ratio = float((saturation > 175).mean())
    if high_sat_ratio > 0.22:
        return f"glitch: high saturation ({high_sat_ratio:.0%} of pixels)"

    blue, green, red = cv2.split(frame.astype(np.float32))
    neon_green_ratio = float(((green > 90) & (green > red * 1.6) & (green > blue * 1.6)).mean())
    if neon_green_ratio > 0.06:
        return f"glitch: neon green patches ({neon_green_ratio:.0%} of pixels)"

    magenta_ratio = float(
        ((red > 70) & (blue > 70) & (green < red * 0.75) & (green < blue * 0.75)).mean()
    )
    if magenta_ratio > 0.06:
        return f"glitch: magenta/purple patches ({magenta_ratio:.0%} of pixels)"

    vivid = saturation > 80
    if int(vivid.sum()) > 500:
        hue = hsv[:, :, 0][vivid]
        green_hue_ratio = float(((hue >= 35) & (hue <= 90)).mean())
        purple_hue_ratio = float(((hue >= 125) & (hue <= 165)).mean())
        if green_hue_ratio > 0.35 and purple_hue_ratio > 0.15:
            return (
                "glitch: green+purple hue mix "
                f"(green={green_hue_ratio:.0%}, purple={purple_hue_ratio:.0%})"
            )

    return None


def assert_orbbec_rgb_frame(frame: np.ndarray, *, path: str = "") -> None:
    """Raise ``ConnectionError`` when the probe frame looks corrupted."""
    reason = detect_orbbec_frame_glitch(frame)
    if reason is None:
        return
    label = f"Orbbec top ({path})" if path else "Orbbec top"
    raise ConnectionError(
        f"{label}: {reason}. Replug USB3 or set ORBBEC_PATH=/dev/videoN"
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
    if detect_orbbec_frame_glitch(frame) is not None:
        score -= 500
    return score, sharpness


def _capture_profile(
    path: str,
    profile_name: str,
    fourcc: str | None,
    width: int | None,
    height: int | None,
    timeout_s: int,
):
    # Do not call reset_orbbec_v4l2 here: forcing YUYV via v4l2-ctl can knock Gemini 335 off RGB.
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


def validate_orbbec_v4l2_path(path: str) -> np.ndarray:
    """Capture one RGB probe frame; raise ``ConnectionError`` if unreadable or glitched."""
    profile_name, fourcc, width, height, timeout_s = ORBBEC_PROFILE
    captured = _capture_profile(path, profile_name, fourcc, width, height, timeout_s)
    if captured is None:
        raise ConnectionError(
            f"Orbbec top ({path}): cannot read a 640x480 frame. "
            "Replug USB3 or set ORBBEC_PATH=/dev/videoN"
        )
    frame, _reported = captured
    assert_orbbec_rgb_frame(frame, path=path)
    return frame


def capture_best_orbbec_rgb(*, prefer: str | None = None) -> OrbbecCaptureResult | None:
    """Try all Orbbec nodes; return the highest-scoring RGB-like capture."""
    best: OrbbecCaptureResult | None = None
    for path in find_orbbec_capture_paths(prefer=prefer):
        for profile_name, fourcc, width, height, timeout_s in ORBBEC_PROFILES:
            captured = _capture_profile(path, profile_name, fourcc, width, height, timeout_s)
            if captured is None:
                continue
            frame, reported = captured
            if detect_orbbec_frame_glitch(frame) is not None:
                continue
            score, sharpness = score_orbbec_rgb_frame(frame)
            candidate = OrbbecCaptureResult(
                path=path,
                frame=frame,
                reported=reported,
                sharpness=sharpness,
                score=score,
                profile_name=profile_name,
                fourcc=fourcc or "auto",
            )
            if best is None or candidate.score > best.score:
                best = candidate
    if best is None or best.score < 0:
        return None
    assert_orbbec_rgb_frame(best.frame, path=best.path)
    return best


def diagnose_orbbec_top_probe(*, prefer: str | None = None) -> str:
    """Human-readable summary when ``capture_best_orbbec_rgb`` finds nothing."""
    paths = find_orbbec_capture_paths(prefer=prefer)
    if not paths:
        return "No Orbbec V4L2 capture nodes found (USB unplugged?)."

    lines: list[str] = []
    for path in paths:
        if not Path(path).exists():
            lines.append(f"{path}: device missing (replug USB?)")
            continue
        node_ok = False
        for profile_name, fourcc, width, height, timeout_s in ORBBEC_PROFILES:
            captured = _capture_profile(path, profile_name, fourcc, width, height, timeout_s)
            if captured is None:
                lines.append(f"{path} [{fourcc}]: cannot read frame (busy or wrong format?)")
                continue
            frame, reported = captured
            glitch = detect_orbbec_frame_glitch(frame)
            if glitch is not None:
                lines.append(f"{path} [{fourcc}] {reported}: {glitch}")
                continue
            score, sharpness = score_orbbec_rgb_frame(frame)
            lines.append(
                f"{path} [{fourcc}] {reported}: OK score={score} sharp={sharpness:.0f} ({profile_name})"
            )
            node_ok = True
        if not node_ok:
            continue
    lines.append(
        "Tips: kill stuck processes on /dev/video* (e.g. `fuser -v /dev/video6`), "
        "replug Orbbec on USB3. Do not force YUYV/manual exposure unless ORBBEC_MANUAL_EXPOSURE=1."
    )
    return "\n".join(lines)


def orbbec_top_opencv_camera_info(*, prefer: str | None = None) -> dict[str, Any] | None:
    """Return a ``lerobot-find-cameras``-compatible entry for the auto-discovered Orbbec top RGB node."""
    env_path = os.environ.get("ORBBEC_PATH", "")
    prefer_path = prefer or (env_path if env_path and Path(env_path).exists() else None)
    best = capture_best_orbbec_rgb(prefer=prefer_path)
    if best is None:
        return None
    return {
        "name": f"Orbbec Gemini top RGB (auto @ {best.path})",
        "type": "OpenCV",
        "id": best.path,
        "backend_api": "V4L2",
        "orbbec_top": True,
        "default_stream_profile": {
            "format": best.fourcc,
            "fourcc": best.fourcc,
            "width": 640,
            "height": 480,
            "fps": 30,
        },
        "probe_sharpness": best.sharpness,
        "probe_score": best.score,
    }


def opencv_config_for_orbbec_top(path: str | int, *, fourcc: str | None = None) -> OpenCVCameraConfig:
    """OpenCV config used by CRP record for the Orbbec top camera."""
    from lerobot.cameras.opencv.configuration_opencv import ColorMode, OpenCVCameraConfig

    path_str = str(path)
    apply_orbbec_v4l2_tuning(path_str)
    resolved_fourcc = None if fourcc in (None, "auto") else fourcc
    return OpenCVCameraConfig(
        index_or_path=path_str,
        width=640,
        height=480,
        fps=30,
        fourcc=resolved_fourcc,
        warmup_s=2,
        color_mode=ColorMode.RGB,
    )
