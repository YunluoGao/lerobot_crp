#!/usr/bin/env bash
# Apply Orbbec top-camera V4L2 tuning (lab Gemini 335: auto-exposure is too dark overhead).
#
#   ORBBEC_PATH=/dev/v4l/by-id/... bash scripts/setup_orbbec_top_v4l2.sh
#
# Override via env, e.g. ORBBEC_EXPOSURE=120 ORBBEC_GAIN=6
# Default: exposure=100 gain=4 (lab overhead). Avoid exposure ~300–800 (driver blow-out).
set -euo pipefail

DEV="${ORBBEC_PATH:-/dev/video6}"
EXPOSURE="${ORBBEC_EXPOSURE:-100}"
GAIN="${ORBBEC_GAIN:-4}"
BRIGHTNESS="${ORBBEC_BRIGHTNESS:-0}"

if [[ ! -e "${DEV}" ]]; then
  echo "setup_orbbec_top_v4l2: device not found: ${DEV}" >&2
  exit 1
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "setup_orbbec_top_v4l2: install v4l-utils (v4l2-ctl)" >&2
  exit 1
fi

v4l2-ctl -d "${DEV}" \
  --set-fmt-video=width=640,height=480,pixelformat=YUYV \
  --set-ctrl=auto_exposure=1 \
  --set-ctrl=exposure_time_absolute="${EXPOSURE}" \
  --set-ctrl=gain="${GAIN}" \
  --set-ctrl=brightness="${BRIGHTNESS}" \
  >/dev/null

echo "Orbbec V4L2 tuned: ${DEV} exposure=${EXPOSURE} gain=${GAIN} brightness=${BRIGHTNESS}"
