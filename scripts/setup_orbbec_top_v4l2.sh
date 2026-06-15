#!/usr/bin/env bash
# Orbbec Gemini 335 top camera V4L2 helper.
#
#   ORBBEC_PATH=/dev/v4l/by-id/... bash scripts/setup_orbbec_top_v4l2.sh
#
# Default: auto exposure only (safe — do NOT force YUYV/manual gain on RGB).
# Legacy lab manual exposure (may break RGB on some firmware):
#   ORBBEC_MANUAL_EXPOSURE=1 ORBBEC_EXPOSURE=100 ORBBEC_GAIN=4 bash scripts/setup_orbbec_top_v4l2.sh
set -euo pipefail

DEV="${ORBBEC_PATH:-/dev/video6}"
MANUAL="${ORBBEC_MANUAL_EXPOSURE:-0}"

if [[ ! -e "${DEV}" ]]; then
  echo "setup_orbbec_top_v4l2: device not found: ${DEV}" >&2
  exit 1
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "setup_orbbec_top_v4l2: install v4l-utils (v4l2-ctl)" >&2
  exit 1
fi

if [[ "${MANUAL}" == "1" ]]; then
  EXPOSURE="${ORBBEC_EXPOSURE:-100}"
  GAIN="${ORBBEC_GAIN:-4}"
  BRIGHTNESS="${ORBBEC_BRIGHTNESS:-0}"
  v4l2-ctl -d "${DEV}" \
    --set-fmt-video=width=640,height=480,pixelformat=YUYV \
    --set-ctrl=auto_exposure=1 \
    --set-ctrl=exposure_time_absolute="${EXPOSURE}" \
    --set-ctrl=gain="${GAIN}" \
    --set-ctrl=brightness="${BRIGHTNESS}" \
    >/dev/null
  echo "Orbbec V4L2 manual: ${DEV} exposure=${EXPOSURE} gain=${GAIN} brightness=${BRIGHTNESS}"
else
  v4l2-ctl -d "${DEV}" --set-ctrl=auto_exposure=3 >/dev/null
  echo "Orbbec V4L2 auto exposure: ${DEV} (ORBBEC_MANUAL_EXPOSURE=1 for legacy manual tuning)"
fi
