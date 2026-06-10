#!/usr/bin/env bash
# Diagnostic helper: dual manual probes while lerobot-crp_tele_dual is running.
# See README.md §「阶段 3 — 与遥操作并发」. Conflicts with built-in per-arm probes on the same IP.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${DIR}/crp_getui_probe"

if [[ ! -x "${PROBE}" ]]; then
  echo "Build first: bash ${DIR}/build.sh" >&2
  exit 1
fi

IP_LEFT="${1:-192.168.0.100}"
IP_RIGHT="${2:-192.168.0.101}"
HZ="${3:-2}"

echo "WARNING: lerobot-crp_tele_dual already starts one probe per arm."
echo "This script adds a SECOND SDK session per IP (diagnostic only)."
echo ""
echo "Terminal A: lerobot-crp_tele_dual"
echo "Terminal B: this script — IPs ${IP_LEFT} / ${IP_RIGHT} @ ${HZ} Hz"
echo ""
read -r -p "Press Enter when teleop is running (Ctrl+C to abort) ..."

echo "Starting manual probes (Ctrl+C stops both) ..."
"${PROBE}" "${IP_LEFT}" --json --indices 50,56,57,58 --hz "${HZ}" --arm-label left &
PID_L=$!
"${PROBE}" "${IP_RIGHT}" --json --indices 50,56,57,58 --hz "${HZ}" --arm-label right &
PID_R=$!

trap 'kill "${PID_L}" "${PID_R}" 2>/dev/null || true' INT TERM
wait
