#!/usr/bin/env bash
# Build crp_getui_probe (official getUI subprocess for lerobot-crp_tele_dual).
# Links libRobotService.so from CrpRobotPy vendor by default; see README.md.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_repo_root() {
  local d="${ROOT}"
  while [[ "${d}" != "/" ]]; do
    if [[ -d "${d}/third_party/CrpRobotPy" ]]; then
      echo "${d}"
      return 0
    fi
    d="$(dirname "${d}")"
  done
  return 1
}

REPO_ROOT="$(find_repo_root)" || {
  echo "error: cannot find repo root (third_party/CrpRobotPy)" >&2
  exit 1
}
OUT="${ROOT}/crp_getui_probe"

find_oss_sdk_root() {
  local c d
  if [[ -n "${CRP_OSSDK_ROOT:-}" ]]; then
    echo "${CRP_OSSDK_ROOT}"
    return 0
  fi
  local -a candidates=(
    "${REPO_ROOT}/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
    "${HOME}/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
    "${HOME}/python_C++/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
    "${HOME}/下载/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
    "/home/xxx/下载/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
    "${REPO_ROOT}/../lerobot_old/CrobotpOSSDK-1.0.4-Linux-x86_64-ubuntu22"
  )
  for c in "${candidates[@]}"; do
    if [[ -f "${c}/cpp/include/CSDKLoader.h" ]]; then
      echo "${c}"
      return 0
    fi
  done
  for d in \
    "${HOME}"/CrobotpOSSDK-* \
    "${HOME}/python_C++"/CrobotpOSSDK-* \
    "${REPO_ROOT}"/CrobotpOSSDK-* \
    "${REPO_ROOT}"/../lerobot_old/CrobotpOSSDK-*; do
    if [[ -d "${d}" && -f "${d}/cpp/include/CSDKLoader.h" ]]; then
      echo "${d}"
      return 0
    fi
  done
  return 1
}

# Which libRobotService.so to ship next to the probe:
#   vendor (default) — third_party/CrpRobotPy, same as lerobot-crp_tele_dual license
#   oss            — CrobotpOSSDK package bin/, needs vendor license for that build
PROBE_LIB="${CRP_PROBE_LIB:-vendor}"

if ! OSS_SDK_ROOT="$(find_oss_sdk_root)"; then
  echo "error: CrobotpOSSDK headers not found (need cpp/include/CSDKLoader.h)." >&2
  echo "Set CRP_OSSDK_ROOT or unpack SDK under ${REPO_ROOT}/" >&2
  exit 1
fi

INCLUDE="${OSS_SDK_ROOT}/cpp/include"
VENDOR_LIB="${REPO_ROOT}/third_party/CrpRobotPy/libRobotService.so"
OSS_LIB="${OSS_SDK_ROOT}/bin/libRobotService.so"

if [[ "${PROBE_LIB}" == "oss" ]]; then
  LIB_SO="${OSS_LIB}"
  LIB_NOTE="CrobotpOSSDK package (${OSS_SDK_ROOT})"
else
  LIB_SO="${VENDOR_LIB}"
  LIB_NOTE="CrpRobotPy vendor lib (same as teleop)"
fi

if [[ ! -d "${INCLUDE}" ]]; then
  echo "error: headers missing at ${INCLUDE}" >&2
  exit 1
fi
if [[ ! -f "${LIB_SO}" ]]; then
  echo "error: ${LIB_SO} not found" >&2
  if [[ "${PROBE_LIB}" == "vendor" ]]; then
    echo "Install CrpRobotPy under third_party/CrpRobotPy or use: CRP_PROBE_LIB=oss bash build.sh" >&2
  fi
  exit 1
fi

LICENSE_SRC=""
for candidate in \
  "${REPO_ROOT}/license.key" \
  "${OSS_SDK_ROOT}/license.key" \
  "${OSS_SDK_ROOT}/bin/license.key" \
  "${REPO_ROOT}/third_party/CrpRobotPy/license.key"; do
  if [[ -f "${candidate}" ]]; then
    LICENSE_SRC="${candidate}"
    break
  fi
done

CXX="${CXX:-g++}"
echo "Building ${OUT} ..."
echo "  headers=${OSS_SDK_ROOT}"
echo "  lib=${LIB_NOTE}"
echo "  CRP_PROBE_LIB=${PROBE_LIB}"

"${CXX}" -std=c++17 -O2 -Wall -Wextra \
  -I"${INCLUDE}" \
  "${ROOT}/crp_getui_probe.cpp" \
  -ldl \
  -o "${OUT}" \
  -Wl,-rpath,'$ORIGIN'

cp -f "${LIB_SO}" "${ROOT}/libRobotService.so"

if [[ -n "${LICENSE_SRC}" ]]; then
  cp -f "${LICENSE_SRC}" "${ROOT}/license.key"
  echo "Copied license.key from ${LICENSE_SRC}"
else
  echo "warning: no license.key found — copy it next to ${OUT} before running (see README)." >&2
fi

echo "Done: ${OUT}"
echo ""
echo "Teleop (default gripper_ui_probe=true):"
echo "  lerobot-crp_tele_dual"
echo ""
echo "Standalone check (same defaults as teleop subprocess: UI50,56–58 @ 2 Hz JSON):"
echo "  cd ${ROOT} && ./crp_getui_probe <robot_ip> --json --arm-label left --count 3"
