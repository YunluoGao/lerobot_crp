#!/usr/bin/env bash
# Build CrpRobotPatch.so (read_end_pose_user_second for dual CRP teleop).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="${ROOT}/patch"
OUT="${ROOT}/CrpRobotPatch.so"

PYTHON="${PYTHON:-python3}"
if ! command -v "$PYTHON" >/dev/null 2>&1; then
  echo "PYTHON not found: $PYTHON" >&2
  exit 1
fi

PYBIND_INCLUDES="$("$PYTHON" -m pybind11 --includes 2>/dev/null)" || {
  echo "Install pybind11: pip install pybind11" >&2
  exit 1
}

PYEXT_SUFFIX="$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
CXX="${CXX:-g++}"
CXXFLAGS=(-std=c++17 -O2 -fPIC -Wall -Wextra -I"${PATCH}")

echo "Building ${OUT} ..."
# Do NOT link libRobotService: second arm uses dlmopen copy; call IRobotService via vtable.
"${CXX}" "${CXXFLAGS[@]}" -shared \
  "${PATCH}/crp_read_second.cpp" \
  "${PATCH}/crp_patch_module.cpp" \
  ${PYBIND_INCLUDES} \
  -ldl \
  -o "${OUT}" \
  -Wl,-rpath,"${ROOT}"

echo "Done: ${OUT} (for CPython ${PYEXT_SUFFIX})"
