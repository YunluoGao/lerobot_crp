#!/usr/bin/env bash
# Create/update Python 3.12+ env for LeRobot 0.5 + CRP (replaces lerobotx / 3.10).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

# Avoid broken sandbox proxies during pip/conda (Cursor sometimes sets these).
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
export NO_PROXY="${NO_PROXY:-127.0.0.1,localhost,::1}"

PYTHON_BIN="${PYTHON_BIN:-}"
CONDA_ENV="${CONDA_ENV:-lerobot}"
USE_VENV="${USE_VENV:-0}"

pick_python() {
  if [[ -n "${PYTHON_BIN}" && -x "${PYTHON_BIN}" ]]; then
    echo "${PYTHON_BIN}"
    return
  fi
  if command -v python3.12 >/dev/null 2>&1; then
    command -v python3.12
    return
  fi
  if [[ -x "${HOME}/miniforge3/bin/python3.12" ]]; then
    echo "${HOME}/miniforge3/bin/python3.12"
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    local ver
    ver="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    local major minor
    major="${ver%%.*}"
    minor="${ver#*.}"
    if [[ "${major}" -eq 3 && "${minor}" -ge 12 ]]; then
      command -v python3
      return
    fi
  fi
  echo "error: need Python >= 3.12 (set PYTHON_BIN=... or install miniforge python 3.12)" >&2
  exit 1
}

install_editable() {
  local py="$1"
  echo "== pip install lerobot (editable) =="
  "${py}" -m pip install -U pip setuptools wheel
  "${py}" -m pip install pybind11
  "${py}" -m pip install -e "${ROOT}[feetech]"
}

build_crp_patch() {
  local py="$1"
  echo "== build CrpRobotPatch.so =="
  PYTHON="${py}" bash "${ROOT}/third_party/CrpRobotPy/build_patch.sh"
}

verify_imports() {
  local py="$1"
  echo "== verify LeRobot + CrpRobotPy =="
  "${py}" -c "
import sys
assert sys.version_info >= (3, 12), sys.version
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual import CRPArmDualConfig
cfg = CRPArmDualConfig(ip1='127.0.0.1', ip2='127.0.0.2')
robot = make_robot_from_config(cfg)
assert robot.name == 'crp_arm_dual'
print('OK LeRobot', sys.version.split()[0], '+ CRPArmDual')
"
}

if [[ "${USE_VENV}" == "1" ]]; then
  PY="$(pick_python)"
  echo "== venv at ${ROOT}/.venv (${PY}) =="
  "${PY}" -m venv "${ROOT}/.venv"
  PY="${ROOT}/.venv/bin/python"
  install_editable "${PY}"
  build_crp_patch "${PY}"
  verify_imports "${PY}"
  echo ""
  echo "Done. Use: source ${ROOT}/.venv/bin/activate"
  exit 0
fi

if command -v conda >/dev/null 2>&1; then
  CONDA_BASE="$(conda info --base 2>/dev/null || echo "${HOME}/miniforge3")"
  # shellcheck disable=SC1091
  source "${CONDA_BASE}/etc/profile.d/conda.sh"
  if conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
    echo "== conda install python=3.12 into env ${CONDA_ENV} =="
    conda install -n "${CONDA_ENV}" python=3.12 pip ffmpeg -c conda-forge -y
  else
    echo "== conda env create ${CONDA_ENV} (python 3.12) =="
    conda env create -f "${ROOT}/environment-crp.yml"
  fi
  conda activate "${CONDA_ENV}"
  PY="$(command -v python)"
  install_editable "${PY}"
  build_crp_patch "${PY}"
  verify_imports "${PY}"
  echo ""
  echo "Done. Use: conda activate ${CONDA_ENV}"
  exit 0
fi

echo "conda not found; falling back to venv" >&2
USE_VENV=1 exec "$0" "$@"
