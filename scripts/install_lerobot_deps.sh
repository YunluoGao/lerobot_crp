#!/usr/bin/env bash
# Install LeRobot + CRP deps into the active conda env or repo .venv.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# shellcheck source=_resolve_python.sh
source "${ROOT}/scripts/_resolve_python.sh"

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

PY="${PYTHON:-$(_resolve_lerobot_python "${ROOT}")}"
echo "Using: ${PY}"
"${PY}" -c "import sys; assert sys.version_info >= (3, 12), f'need Python >= 3.12, got {sys.version}'"

echo "== pip install =="
"${PY}" -m pip install -U pip setuptools wheel
"${PY}" -m pip install pybind11
"${PY}" -m pip install -e "${ROOT}[feetech]"

echo "== build CrpRobotPatch =="
PYTHON="${PY}" bash "${ROOT}/third_party/CrpRobotPy/build_patch.sh"

echo "== smoke test =="
"${PY}" -c "
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual import CRPArmDualConfig
make_robot_from_config(CRPArmDualConfig(ip1='127.0.0.1', ip2='127.0.0.2'))
print('OK')
"
echo "Done."
