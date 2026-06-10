#!/usr/bin/env bash
# Install pinned placo / pin / cmeel-urdfdom stack for CRP SO101 FK (LeRobot 0.5).
# Fixes: ImportError: liburdfdom_sensor.so.4.0: cannot open shared object file
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# shellcheck source=_resolve_python.sh
source "${ROOT}/scripts/_resolve_python.sh"

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

PY="${PYTHON:-$(_resolve_lerobot_python "${ROOT}")}"
echo "== install placo stack (PYTHON=${PY}) =="

# placo 0.9.16 wheels link against urdfdom 4.x (.so.4.0).
# A loose `pip install placo` may pull cmeel-urdfdom 6.x and break import.
"${PY}" -m pip install -U pip
"${PY}" -m pip install \
  'placo>=0.9.6,<0.9.17' \
  'pin==3.4.0' \
  'cmeel-urdfdom==4.0.1' \
  'cmeel-tinyxml2==10.0.0' \
  'cmeel-console-bridge==1.0.2.3'

echo "== verify placo import =="
"${PY}" -c "import placo; print('placo', placo.__file__)"

echo "== verify SO101 FK smoke (no robot power) =="
PYTHONPATH="${ROOT}/src" "${PY}" -c "
from lerobot.model.kinematics import RobotKinematics
from lerobot.scripts.crp_gp.loop import TARGET_FRAME_NAME
from lerobot.scripts.crp_gp.mappers.so101_urdf import (
    SO101_ARM_MOTOR_NAMES,
    get_endpose2Crp_urdf,
    resolve_so101_urdf_path,
)
kin = RobotKinematics(
    resolve_so101_urdf_path(),
    TARGET_FRAME_NAME,
    list(SO101_ARM_MOTOR_NAMES),
)
action = {f'{m}.pos': 0.0 for m in SO101_ARM_MOTOR_NAMES}
print('FK gp_mm_deg =', get_endpose2Crp_urdf(action, kinematics=kin))
"

echo "== placo deps OK =="
