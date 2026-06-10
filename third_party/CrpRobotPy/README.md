# CrpRobotPy SDK

Binaries: `CrpRobotPy.so`, `libRobotService.so`.

## Build (recommended)

Dual-arm reads require `dlmopen` isolation for the second `libRobotService.so` session.
Build from source and deploy **CrpRobotPy.so only** (keep vendor `libRobotService.so`):

```bash
cd ~/python_C++/CrpRobotPy
PYTHON=~/miniforge3/envs/lerobot/bin/python3 ./build.sh
cp -f CrpRobotPy.so ~/lerobot/third_party/CrpRobotPy/
cp -f ~/lerobot/license.key ~/lerobot/third_party/CrpRobotPy/
```

Do **not** overwrite `libRobotService.so` unless you also replace `license.key` with the
matching CrobotpOSSDK authorization (`DEPLOY_OSS_LIB=1 ./build.sh`).

Vendor layout: `libRobotService.so` ~14.4 MB + repo root `license.key` (480 B hex file).

See also `接口参考文件.cpp` for the pybind surface.

## Legacy patch (`CrpRobotPatch.so`)

Only needed for **old vendor** `.so` files that expose `connect_second` but lack native
`read_end_pose_user_second` / `read_joints_second`. Current builds from `python_C++/CrpRobotPy`
do not need the patch for TCP/joint reads.

```bash
bash third_party/CrpRobotPy/build_patch.sh
```

## Exit segfault (dual session)

Some builds segfault when the pybind destructor runs after dual disconnect. `CRPArmDual.disconnect()`
registers `os._exit(0)` at process exit. Disable with: `export CRP_DUAL_NO_OS_EXIT=1`.

## Verify

```bash
python -c "
from lerobot.robots.crp_arm_dual.sdk import ensure_crp_sdk_loaded
ensure_crp_sdk_loaded()
from CrpRobotPy import CrpRobotPy
r = CrpRobotPy()
print('connect_second', hasattr(r, 'connect_second'))
print('read_end_pose_user_second', hasattr(r, 'read_end_pose_user_second'))
print('read_joints_second', hasattr(r, 'read_joints_second'))
"
```
