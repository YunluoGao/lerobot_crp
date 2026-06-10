# third_party (CRP / SO101)

| Path | Purpose |
|------|---------|
| `CrpRobotPy/` | Vendor SDK + dual-arm `CrpRobotPatch.so` |
| `SO101/` | URDF for leader FK → CRP GP |

Native getUI probe: `src/lerobot/robots/crp_arm_dual/getui_probe/`.

## Python 3.12+（LeRobot 0.5 必须）

旧环境 `lerobotx`（Python 3.10）**不能** import 新版 LeRobot。请一次性迁移：

```bash
cd /path/to/lerobot
bash scripts/setup_python_env.sh
conda activate lerobot   # 或: source .venv/bin/activate 若 USE_VENV=1
```

脚本会：安装/升级到 **Python 3.12**、``pip install -e ".[feetech]"``、重编 ``CrpRobotPatch.so``、冒烟 ``CRPArmDual``。

仅 venv、不用 conda：

```bash
USE_VENV=1 bash scripts/setup_python_env.sh
```

## 冒烟

安装完成后可验证 import：

```bash
python -c "from lerobot.robots.crp_arm_dual import CRPArmDualConfig; print('OK')"
```

## CrpRobotPy 与 Python ABI

``CrpRobotPy.so`` 必须与当前解释器 ABI 一致。若 ``setup_python_env.sh`` 在 3.12 上报 ``undefined symbol``，需要向厂商索取 **cp312** 版 ``CrpRobotPy.so`` / ``libRobotService.so``，替换 ``third_party/CrpRobotPy/`` 后再运行：

```bash
bash third_party/CrpRobotPy/build_patch.sh
```

``license.key`` 放仓库根目录即可；``getui_probe/build.sh`` 会自动复制到探针目录。
