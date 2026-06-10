# crp_getui_probe — 官方 `IRobotService::getUI` 读夹爪 UI

独立 C++ 可执行文件，通过 **CrobotpOSSDK** 头文件里的 `IRobotService::getUI` 轮询示教器 UI 寄存器。  
**不**链接 `CrpRobotPy` / `CrpRobotPatch`（与主遥操作进程的写 UI / 发 GP 路径隔离）。

## 在遥操作中的角色（正式路径）

`lerobot-crp_tele_dual` 默认会为左右臂各起一个本工具的子进程（见 `lerobot.scripts.crp_gp.ui_probe`（迁移后；当前阶段见 `getui_probe/paths.py`））：

| 项 | 说明 |
|----|------|
| 写入 UI50/51–54 | 仍由主进程 `CrpRobotPy` 完成 |
| 读取 UI50/56/57/58 | 本子进程（官方 getUI），用于心跳日志中的 `L_UI50` / `L_POSIT` / `L_SPEED` / `L_TORQUE` 等 |
| 默认索引 | `50,56,57,58`（与 `crp_ui.py` 中常量一致） |
| 默认频率 | `2 Hz`（`--gripper_ui_probe_hz`） |

**首次使用前必须构建**（未构建时遥操作会打 warning，心跳里 POSIT/SPEED/TORQUE 显示为 `?`）：

```bash
bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh
lerobot-crp_tele_dual   # 平时参数即可；默认已开启 UI 子进程
```

关闭子进程读数（仅保留主手映射的 UI50 命令值）：

```bash
lerobot-crp_tele_dual --gripper_ui_probe=false
```

调轮询频率：

```bash
lerobot-crp_tele_dual --gripper_ui_probe_hz=5
```

仓库根目录 [README.md](../../README.md) 中有完整配置表与数据流说明。

## 构建

```bash
bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh
```

- 头文件：仓库里的 `CrobotpOSSDK-*/cpp/include`
- **默认运行时库**：`third_party/CrpRobotPy/libRobotService.so`（与 `lerobot-crp_tele_dual` 相同，可用仓库根目录的 `license.key`）
- 构建时会自动把仓库根目录的 `license.key` 复制到 `src/lerobot/robots/crp_arm_dual/getui_probe/license.key`

若要用 **CrobotpOSSDK 安装包自带** 的 `bin/libRobotService.so`（需厂商为该版本单独授权）：

```bash
CRP_PROBE_LIB=oss bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh
```

产物：

- `src/lerobot/robots/crp_arm_dual/getui_probe/crp_getui_probe`
- `src/lerobot/robots/crp_arm_dual/getui_probe/libRobotService.so`

## 授权（`unlicensed` / `getService failed`）

日志里出现：

```text
[warning] unlicensed
error: getService<IRobotService> failed
```

说明 **`license.key` 与当前目录下的 `libRobotService.so` 不匹配**。

| 使用的库 | `license.key` |
|----------|----------------|
| 默认（CrpRobotPy vendor） | 仓库根目录 `license.key`（构建时会复制到本目录） |
| `CRP_PROBE_LIB=oss` | 需向卡诺普索取 **针对 CrobotpOSSDK 1.0.4 / 本机** 的授权 |

检查：

```bash
cd src/lerobot/robots/crp_arm_dual/getui_probe
ls -la license.key libRobotService.so
./crp_getui_probe 192.168.0.100 --count 1
```

### `connect` 失败 / 段错误

- **段错误**：旧版在 `connect` 失败后会重复释放 SDK，请重新 `bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh` 后再试。
- **Could not connect**：检查 `ping`、柜机上电/急停、IP 是否与 `ip1`/`ip2` 一致；多试 `--connect-retries 10`。
- **与遥操作并发**：正式集成下每臂已各有一个子进程；**不要再手动**对同一 IP 起第二个 probe，除非做下面「诊断」里的对比实验。

## 单独运行（调试 / 对比）

与遥操作子进程相同的 JSON 模式示例：

```bash
cd src/lerobot/robots/crp_arm_dual/getui_probe

# 默认即 UI50,56,57,58 @ 2 Hz；teleop 子进程同样传 --json
./crp_getui_probe 192.168.0.100 --json --arm-label left

# 保存日志便于对比时段
./crp_getui_probe 192.168.0.101 --json --arm-label right | tee ui_right.jsonl
```

手动 CLI 时请先 **关掉** `lerobot-crp_tele_dual`，或确认控制器允许多路 SDK，避免与正式子进程抢连接。

## 诊断流程（可选）

当心跳长期为 `?`、子进程退出或怀疑控制器/SDK 问题时，可按阶段单独压测（**非**日常使用步骤）。

### 阶段 1 — 仅小工具（基线）

1. 不要运行 `lerobot-crp_tele_dual`。
2. 示教器夹爪程序照常，手动动夹爪。
3. `./crp_getui_probe <IP> --count 5`（默认 50,56,57,58 @ 2 Hz）

看：`abort` / `getUI FAIL`；UI56–58 是否随夹爪变化。

### 阶段 2 — 提高频率（仍无遥操作）

```bash
./crp_getui_probe 192.168.0.100 --indices 56,57,58 --hz 20
```

### 阶段 3 — 与遥操作并发（排查双会话）

终端 A：`lerobot-crp_tele_dual`  
终端 B（可选脚本，与内置子进程重复，仅诊断）：

```bash
bash src/lerobot/robots/crp_arm_dual/getui_probe/run_phase3_example.sh 192.168.0.100 192.168.0.101 2
```

或手动 `./crp_getui_probe <IP> --json --arm-label left`（勿对已在 teleop 中占用的 IP 再起第三路）。

| 结果 | 含义 |
|------|------|
| 仅 teleop（内置子进程）稳定 | 正常生产路径 |
| 手动第二路 probe 导致断连/崩溃 | 勿对同一 IP 再起 probe；调 `--gripper_ui_probe_hz` 或查示教程序 |
| 阶段 1/2  alone 即崩 | 查授权、示教程序或控制器，与 CrpRobotPy 写路径无关 |

双臂需两个 IP、两个进程（与 `start_dual` 一致）。

## 命令行参数

| 参数 | 说明 |
|------|------|
| `<robot_ip>` | 控制器 IP（必填） |
| `--indices` | 逗号分隔 UI 下标，默认 `50,56,57,58`（与 teleop 子进程一致） |
| `--hz` | 轮询频率，默认 `2.0`（与 `gripper_ui_probe_hz` 默认一致），最大 `200` |
| `--count N` | 采样 N 次后退出 |
| `--json` | 每行一个 JSON（teleop 集成**必须**） |
| `--arm-label NAME` | JSON 字段 `arm`（如 `left` / `right`） |
| `--no-clear-error` | 连接后不自动 `clearError()` |
| `--connect-retries N` | 连接重试次数 |

## 注意

- 默认链接 **CrpRobotPy 的 `libRobotService.so`**（与遥操作同授权）；纯 1.0.4 包 so 用 `CRP_PROBE_LIB=oss` 并换对应 `license.key`。
- 本工具**只读** UI；写 UI50/51–54 仍由 `lerobot-crp_tele_dual` 主进程完成。
- Python 侧仅通过子进程 stdout 解析 JSON，无编译期依赖；修改 C++ 后需重新 `build.sh`。
