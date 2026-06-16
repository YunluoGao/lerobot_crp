# LeRobot + 双 CRP 臂遥操作与数据采集

## 目录

- [项目简介](#项目简介)
- [默认参数](#默认参数)
- [环境安装](#环境安装)
- [开录前检查](#开录前检查)
- [遥操作](#遥操作)
- [数据采集](#数据采集)
- [训练](#训练)
- [回放与策略部署](#回放与策略部署)
- [已知限制与排查](#已知限制与排查)
- [后续待办明细](#后续待办明细)

---

## 项目简介

将 LeRobot 适配到 **卡诺普（CRP）双 GP 从臂 + 双 SO101 主臂** 场景：遥操作、数据集录制、ACT 等策略训练。

**常用命令：**

```bash
cd ~/lerobot
conda activate lerobot
sudo chmod 666 /dev/ttyACM*

lerobot-crp-tele-dual

# TOP-RGB 检测
lerobot-find-cameras opencv
# 在 outputs/captured_images/ 里找到真 RGB 对应的 /dev/videoN
export ORBBEC_PATH=/dev/videoN

lerobot-crp-record-dual \
    --dataset.repo_id=user/20260615_gyl_6 \
    --dataset.episode_time_s=300 \
    --dataset.reset_time_s=10 \
    --dataset.num_episodes=7 \
    --resume=false

# 同步显示所有相机视频
lerobot-dataset-viz \
    --repo-id user/20260615_gyl_5 \
    --episode-index 3

lerobot-train \
  --policy.type=act \
  --dataset.repo_id=user/20260615_gyl_merged \
  --output_dir=outputs/train/act_20260615_gyl \
  --job_name=act_20260615_gyl \
  --policy.device=cuda \
  --steps=50000 \
  --batch_size=4 \
  --eval.n_episodes=0 \
  --wandb.enable=false \
  --policy.push_to_hub=false

lerobot-crp-replay-dual \
  --dataset.repo_id=user/20260615_gyl_merged \
  --dataset.episode=0 \
  --fps=8 \
  --max_joint_delta_deg=5 \
  --speed_ratio=15 \
  --gp_align_delay_s=5 \
  --robot.ip1=192.168.0.100 \
  --robot.ip2=192.168.0.101

# 策略部署（须先 pip install -e . 注册 lerobot-crp-deploy-dual）
lerobot-crp-deploy-dual \
  --policy.path=outputs/train/act_20260615_gyl/checkpoints/last/pretrained_model \
  --policy.n_action_steps=1 \
  --single_task=assemble \
  --fps=8 \
  --duration_s=300 \
  --max_joint_delta_deg=10 \
  --speed_ratio=25 \
  --gp_align_delay_s=5 \
  --robot.ip1=192.168.0.100 \
  --robot.ip2=192.168.0.101 \
  --play_sounds=false
```

**CLI 注册：** 修改仓库后执行 `cd ~/lerobot && pip install -e .`，否则用  
`python -m lerobot.scripts.lerobot_crp_deploy_dual ...` 代替 `lerobot-crp-deploy-dual`。

默认值见 [`src/lerobot/scripts/crp_gp/config.py`](src/lerobot/scripts/crp_gp/config.py)。

---

## 默认参数

### 硬件默认值

| 项 | 默认值 | 说明 |
|----|--------|------|
| CRP 左 / 右 IP | `192.168.0.100` / `192.168.0.101` | `--robot.ip1` / `--robot.ip2` |
| SO101 左 / 右串口 | `/dev/ttyACM1` / `/dev/ttyACM0` | 标定 `1.json` / `2.json` |
| Top 相机 | `ORBBEC_PATH` 或 `--robot.cameras.top.index_or_path` | 先用 `lerobot-find-cameras opencv` 确认 RGB 对应的 `/dev/videoN`，再 `export ORBBEC_PATH=...` |
| 腕部 RealSense | `218622273151` / `218622278121` | **须 USB3**；848×480@30 |
| 录制相机 | `top` + 双腕 | 640×480 top；`warmup_s=2` |
| 遥操作相机 | 无 | `--robot.cameras={}` |
| `--left.wrist_flex_sign` / `--right.wrist_flex_sign` | `0` | 默认**不**把 J4（腕弯）映射到 CRP 姿态，末端保持 align 时竖直；需要腕部倾斜自由度时改为 `1` |

### dataset 默认值

| 参数 | 默认 | 说明 |
|------|------|------|
| `--dataset.repo_id` | `user/date_name_ordinal` | 占位名，开录前改成实际 repo |
| `--dataset.fps` | `16` | 写盘 / MP4 帧率 |
| `--dataset.episode_time_s` | `300` | 单集最长（秒） |
| `--dataset.reset_time_s` | `30` | 集间 reset；最后 10s 倒计时 |
| `--dataset.num_episodes` | `30` | 一次启动录几集 |
| `--dataset.single_task` | `assemble` | 每帧 task 字段 |
| `--dataset.push_to_hub` | `false` | 默认仅本地；可事后 `push_to_hub()` |
| `--resume` | `false` | 续录改为 `true` |
| `--dataset.camera_encoder` | `h264` / `crf=18` | MP4 编码 |
| `--dataset.num_image_writer_processes` | `0` | 主进程写 PNG |
| `control_fps`（内置） | `80` | GP 控制环，与写盘 fps 独立 |

完整 CLI：`lerobot-crp-tele-dual --help` / `lerobot-crp-record-dual --help`。

### 核心目录

| 路径 | 作用 |
|------|------|
| `src/lerobot/scripts/lerobot_crp_tele_dual.py` | 遥操作 CLI |
| `src/lerobot/scripts/lerobot_crp_record_dual.py` | 录制 CLI |
| `src/lerobot/scripts/lerobot_crp_replay_dual.py` | 数据集 GJ 回放 CLI |
| `src/lerobot/scripts/lerobot_crp_deploy_dual.py` | ACT 等策略 GJ 部署 CLI |
| `src/lerobot/scripts/crp_gp/deploy.py` | 策略加载 + deploy 主循环 |
| `src/lerobot/scripts/crp_gp/` | GP 环、录制、配置、Orbbec 探测 |
| `src/lerobot/robots/crp_arm_dual/` | CRP 双臂 Robot + UI 探针 |
| `third_party/CrpRobotPy/` | 厂商 SDK（含 `set_GJs_second`） |
| `scripts/setup_python_env.sh` | Python 3.12 环境一键安装 |

---

## 环境安装

| 组件 | 说明 |
|------|------|
| Python | **≥ 3.12** |
| LeRobot | 本仓库 `pip install -e .` |
| CrpRobotPy / Patch | `third_party/CrpRobotPy/`；`bash build_patch.sh` |
| SO101 URDF | `third_party/SO101/so101_new_calib.urdf` |

**推荐一键：**

```bash
cd ~/lerobot
bash scripts/setup_python_env.sh
conda activate lerobot
pip install -e ".[feetech,dataset]"
pip install 'lerobot[intelrealsense]' 
bash third_party/CrpRobotPy/build_patch.sh
```

**部署注意：** 只部署厂商 `CrpRobotPy.so`；**勿覆盖** vendor `libRobotService.so`。`license.key` 须与 vendor 库配对。

---

## 开录前检查

按顺序做（USB 重插后必重做相机项）：

1. **Python / CLI：** `lerobot-crp-tele-dual --help`、`lerobot-crp-record-dual --help`、`lerobot-crp-replay-dual --help`、`lerobot-crp-deploy-dual --help`（若无命令：`pip install -e .`）
2. **CRP 网络：** `ping -c 2 192.168.0.100` 与 `.101`
3. **SO101 串口：** `ls -l /dev/ttyACM0 /dev/ttyACM1`；用户在 `dialout` 组；必要时 `sudo chmod 666 /dev/ttyACM*`
4. **标定：** `ls ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/{1,2}.json`
5. **CrpRobotPy + UI 探针：** 从 `~/python_C++/CrpRobotPy` 编译并部署 `CrpRobotPy.so`（含 `set_GJs_second`）；旧 SDK 才需 `bash third_party/CrpRobotPy/build_patch.sh`；`bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh`
6. **相机：**
   - **枚举：** `lerobot-find-cameras opencv`（含 Orbbec 各 `/dev/video*`）/ `lerobot-find-cameras realsense`
   - **实时预览：** `lerobot-camera-stream realsense` / `lerobot-camera-stream opencv`

---

## 遥操作

```bash
lerobot-crp-tele-dual
```

流程：UI 探针 → 伺服上电 → **5s 倒计时** → GP 对齐 → **80 Hz** 主环（默认不接相机）。

**与录制不同的常用参数：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `--fps` | `80` | 主控制环 |
| `--gp_send_fps` | `50` | GP 下发频率 |
| `--gp_align_delay_s` | `5` | 对齐前等待 |
| `--gp_position_step_mm` | `80` | XY 步进限幅（0=关） |
| `--gripper_ui_probe` | `true` | UI56–58 子进程 |
| `--left.wrist_flex_sign` / `--right.wrist_flex_sign` | `0` | 见上表；需 J4 控倾斜时：`--left.wrist_flex_sign=1 --right.wrist_flex_sign=1` |

---

## 数据采集

```bash
lerobot-crp-record-dual \
  --dataset.repo_id=user/20260610_gyl_1 \
  --dataset.num_episodes=3
```

**热键（须焦点在录制终端）：** **→** / **d** 结束本集并保存；**←** / **a** 重录；**Esc** 停止全部。

**集间：** save 后默认 **reset 30s**（最后 10s 倒计时）；**→** / **d** 可提前开下一集（会先 realign）。

**续录：** `lerobot-crp-record-dual --resume=true --dataset.repo_id=...`

### Schema 简表

| | 主要字段 |
|--|----------|
| **Observation** | `left/right_j1–j6.pos`；`left/right_ui56–58`（夹爪反馈）；`observation.images.*` |
| **Action** | 同上关节角（executed）；`left/right_ui50`（夹爪指令） |
| **元数据** | `fps=16`；`robot_type=crp_arm_dual`；`task=assemble`（可改） |

改相机 key/数量须换 `repo_id`，不可与旧数据集 `--resume` 混用。

### 数据集路径

默认：`~/.cache/huggingface/lerobot/<repo_id>/`（`meta/`、`data/`、`videos/`）。自定义：`--dataset.root=...`

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("user/20260610_gyl_1")
print(ds[0]["observation.images.top"].shape)
```

**上传 Hub（可选，默认不上传）：** 录完后 `LeRobotDataset("user/...").push_to_hub()`；训练可不联网，本地路径即可。

---

## 训练

```bash
pip install -e ".[training]"
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=user/20260610_gyl_1 \
  --output_dir=outputs/train/act_crp_dual \
  --job_name=act_crp_dual \
  --policy.device=cuda \
  --steps=100000 \
  --batch_size=8 \
  --eval.n_episodes=0
```

**注意：** 相机 key/数量须与数据集一致；`meta/info.json` 中 `fps` 须与采集一致（默认 16）。更多策略与参数见 [LeRobot 官方文档](https://huggingface.co/docs/lerobot) 与 `docs/source/policy_act_README.md`。

---

## 回放与策略部署

**勿**对真机使用 `lerobot-deploy --robot.type=crp_arm_dual` 或 `lerobot-replay --robot.type=crp_arm_dual`：通用 deploy 未走 GJ10/GJ20 + ui50 路径。

### 真机前流程（replay / deploy 共用）

1. 示教 GJ 程序 **STOP**；先停掉 `lerobot-crp-tele-dual` / `lerobot-crp-record-dual`
2. 确认 `export ORBBEC_PATH=/dev/videoN`（与录制一致）
3. 运行 CLI → PC **seed GJ10/GJ20** → **5s 倒计时** 内左右示教器按 **START**（绿灯常亮）
4. 无图形界面时无法用键盘提前退出，可用 **Ctrl+C**；有 `pynput` 时 **→** 可提前结束 deploy

### GJ 数据集回放

读 dataset 的 14 维 action（12 关节 + 左右 ui50），经 `CrpDualGjExecutor` 写 GJ。**用于验证 GJ/夹爪通路是否与录制一致。**

```bash
lerobot-crp-replay-dual \
  --dataset.repo_id=user/20260615_gyl_merged \
  --dataset.episode=0 \
  --fps=8 \
  --max_joint_delta_deg=5 \
  --speed_ratio=15 \
  --gp_align_delay_s=5 \
  --robot.ip1=192.168.0.100 \
  --robot.ip2=192.168.0.101
```

| 参数 | 说明 |
|------|------|
| `--dataset.repo_id` / `--dataset.episode` | 本地数据集与 episode 索引 |
| `--dataset.fps` | 回放目标 Hz；省略则用数据集 meta（通常 16） |
| `--max_joint_delta_deg` | 每 tick 单关节最大变化（°）；`null` 关闭裁剪 |
| `--speed_ratio` | CRP `set_speed_ratio`；越大 GJ 执行越快 |
| `--gp_align_delay_s` | 启动前倒计时，留时间按示教 START |

### 策略部署（ACT → 真机）

闭环：**观测（关节 + UI56–58 + 三相机）→ ACT → GJ10/GJ20 + ui50**。与 replay 共用同一执行器，动作来自策略而非数据集。

**Checkpoint 路径** 必须指向 `.../pretrained_model`（含 `config.json`、`train_config.json`、`model.safetensors`），不是 `outputs/train/.../` 根目录。

**首次 smoke（60s、保守）：**

```bash
lerobot-crp-deploy-dual \
  --policy.path=outputs/train/act_20260615_gyl/checkpoints/last/pretrained_model \
  --policy.n_action_steps=1 \
  --single_task=assemble \
  --fps=4 \
  --duration_s=60 \
  --max_joint_delta_deg=3 \
  --speed_ratio=10 \
  --gp_align_delay_s=5 \
  --robot.ip1=192.168.0.100 \
  --robot.ip2=192.168.0.101 \
  --play_sounds=false
```

**当前推荐（对准/合爪时机，仍须现场确认安全）：**

```bash
lerobot-crp-deploy-dual \
  --policy.path=outputs/train/act_20260615_gyl/checkpoints/last/pretrained_model \
  --policy.n_action_steps=1 \
  --single_task=assemble \
  --fps=8 \
  --duration_s=300 \
  --max_joint_delta_deg=10 \
  --speed_ratio=25 \
  --gp_align_delay_s=5 \
  --robot.ip1=192.168.0.100 \
  --robot.ip2=192.168.0.101 \
  --play_sounds=false
```

#### Deploy 参数说明

| 参数 | 作用 |
|------|------|
| `--policy.path` | ACT checkpoint 目录（`.../pretrained_model`） |
| `--policy.n_action_steps=1` | **建议 deploy 使用**：每 tick 根据新观测重算一步；默认 100 步队列 + 低 fps 会导致合爪时机严重偏移 |
| `--single_task` | 与录制 `--dataset.single_task` 一致（默认 `assemble`） |
| `--fps` | 目标控制频率；实际常低于设定（相机 + GPU 推理耗时）。训练 fps=16，可逐步试 8→12→16 |
| `--duration_s` | 最长运行秒数；**到点即停，不检测是否抓成功** |
| `--max_joint_delta_deg` | 每 tick 每关节相对当前角的最大变化（°）；过大动作快但需确认安全；`null` 关闭 |
| `--speed_ratio` | 示教/GJ 程序执行速度倍率（SDK `set_speed_ratio`） |
| `--gp_align_delay_s` | 启动倒计时，便于按示教 START |
| `--robot.ip1` / `--robot.ip2` | 左/右 CRP IP；仅改 IP 时 CLI 会自动恢复默认三相机 |
| `--play_sounds=false` | 关闭 TTS 提示 |

**Deploy 未写 `--robot.cameras` 时** 默认：`top`（Orbbec）+ 双 RealSense 腕部，须与训练数据集 camera key 一致。

**日志：** 正常循环会出现 `deploy tick=N left Δmax=... right Δmax=... sent_keys=14`；结束时 `Deploy finished: N ticks`。

### 离线可视化

```bash
lerobot-dataset-viz --repo-id user/20260615_gyl_merged --episode-index 0
```

---

## 已知限制与排查

| 现象 | 可能原因 | 建议 |
|------|----------|------|
| `lerobot-crp-deploy-dual: 未找到命令` | 未 editable 安装 | `pip install -e .` 或 `python -m lerobot.scripts.lerobot_crp_deploy_dual` |
| GJ init 后立即退出、无 `deploy tick` | 曾误用 `make_default_processors` 解包顺序（已修）；或 `cameras: {}` | 确认 config 里 `cameras` 非空；更新到最新代码 |
| 能到物体附近但抓不准 | 策略泛化、左臂几乎不动、场景与训练不一致 | 先试 `--policy.n_action_steps=1`；对比 **replay 同 episode** 是否准 |
| 合爪总是偏早/偏晚 | ACT 默认 `n_action_steps=100` 且 deploy fps ≠ 训练 16fps | deploy 设 `--policy.n_action_steps=1` |
| 没抓到也继续下一步 | **设计如此**：`duration_s` 开环，无 ui56/力矩成功检测 | 需额外开发抓取成功分支（暂未实现） |
| 左臂 Δmax≈1°、仅右臂大动 | 策略输出或数据偏置 | 补数据 / 检查示范是否双手配合 |
| 实际 tick 远低于 `--fps` | 三相机读取 + ACT 推理慢 | 正常；提高 fps 前先确认 GPU 与 USB3 |
| Headless 无键盘退出 | 缺 `pynput` | `pip install pynput` 或 `Ctrl+C` |

**诊断顺序：** replay 同 episode 能抓 → 策略/时序问题；replay 也不准 → 摆放、夹爪 ui50、示教程序、GJ 通路。

---

## 后续待办明细

**阶段 6 — GJ 回放：**

- [x] 独立 `lerobot-crp-replay-dual`：dataset action → GJ10/GJ20 + ui50
- [x] 安全：`max_joint_delta_deg`、`speed_ratio`
- [ ] 实现 `CRPArmDual.send_action()`（供通用 `lerobot-replay` 使用，非当前主路径）

**阶段 7 — 策略部署：**

- [x] `lerobot-crp-deploy-dual`：Policy → GJ/UI50（与训练 action schema 一致）
- [x] 相机 obs 与训练 key 对齐（top + 双腕）
- [ ] 抓取成功检测 / 失败重试 / 提前终止
- [ ] `temporal_ensemble_coeff` 或更高 fps 下的系统调参文档

**阶段 8 — 训练与效果：**

- [x] `lerobot-train --policy.type=act` 在 merged 数据集上 smoke
- [ ] 离线对比 policy 预测 vs 数据集（公平 eval 需每帧 `policy.reset()`）
- [ ] 相机数变更时重新采集或 mask 策略
- [ ] DAgger / 增广数据提升抓取成功率
