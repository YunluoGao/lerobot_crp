# LeRobot + 双 CRP 臂遥操作与数据采集

## 目录

- [项目简介](#项目简介)
- [默认参数](#默认参数)
- [环境安装](#环境安装)
- [开录前检查](#开录前检查)
- [遥操作](#遥操作)
- [数据采集](#数据采集)
- [训练](#训练)
- [回放与推理（尚未支持）](#回放与推理尚未支持)
- [后续待办明细](#后续待办明细)

---

## 项目简介

将 LeRobot 适配到 **卡诺普（CRP）双 GP 从臂 + 双 SO101 主臂** 场景：遥操作、数据集录制、ACT 等策略训练。

| 能力 | CLI | 状态 |
|------|-----|------|
| 双主臂 → 双 CRP 遥操作 | `lerobot-crp-tele-dual` | ✅ 真机已验收 |
| 遥操作 + 数据集录制 | `lerobot-crp-record-dual` | ✅ 代码完成；真机待 USB 稳定验收 |
| 策略训练 | `lerobot-train` | ✅ 通用（须匹配 dataset schema） |
| 真机 replay / rollout | `lerobot-replay` / `lerobot-rollout` | ⏳ 需 GP 动作桥接（见下文） |

**常用命令：**

```bash
cd ~/lerobot
conda activate lerobot
sudo chmod 666 /dev/ttyACM*

lerobot-crp-tele-dual

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
```

默认值见 [`src/lerobot/scripts/crp_gp/config.py`](src/lerobot/scripts/crp_gp/config.py)。

---

## 默认参数

### 硬件默认值

| 项 | 默认值 | 说明 |
|----|--------|------|
| CRP 左 / 右 IP | `192.168.0.100` / `192.168.0.101` | `--robot.ip1` / `--robot.ip2` |
| SO101 左 / 右串口 | `/dev/ttyACM1` / `/dev/ttyACM0` | 标定 `1.json` / `2.json` |
| Top 相机 | **启动时自动探测 RGB 节点** | Orbbec Gemini 335；可用 `ORBBEC_PATH` 优先；`ORBBEC_AUTO_DISCOVER=0` 关闭 |
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
| `src/lerobot/scripts/crp_gp/` | GP 环、录制、配置、Orbbec 探测 |
| `src/lerobot/robots/crp_arm_dual/` | CRP 双臂 Robot + UI 探针 |
| `third_party/CrpRobotPy/` | 厂商 SDK + `build_patch.sh` |
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

1. **Python / CLI：** `lerobot-crp-tele-dual --help`、`lerobot-crp-record-dual --help`
2. **CRP 网络：** `ping -c 2 192.168.0.100` 与 `.101`
3. **SO101 串口：** `ls -l /dev/ttyACM0 /dev/ttyACM1`；用户在 `dialout` 组；必要时 `sudo chmod 666 /dev/ttyACM*`
4. **标定：** `ls ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/{1,2}.json`
5. **SDK 补丁 + UI 探针：** `bash third_party/CrpRobotPy/build_patch.sh`；`bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh`
6. **相机：**
   - **枚举：** `lerobot-find-cameras realsense` / `lerobot-find-cameras opencv` / `lerobot-find-cameras orbbec`
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

## 回放与推理（尚未支持）

`CRPArmDual.send_action()` 未实现，**勿**对真机运行 `lerobot-replay --robot.type=crp_arm_dual`。

**当前可用：** 数据集时间轴可视化

```bash
lerobot-dataset-viz --repo-id user/20260610_gyl_1 --episode-index 0
```

**待实现：** dataset action → GP + UI50 → `send_gp_tick`；完成后才可用 `lerobot-rollout` 真机部署。

---

## 后续待办明细

**阶段 6 — GP 回放：**

- [ ] 实现 `CRPArmDual.send_action()`：关节 + UI50 → GP + `set_ui_*`
- [ ] 或独立 `lerobot-crp-replay-dual`：读 dataset action → GP tick
- [ ] 安全：速度限幅、工作空间、急停

**阶段 7 — 策略部署：**

- [ ] Policy 输出 → GP/UI50 桥接（与训练 action schema 一致）
- [ ] 相机 obs 与训练时 key 对齐（top-only vs 三相机）
- [ ] `lerobot-rollout` 或专用 rollout CLI

**阶段 8 — 训练：**

- [ ] `lerobot-train --policy.type=act` 在 top-only 数据集上 smoke
- [ ] 对比 dataset-viz 与 policy 预测（离线）
- [ ] 相机数变更时重新采集或 mask 策略
