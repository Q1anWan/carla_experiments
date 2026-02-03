# AI Agent 操作指南

本文档为 AI Agent（如 Claude Code、Cursor 等）提供标准化的操作规范。

## 环境配置

### 基础信息
```yaml
project_root: /home/qianwan/ws/hkustgz/CARLA_Latest/PythonAPI/examples/carla_experiment_client
conda_env: CARLA312
carla_server: 192.168.31.32:2000
tm_port: 8000
python_version: "3.12"
```

### 命令前缀
所有 Python 命令必须使用以下前缀：
```bash
conda run -n CARLA312 --no-capture-output python -m carla_experiment_client.cli <command>
```

或简写形式（需先激活环境）：
```bash
conda activate CARLA312
python -m carla_experiment_client.cli <command>
```

---

## CLI 命令参考

### 场景运行（V3 推荐）

```bash
# 快速测试 (640x360, 120s)
conda run -n CARLA312 --no-capture-output python -m carla_experiment_client.cli run-v3 \
  --scenario lane_change_cut_in:config_v3_quick \
  --out outputs/lane_change_cut_in/<test_name> \
  --host 192.168.31.32 \
  --allow-version-mismatch

# 高清渲染 (2560x1080 21:9, 120s)
conda run -n CARLA312 --no-capture-output python -m carla_experiment_client.cli run-v3 \
  --scenario lane_change_cut_in:config_v3_21x9_1080p \
  --out outputs/lane_change_cut_in/<render_name> \
  --host 192.168.31.32 \
  --allow-version-mismatch
```

**参数说明**：
| 参数 | 必需 | 说明 |
|------|------|------|
| `--scenario` | ✓ | 场景 ID 或 YAML 路径 (`scenario_id:config_name`) |
| `--out` | ✓ | 输出目录 |
| `--host` | - | CARLA 服务器地址 (默认读取 configs/client.yaml) |
| `--allow-version-mismatch` | - | 允许客户端/服务端版本不一致 |

### 传统工作流 (V1/V2)

```bash
# 完整流水线：编辑 → 计划 → 验证 → 渲染
python -m carla_experiment_client.cli pipeline --episode <episode_id>

# 单独步骤
python -m carla_experiment_client.cli plan --episode <episode_id>
python -m carla_experiment_client.cli validate --plan outputs/<episode_id>/plan.json
python -m carla_experiment_client.cli render --plan outputs/<episode_id>/plan.json
```

### 工具命令

```bash
# 遥测可视化（生成静态图片）
python -m carla_experiment_client.cli visualize \
  --run-dir outputs/lane_change_cut_in/<run_name>

# 遥测可视化（交互式查看器）
python -m carla_experiment_client.cli visualize \
  --run-dir outputs/lane_change_cut_in/<run_name> \
  --interactive

# 遥测对比分析
python -m carla_experiment_client.cli compare \
  --scene outputs/scene_designs/<scene>/scene_edit.json \
  --telemetry-csv outputs/<run>/telemetry.csv \
  --output outputs/<run>/comparison

# 遥测转场景
python -m carla_experiment_client.cli convert \
  --csv outputs/<run>/telemetry.csv \
  --json outputs/<run>/telemetry.json \
  --events outputs/<run>/events.json \
  --output outputs/<scene>/scene_edit.json

# 交互式编辑器
python -m carla_experiment_client.cli editor --scene outputs/scene_designs/<scene>/scene_edit.json
```

---

## 遥测可视化

### 方法 1: 使用内置查看器（交互式）

```bash
python -m carla_experiment_client.tools.telemetry_viewer \
  --plan outputs/<run>/plan.json \
  --telemetry outputs/<run>/telemetry.json
```

### 方法 2: Python 脚本生成静态图

```python
import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# 加载遥测数据
with open('outputs/<run>/telemetry.json') as f:
    data = json.load(f)

frames = data['frames']
t_sim = [f['t_sim'] for f in frames]
ego_x = [f['ego']['position']['x'] for f in frames]
ego_y = [f['ego']['position']['y'] for f in frames]
ego_speed = [f['ego']['speed_actor'] for f in frames]

# 创建图表
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. 轨迹图
axes[0, 0].scatter(ego_x, ego_y, c=t_sim, cmap='viridis', s=2)
axes[0, 0].set_title('Ego Trajectory')
axes[0, 0].set_aspect('equal')

# 2. 速度曲线
axes[0, 1].plot(t_sim, ego_speed)
axes[0, 1].set_title('Speed Profile')

# 3. 红绿灯状态（如有）
if 'traffic_lights' in frames[0]:
    # 提取红绿灯数据并绘制时序图
    pass

plt.tight_layout()
plt.savefig('outputs/<run>/telemetry_visualization.png', dpi=150)
```

---

## 输出目录结构

### V3 场景输出
```
outputs/lane_change_cut_in/<run_name>/
├── scenario.yaml           # 场景配置快照
├── run.log                  # 运行日志
├── master_video.mp4         # 渲染视频
├── telemetry.json           # 完整遥测 (含 traffic_lights)
├── telemetry.csv            # Ego 遥测 CSV
├── events.json              # 事件时间线
├── director_events.json     # Director 事件状态
├── gate_report.json         # 质量门禁报告
├── run_metadata.json        # 元数据
└── frames/                  # 原始帧 (可选)
```

---

## 常见任务示例

### 任务 1: 渲染测试视频并验证红绿灯

```bash
# 1. 运行场景
conda run -n CARLA312 --no-capture-output python -m carla_experiment_client.cli run-v3 \
  --scenario lane_change_cut_in:config_v3_quick \
  --out outputs/lane_change_cut_in/test_$(date +%Y%m%d_%H%M%S) \
  --host 192.168.31.32 \
  --allow-version-mismatch

# 2. 验证红绿灯状态
python3 -c "
import json
with open('outputs/lane_change_cut_in/<run>/telemetry.json') as f:
    data = json.load(f)
frame = data['frames'][100]
print('Traffic lights:', frame.get('traffic_lights', []))
"
```

### 任务 2: 生成高清视频

```bash
conda run -n CARLA312 --no-capture-output python -m carla_experiment_client.cli run-v3 \
  --scenario lane_change_cut_in:config_v3_21x9_1080p \
  --out outputs/lane_change_cut_in/hd_$(date +%Y%m%d_%H%M%S) \
  --host 192.168.31.32 \
  --allow-version-mismatch
```

### 任务 3: 分析遥测数据

```bash
# 提取关键指标
python3 -c "
import json
import numpy as np
with open('outputs/<run>/telemetry.json') as f:
    data = json.load(f)
speeds = [f['ego']['speed_actor'] for f in data['frames']]
print(f'Duration: {data[\"frames\"][-1][\"t_sim\"]:.1f}s')
print(f'Avg speed: {np.mean(speeds):.2f} m/s')
print(f'Max speed: {max(speeds):.2f} m/s')
"
```

---

## 错误处理

### 常见错误及解决方案

| 错误 | 原因 | 解决方案 |
|------|------|----------|
| `Version mismatch` | 客户端/服务端版本不一致 | 添加 `--allow-version-mismatch` |
| `time-out of 10000ms` | CARLA 服务器未响应 | 检查服务器状态，重启 CARLA |
| `Failed to spawn walker` | Walker 生成位置冲突 | 通常可忽略，不影响主场景 |
| `Actor already destroyed` | 清理时 Actor 已销毁 | 正常警告，无需处理 |

### 服务器连接检查

```bash
# 检查 CARLA 服务器连接
python3 -c "
import carla
try:
    client = carla.Client('192.168.31.32', 2000)
    client.set_timeout(5.0)
    print('Server version:', client.get_server_version())
except Exception as e:
    print('Connection failed:', e)
"
```

---

## 版本对照

| 版本 | 配置文件格式 | CLI 命令 | 特性 |
|------|-------------|----------|------|
| V1 | `scenario.yaml` | `run-scenario` | 帧号触发 |
| V2 | `scene_edit.json` | `pipeline` | 关键帧编辑 |
| V3 | `config_v3*.yaml` | `run-v3` | 条件 DSL |

---

## 文件路径约定

```
carla_experiment_client/
├── cli.py                          # 主 CLI 入口
├── configs/
│   ├── client.yaml                 # 服务器配置
│   └── globals.yaml                # 全局配置
├── scenarios/
│   └── lane_change_cut_in/
│       ├── config_v3.yaml          # V3 标准配置
│       ├── config_v3_quick.yaml    # V3 快速测试
│       └── config_v3_21x9_1080p.yaml # V3 高清
├── director/                       # V3 Director 模块
├── driver_backends/                # V3 驱动后端
├── tools/                          # 工具脚本
└── docs/
    ├── architecture.md             # 架构设计
    ├── technical_details.md        # 技术细节
    ├── V3.md                       # V3 专项文档
    └── AI_AGENT_GUIDE.md           # 本文档
```
