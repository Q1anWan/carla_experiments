# CARLA 路径编辑器与遥测可视化工具 — 技术细节

## 1. 系统架构

```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Scene Edit  │────▶│  Plan Build   │────▶│   Validate   │────▶│    Render    │
│ (Editor GUI) │     │ (Trajectory)  │     │ (Constraints) │     │  (CARLA 3D) │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                    │                    │
  scene_edit.json      plan.json          validation.json     master_video.mp4
  (稀疏关键帧)        (密集轨迹 @50Hz)     (约束检查报告)       telemetry.json/csv
```

### 设计原则
- **确定性回放**：所有 Actor 使用 TeleportFollower 逐帧传送，不依赖物理引擎
- **时间参数化轨迹**：所有运动定义为 t 的函数，确保视频/遥测时间对齐
- **SAE J670 坐标系**：遥测使用车辆相对坐标（Y轴正方向=左侧，与 CARLA 相反）
- **惰性车道吸附**：基于空间哈希索引的交互式编辑辅助

---

## 2. 交互式编辑器 (interactive_editor.py)

### 2.1 核心数据结构

```python
@dataclass
class Keyframe:
    t: float        # 时间 (秒)
    x: float        # X 坐标 (米)
    y: float        # Y 坐标 (米)
    v: Optional[float] = None  # 速度提示 (m/s)

@dataclass
class ActorState:
    actor_id: str       # 唯一标识符
    kind: str           # "vehicle" | "walker"
    role: str           # "ego" | "npc"
    blueprint: str      # CARLA 蓝图 ID
    controller: str     # "teleport"
    keyframes: List[Keyframe]  # 轨迹控制点（稀疏）
    color: str          # 可视化颜色
    # Matplotlib artist 引用
    line: Any = None
    scatter: Any = None
    ghost: Any = None
    conflict_scatter: Any = None
    infeasible_scatter: Any = None
    ttc_scatter: Any = None
    kinematic_scatter: Any = None

@dataclass
class SceneEdit:
    version: str        # "0.1"
    episode_id: str
    town: str           # CARLA 地图名
    dt: float           # 仿真步长 (0.05s)
    duration: float     # 场景时长 (秒)
    map_dir: str        # 地图数据路径
    seed: int
    actors: List[ActorState]
    events: List[EventMarker]
```

### 2.2 UI 布局 (Matplotlib Widgets)

```
┌─────────────────────────────────────┬──────────────────────┐
│                                     │ [Actor Radio Buttons] │
│                                     ├──────────┬───────────┤
│            ax_map                   │ Add KF   │ AddEvt    │
│         (2D Scene)                  │ Move KF  │ DelEvt    │
│       65% × 85%                     │ Del KF   │ Undo      │
│                                     │ Snap     │ Redo      │
│  - 车道中心线 (蓝色)                │ Analyze  │ Export    │
│  - 交叉口区域 (棕色填充)            │ Save     │ Reset     │
│  - Actor 轨迹 (彩色线+散点)         ├──────────┴───────────┤
│  - 选中标记 (黄色空心圆)            │ [Event Type Input]   │
│  - Ghost 位置 (半透明散点)           │ [Event Action Input] │
│                                     │ [KF Time Input]      │
│                                     │ [KF Speed Input]     │
│                                     │ [New Actor Input]    │
│                                     │ [Add Actor] [Del]    │
├─────────────────────────────────────┤                      │
│ [═══ Time Slider (0 - duration) ══] │                      │
├─────────────────────────────────────┤                      │
│ Status: Mode=add | Snap=OFF        │                      │
└─────────────────────────────────────┴──────────────────────┘
  Figure: 14×10 inches
```

### 2.3 编辑模式

| 模式 | 快捷键 | 行为 |
|------|--------|------|
| Add  | `a`    | 点击地图 → 在 slider 时间处创建关键帧 |
| Move | `m`    | 点击关键帧 → 拖拽到新位置 |
| Delete | `d`  | 点击关键帧 → 删除 |
| Select | `s`  | 点击关键帧 → 选中并显示属性 |

其他快捷键：
- `Space` / `空格`: 运行分析 (Analyze)
- `Ctrl+Z`: 撤销
- `Ctrl+Y` / `Ctrl+Shift+Z`: 重做
- `Delete` / `Backspace`: 删除选中关键帧
- `t`: 控制台打印关键帧表格
- `h`: 显示帮助
- `r`: 重置视图

### 2.4 车道吸附算法 (MapIndex)

```python
class MapIndex:
    """空间哈希索引，用于 O(1) 最近邻查询"""

    def __init__(self, lane_points, cell_size=5.0):
        # 将所有车道中心线点按 5m 网格分桶
        self._cells: Dict[Tuple[int, int], List[Tuple[x, y, lane_id]]]

    def snap_to_centerline(self, x, y, max_radius=6):
        """扩展半径搜索最近车道中心线点"""
        # 1. 计算查询点所在网格单元 (cx, cy)
        # 2. 从 radius=0 开始，逐圈扩展搜索
        # 3. 检查 (2r+1)² - (2r-1)² 个单元中的所有点
        # 4. 返回 (snap_x, snap_y, offset_distance)
        # 搜索范围: 最大 6×5m = 30m
```

**性能特征**: 网格大小 5m，最大搜索半径 6 格 = 30m。对于 Town05 约 10,000 车道点，建索引 < 10ms，查询 < 0.1ms。

### 2.5 轨迹构建算法 (_build_trajectory)

```
输入: 稀疏关键帧 [(t₀,x₀,y₀,v₀), ..., (tₙ,xₙ,yₙ,vₙ)]
输出: 密集轨迹 [TrajectoryPoint] @ dt=0.05s

步骤:
1. 边界补齐: 确保 t=0 和 t=duration 存在关键帧
2. 逐帧插值 (frame = 0, 1, ..., duration/dt):
   t = frame × dt
   找到包围关键帧 k₀, k₁ (k₀.t ≤ t ≤ k₁.t)
   α = (t - k₀.t) / (k₁.t - k₀.t)
   x = k₀.x + (k₁.x - k₀.x) × α    ← 线性插值
   y = k₀.y + (k₁.y - k₀.y) × α
3. 航向计算: yaw = atan2(Δy, Δx) × 180/π
4. 速度计算: v = √(Δx² + Δy²) / dt
5. 加速度: a = (v[i] - v[i-1]) / dt
6. 车道匹配: lane_id = map_index.nearest_lane(x, y)
```

**已知局限**:
- 线性插值在弯道处产生路径偏差（典型误差 1-10m）
- 速度在关键帧间不连续过渡
- 无曲率约束，可能产生物理上不可行的急转弯

### 2.6 分析功能 (_analyze)

| 检查项 | 阈值 | 可视化 |
|--------|------|--------|
| 超速 | v > 30 m/s (108 km/h) | 紫色三角 |
| 纵向加速 | a > 8.0 m/s² | 紫色三角 |
| 纵向减速 | a < -10.0 m/s² | 紫色三角 |
| 横向加速 | κ×v² > 5.0 m/s² | 紫色三角 |
| 偏离车道 | 车辆 > 1.8m, 行人 > 2.8m | 橙色菱形 |
| Actor 碰撞 | 距离 < 3.0m | 红色圆 |
| TTC 警告 | TTC < 3.0s | 粉色叉 |

**TTC 计算**:
```
对每对 Actor (A, B) 在每帧:
  Δpos = pos_A - pos_B
  Δvel = vel_A - vel_B
  approach = -(Δpos · Δvel) / |Δpos|
  if approach > 0:
    TTC = |Δpos| / |Δvel|
    if TTC < 3.0s: 标记警告
```

### 2.7 撤销/重做

```python
_undo_stack: List[str]  # JSON 序列化的场景状态，最大 50 条
_redo_stack: List[str]

def _push_undo():
    state = json.dumps(_scene_to_dict())
    _undo_stack.append(state)  # 记录当前状态
    _redo_stack.clear()        # 新操作清除 redo

def _undo():
    current = json.dumps(_scene_to_dict())
    _redo_stack.append(current)
    state = _undo_stack.pop()
    _clear_actor_artists()     # 清除旧 matplotlib artists
    _restore_from_dict(json.loads(state))
```

---

## 3. CLI 管线 (cli.py)

### 命令总览

```bash
# 子命令模式
python -m carla_experiment_client.cli {map|plan|validate|render|pipeline|preview|editor|suite}

# 交互菜单模式
python ./cli.py --menu    # 或无参数运行
```

| 命令 | 功能 | 需要 CARLA | 输入 | 输出 |
|------|------|-----------|------|------|
| map | 导出地图资产 | 是 | 地图名 | map_graph.json, *.geojson |
| plan | 编译 plan.json | 是 | episode YAML | plan.json |
| validate | 验证 plan | 是 | plan.json | validation_report.json |
| render | 渲染视频 | 是 | plan.json | video, telemetry |
| pipeline | plan+validate+render | 是 | episode YAML | 全部输出 |
| preview | 2D 预览图 | 否 | plan.json + map | PNG |
| editor | 交互式编辑器 | 否 | scene_edit.json + map | scene_edit.json, plan.json |
| suite | 批量运行 | 是 | suite 配置 | 每场景全部输出 |

### 管线数据流

```
scene_edit.json ──editor──▶ plan.json ──validate──▶ report.json
                                      ──render────▶ master_video.mp4
                                                    telemetry.json
                                                    telemetry.csv
                                                    events.json
                                                    frames/
```

---

## 4. 数据格式

### 4.1 scene_edit.json (编辑器格式)

```json
{
  "version": "0.1",
  "episode_id": "lane_change_cut_in",
  "town": "Town05",
  "dt": 0.05,
  "duration": 60.0,
  "map_dir": "data/maps/Town05",
  "seed": 0,
  "actors": [
    {
      "id": "ego",
      "kind": "vehicle",
      "role": "ego",
      "blueprint": "vehicle.tesla.model3",
      "controller": "teleport",
      "color": "#d1495b",
      "keyframes": [
        {"t": 0.0, "x": 15.0, "y": 200.0, "v": 10.0},
        {"t": 30.0, "x": 150.0, "y": 168.0, "v": 8.0}
      ]
    }
  ],
  "events": [
    {
      "t_event": 8.0,
      "event_type": "lane_change",
      "expected_action": "lane_change",
      "audio_id": "lane_change_00",
      "t_voice_start": 5.0,
      "t_robot_precue": 4.5
    }
  ]
}
```

### 4.2 plan.json (渲染器格式)

```json
{
  "version": "0.1",
  "episode_id": "...",
  "town": "Town05",
  "seed": 0,
  "dt": 0.05,
  "duration": 60.0,
  "actors": [
    {
      "actor_id": "ego",
      "kind": "vehicle",
      "role": "ego",
      "blueprint": "vehicle.tesla.model3",
      "controller": "teleport",
      "trajectory": [
        {"t": 0.0, "x": 15.0, "y": 200.0, "yaw": -18.1, "v": 10.0, "a": 0.0, "lane_id": -1},
        {"t": 0.05, "x": 15.19, "y": 199.94, "yaw": -18.1, "v": 10.01, "a": 0.2, "lane_id": -1}
        // ... 每帧一个点 (dt=0.05s → 1200帧/分钟)
      ]
    }
  ],
  "events_plan": [...]
}
```

### 4.3 telemetry.json (遥测格式)

```json
{
  "metadata": {
    "fps": 20,
    "total_frames": 1200,
    "coordinate_system": "SAE_J670",
    "units": {"position": "meters", "velocity": "m/s", "angles": "degrees"}
  },
  "frames": [
    {
      "frame": 0,
      "t_sim": 0.0,
      "dt": 0.05,
      "ego": {
        "position": {"x": -34.9, "y": -87.9, "z": 0.0},
        "velocity": {"vx": 4.6, "vy": 0.0, "vz": 0.0},
        "acceleration": {"ax": 0.0, "ay": 0.0, "az": 0.0},
        "orientation": {"roll": 0.0, "pitch": 0.08, "yaw": 0.09},
        "speed": 4.59,
        "control": {"throttle": 0.85, "brake": 0.0, "steer": 0.004}
      },
      "actors": [
        {
          "id": 1439,
          "type": "vehicle",
          "type_id": "vehicle.citroen.c3",
          "role_name": "cut_in_vehicle",
          "position": {"x": -24.0, "y": -84.4, "z": 0.04},
          "rotation": {"roll": 0.0, "pitch": 0.05, "yaw": 0.08},
          "distance_to_ego": 11.45,
          "speed": 1.49
        }
      ]
    }
  ]
}
```

### 4.4 telemetry.csv (表格格式, 仅 ego)

```
frame, t_sim, t_world, dt, world_x, world_y, world_z,
vx, vy, vz, ax, ay, az,
roll_rate, pitch_rate, yaw_rate, roll, pitch, yaw,
speed, throttle, brake, steer
```

**SAE J670 坐标注意**: `vy` = CARLA_Y × (-1)，Y轴正方向=车辆左侧

### 4.5 地图数据

**map_graph.json** — 拓扑图:
```json
{
  "map": "Carla/Maps/Town05",
  "waypoint_distance": 2.0,
  "lanes": [
    {
      "id": "2358:0:-1",
      "road_id": 2358,
      "lane_type": "Driving",
      "left_lane": null,
      "right_lane": "2358:0:-2",
      "successors": ["27:0:-1"],
      "centerline": [[15.17, 201.64], [17.42, 201.50], ...]
    }
  ]
}
```

**lane_centerlines.geojson** — GeoJSON LineString 集合
**junction_areas.geojson** — GeoJSON Polygon 集合 (交叉口边界)

---

## 5. 渲染器 (renderer.py)

### 渲染流程

```
1. 加载 plan.json + globals.yaml
2. 连接 CARLA 服务器
3. 加载目标地图 (Town05 等)
4. 配置世界 (sync=True, fixed_delta=0.05s)
5. 生成 Actor:
   - 从首帧轨迹点获取 spawn 位置
   - 投影 Z 坐标到道路表面 + 0.2m 偏移
   - 创建蓝图并设置 role_name
6. 构建 TeleportFollower 实例
7. 初始化 CameraRecorder + TelemetryRecorder
8. 主循环 (每帧):
   - follower.apply(frame_idx)  → 传送 Actor 到 trajectory[frame_idx]
   - world.tick()               → 同步步进
   - 录制相机图像 → frames/NNNN.png
   - 录制遥测数据
9. 保存输出:
   - frames/ → ffmpeg → master_video.mp4
   - telemetry.json, telemetry.csv
   - events.json (含 ego 状态快照)
   - run_metadata.json
```

### Spawn 策略

```python
def _first_transform(plan, actor_plan, map_obj):
    point = actor_plan.trajectory[0]
    loc = carla.Location(x=point.x, y=point.y, z=0.0)
    wp = map_obj.get_waypoint(loc, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
    z = wp.transform.location.z if wp else 0.0
    return carla.Transform(
        carla.Location(point.x, point.y, z + 0.2),
        carla.Rotation(yaw=point.yaw)
    )
```

---

## 6. 遥测可视化对比工具

### 对比图生成逻辑 (comparison.png)

每个场景生成 3 子图:

| 子图 | 内容 | 数据源 |
|------|------|--------|
| 左: XY 轨迹 | 实线=CSV 遥测轨迹, 虚线+圆点=keyframe 插值 | telemetry.csv + scene_edit.json |
| 中: 速度曲线 | 实线=CSV ego speed, 虚线=keyframe speed, 竖线=事件 | telemetry.csv + scene_edit.json + events.json |
| 右: 插值误差 | ego keyframe 线性插值 vs CSV 实际轨迹的位置偏差 | 两者差值 |

### 当前插值误差统计

| 场景 | 平均误差 | 最大误差 | 误差来源 |
|------|---------|---------|---------|
| lane_change_cut_in | 1.26m | 5.63m | 弯道处线性插值偏差 |
| unprotected_left_turn | 2.27m | 8.28m | 左转弯道 |
| yield_to_emergency | 1.60m | 5.18m | 变道/避让弯道 |
| red_light_conflict | 1.13m | 7.76m | 交叉口转弯 |
| pedestrian_emerge | 2.56m | 10.49m | 避让行人急转 |
| highway_merge | 1.56m | 6.67m | 匝道合流弧线 |

---

## 7. 遥测 → 场景设计转换工具

### 转换流程

从 `runs/final_fix_validation/` 验证遥测转换为 `outputs/scene_designs/` 编辑器格式:

```
telemetry.json ─┐
telemetry.csv  ─┼──▶ scene_edit.json  (稀疏关键帧)
events.json    ─┘    comparison.png   (验证对比图)
```

### 转换步骤

1. **Ego 轨迹提取**: 从 `telemetry.csv` 读取 `world_x`, `world_y`, `speed` 列，每 10s 采样一个关键帧
2. **NPC 识别**: 从 `telemetry.json` 按 `role_name` 筛选关键 NPC (cut_in_vehicle, oncoming, emergency, walker 等)，排除 background traffic
3. **NPC 关键帧**: 根据 NPC 出现时间和位置变化采样 5-8 个关键帧
4. **事件保留**: 从 `events.json` 提取事件时间和类型
5. **Actor ID 去重**: 多个 "autopilot" 角色重命名为 `autopilot_2`, `autopilot_3`

### 验证对比图 (comparison.png)

每场景生成 3 子图:

```python
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

# 左图: XY 轨迹叠加
ax1.plot(csv_x, csv_y, 'b-', label='CSV telemetry')
ax1.plot(kf_x, kf_y, 'r--o', label='Keyframe interp')

# 中图: Ego 速度对比
ax2.plot(csv_t, csv_speed, 'b-', label='CSV speed')
ax2.plot(kf_t, kf_speed, 'r--', label='KF speed')
for evt in events:
    ax2.axvline(evt.t, color='green')

# 右图: 插值误差曲线
ax3.plot(t_common, error_m, 'k-')
ax3.axhline(mean_err, color='orange', linestyle='--')
ax3.set_title(f'mean={mean_err:.2f}m, max={max_err:.2f}m')
```

---

## 8. 已知局限与优化方向

### 编辑器
0. **视图初始化**: 已修复 — 加载场景后自动调用 `_reset_view()` 适配所有 Actor 范围
1. **线性插值**: 弯道处误差大，需要三次样条或 Bézier 曲线插值
2. **无曲率约束**: 可能生成物理不可行的急转弯轨迹
3. **固定时间步长**: 无法自适应调整弯道密度
4. **2D 编辑**: 无高程信息（Z 坐标由渲染器投影）
5. **单一地图数据**: Town03/04 场景需要先导出地图资产

### 遥测可视化
1. **仅支持 ego CSV**: NPC 轨迹需从 telemetry.json 提取
2. **无动画播放**: 当前仅生成静态对比图
3. **无统计面板**: 缺少速度/加速度分布直方图

### 渲染器
1. **Spawn 失败无重试**: 位置冲突时直接报错
2. **无碰撞检测**: 渲染过程中不检测物理碰撞
3. **版本兼容**: 客户端/服务端 API 版本不匹配时仅警告

---

## Appendix: Implementation Pointers

Quick reference mapping each documentation section to source code entry points.

| Doc Section | Source File | Key Symbols |
|---|---|---|
| §1 Architecture | `cli.py:117` | `do_pipeline()` — orchestrates plan → validate → render |
| §2.1 Data Structures | `editor/interactive_editor.py:26-62` | `Keyframe`, `ActorState`, `EventMarker`, `SceneEdit` |
| §2.2 UI Layout | `editor/interactive_editor.py:488-597` | `SceneEditor._build_ui()`, `ax_map`, `ax_slider`, `ax_status` |
| §2.3 Edit Modes | `editor/interactive_editor.py:709-721` | `_set_mode_add()`, `_set_mode_move()`, `_set_mode_delete()` |
| §2.4 MapIndex | `editor/interactive_editor.py:74-112` | `MapIndex.__init__()`, `nearest_lane()`, `snap_to_centerline()` |
| §2.5 Trajectory Build | `editor/interactive_editor.py:223` | `_build_trajectory(keyframes, dt, duration, map_index)` |
| §2.6 Analysis | `editor/interactive_editor.py:1218` | `SceneEditor._analyze()` |
| §2.7 Undo/Redo | `editor/interactive_editor.py:727-836` | `_push_undo()`, `_undo()`, `_redo()`, `_scene_to_dict()`, `_restore_from_dict()` |
| §3 CLI | `cli.py:368-452` | `build_parser()`, `main()`, `do_map/plan/validate/render/pipeline/editor/suite` |
| §4.1 scene_edit.json | `editor/interactive_editor.py:352,403` | `_load_scene()`, `_save_scene()` |
| §4.2 plan.json | `planning/trajectory_schema.py:128-171` | `Plan`, `ActorPlan`, `TrajectoryPoint`, `save_plan()`, `load_plan()` |
| §4.3 telemetry.json | `telemetry/recorder.py:63` | `TelemetryRecorder`, `TelemetryFrame`, `TelemetryConfig` |
| §4.4 telemetry.csv | `telemetry/recorder.py` + `telemetry/sae_j670.py` | `TelemetryRecorder.save()`, `SAEJ670Transformer` |
| §4.5 Map data | `planning/map_exporter.py:126` | `export_map()`, `MapExportConfig`, `_lane_uid()` |
| §5 Renderer | `render/renderer.py:123` | `render_plan()`, `_spawn_actor()`, `_first_transform()`, `_build_events()` |
| §5 Replay | `render/replay_controller.py:17` | `TeleportFollower`, `build_follower()` |
| §6 Comparison | — | **MISSING** — needs `tools/make_comparison.py` |
| §7 Conversion | — | **MISSING** — needs `tools/convert_telemetry_to_scene.py` |

All paths relative to `carla_experiment_client/`.
