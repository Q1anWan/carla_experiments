下面是一份**“6 套场景（Suites P1–P6）完整详细设计文档”**（Markdown），目标是：
 
* **学术实验可控**：每位被试 6 个 Block（6 条件），每个 Block 使用 **1 套 Suite**（含 6 段 Episode），避免被试内重复观看同一片段。
* **工程可实现**：用**配置驱动（YAML/JSON）**描述一切；提供统一 Schema、可调参数、触发事件（events.json）规范、验证准则、失败准则、日志产物。
* **可维护可修改**：所有关键数值都集中在 `globals.yaml` 与每个 episode 的 `params` 里，调整不改代码。
* **直接给 Codex**：文末附录提供“项目级提示词 + 模块任务提示词 + 36 个 episode 配置清单模板”。

> 你可以把本文直接复制到你的 repo 的 `docs/scenario_design.md`。
> 文中默认 CARLA 0.9.16；Server 在 Win+3080，Client 在 Ubuntu（无 GPU）。

---

# Repo implementation notes (current)

- Code overview: `docs/architecture.md`.
- Runtime config uses `configs/client.yaml` + `configs/scenarios/*.yaml`.
- `configs/render_presets.yaml` provides `fast`, `fast_long`, `final`.
- Plan/validate/render pipeline is available via `cli.py` or `runner.py`, producing outputs under `outputs/<episode_id>/`.
- Interactive editor (Matplotlib) exports `scene_edit.json`, `plan.json`, and `events_plan.json`.
- `prewarm_seconds` ticks the world before recording to avoid initial drop.
- All actors spawn before recording; runtime relocation is disabled in 30s configs.

# CARLA 实验刺激场景库设计文档（Suites P1–P6）

## 0. 关键术语与实验结构（避免歧义）

* **Condition（条件）**：2×3=6（语音 V0/V1/V2 × 机器人 R0/R1）
* **Block（试验块）**：一个 Block 固定一个 Condition，并播放一套刺激材料；Block 结束后填一次问卷（信任+TLX）
* **Suite（场景套件）**：P1–P6 共 6 套；**每个 Block 使用一套 Suite**
* **Episode（场景片段）**：每套 Suite 含 6 个 Episode（T1–T6 六类典型情境），每段约 45–70s（建议标准 60s）
* **被试内重复规避**：每位被试 6 个 Block → 使用 P1..P6 各一次 → 共观看 36 个 Episode，**同一 Episode 不重复**

---

## 1. 设计目标与可调原则

### 1.1 目标

1. 生成可控、可复现、可批量的驾驶第一人称视频（无音轨母带）。
2. 为每个 Episode 输出 `events.json`：包含**关键决策点**（用于语音/机器人提示的统一触发）。
3. 通过 6 套 Suite 的“同构但不同实例”，避免学习效应，同时保持难度在可比范围内。

### 1.2 “同构”定义（跨 Suite 一致）

每套 Suite 必须包含以下 6 类 Episode（T1–T6）：

* **T1 Merge（合流/并道）**
* **T2 Lane Change / Overtake（变道/超车）**
* **T3 Signalized Intersection（信号灯路口通行）**
* **T4 Unprotected Left Turn（无保护左转）**
* **T5 Emergency Vehicle Yield（紧急车让行）**
* **T6 VRU Interaction（行人/骑行者等弱势交通参与者）**

### 1.3 “不重复”定义（被试内）

* 同一个被试在 6 个 Block 中**不得观看同一个 Episode 文件**（包括同一路段、同一触发点、同一 NPC 脚本、同一随机种子）。
* 允许“同类型重复”（例如每个 Suite 都有变道），但必须是不同实例（不同地图/路段/触发点/NPC 组合/seed）。

---

## 2. 总体工程架构（给 Codex 的落地目标）

### 2.1 项目目录建议

```
project/
  configs/
    globals.yaml
    suites/
      P1.yaml ... P6.yaml
    episodes/
      P1_T1_merge.yaml
      ...
      P6_T6_vru.yaml
  src/
    runner.py
    config_loader.py
    topology_query.py
    episode_builder.py
    sensors_recorder.py
    event_extractor.py
    mux_audio.py
    counterbalance.py
  assets/
    narration_texts.json   # What / What+Why 文本资产（离线生成）
    tts_audio/             # 语音 wav（离线 TTS 输出）
  outputs/
    suites/P1/...
  docs/
    scenario_design.md
```

### 2.2 两阶段实现策略（强烈推荐）

* **Stage A（先跑通）**：Ego 使用 CARLA TrafficManager autopilot（最稳定），保证场景触发、事件提取、录制链路、后期混音全部通。
* **Stage B（再替换）**：Ego 改为 Autoware Universe 控制（ROS2），保持同一套 episode 配置与事件触发逻辑；NPC 仍可用 TM 生成。

> 这能显著降低“Autoware 集成不稳定 → 全链路卡死”的风险。

---

## 3. 可复现性与运行规范（必须项）

### 3.1 同步仿真（Determinism）

全库统一要求：

* `world_settings.synchronous_mode = true`
* `world_settings.fixed_delta_seconds = 0.05`（建议 20 FPS tick；视频可 30 FPS 采样）
* Traffic Manager 同步：`tm.set_synchronous_mode(true)` 且与 fixed_delta 对齐
* 每个 Episode 固定 `seed`（写入 config 与产物元数据）

### 3.2 统一视觉规格

* 摄像头：驾驶位第一人称 RGB
* 分辨率：1920×1080（或 1280×720，建议统一）
* FOV：90（统一）
* 视频时长：`duration_s = 60`（±10 可接受，但建议统一）

### 3.3 产物规范（每个 Episode）

输出目录：`outputs/suites/<suite>/<episode>/`

* `master_video.mp4`（无音轨母带；或 `frames/*.png`）
* `events.json`（决策事件时间线）
* `run_metadata.json`（地图、版本、seed、参数快照、FPS、开始时间等）
* `tick_log.jsonl`（逐 tick 可选；建议至少记录触发窗口内的状态）

---

## 4. 配置 Schema（YAML）与可调参数

### 4.1 `globals.yaml`（全局默认）

```yaml
version: "0.1"
carla:
  host: "WIN_SERVER_IP"
  port: 2000
  timeout_s: 10
  fixed_delta_seconds: 0.05
  synchronous: true
  traffic_manager_port: 8000

recording:
  duration_s: 60
  video_fps: 30
  image_size: [1280, 720]
  fov: 90
  output_format: "mp4"   # mp4 or frames

weather:
  preset: "ClearNoon"    # 全库默认；可 per-suite override

ego:
  blueprint: "vehicle.tesla.model3"
  control_mode: "tm"     # tm or autoware
  target_speed_kmh: 45   # 城市默认
  safety:
    max_brake: 0.8

traffic:
  vehicles:
    enabled: true
    count: 25
    autopilot: true
    speed_delta_pct_mean: -5
    speed_delta_pct_std: 3
  walkers:
    enabled: true
    count: 40

events:
  voice_lead_time_s: 3.0      # 语音提前量（V1/V2）
  robot_precue_lead_s: 0.5    # 机器人提前量（R1）
  min_events_per_episode: 1

validation:
  fail_if_no_event: true
  fail_if_collision: true
  fail_if_route_not_progress: true
  min_distance_progress_m: 120
```

### 4.2 Episode 配置（统一 Schema）

每个 episode YAML 至少包含：

```yaml
episode_id: "P1_T2_lane_change"
suite_id: "P1"
type_id: "T2"
town:
  preferred: ["Town05"]
  fallback: ["Town03", "Town04"]

seed: 1102
duration_s: 60
weather:
  preset: "ClearNoon"

ego:
  blueprint: "vehicle.tesla.model3"
  control_mode: "tm"         # stage A 用 tm；stage B 改 autoware
  target_speed_kmh: 50

topology_query:
  start_selector: {...}
  route_selector: {...}
  decision_point_selector: {...}

npc_plan:
  traffic_manager:
    enabled: true
    vehicle_count: 25
    walker_count: 30
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: {...}
      behavior: {...}
    - id: "adjacent_blocker"
      kind: "vehicle"
      blueprint: "vehicle.tesla.model3"
      spawn_rule: {...}
      behavior: {...}

event_markers:
  - name: "decision_1"
    trigger: {...}
    expected_action: "lane_change_right"
    tags: ["what", "why", "lane_change", "gap_acceptance"]

termination:
  mode: "time_or_distance"
  max_time_s: 60
  min_progress_m: 180
  end_on_event_completed: false
```

---

## 5. `events.json` 事件规范（语音/机器人提示对齐）

### 5.1 Schema

```json
{
  "episode_id": "P1_T2_lane_change",
  "suite_id": "P1",
  "seed": 1102,
  "fps": 30,
  "events": [
    {
      "t": 12.35,
      "event_type": "lane_change_right",
      "action_text_id": "T2_WHAT_01",
      "reason_text_id": "T2_WHY_01",
      "voice_start_t": 9.35,
      "robot_precue_t": 8.85,
      "state_snapshot": {
        "ego_speed_mps": 12.1,
        "lead_speed_mps": 6.2,
        "headway_m": 18.4,
        "adjacent_gap_m": 22.0
      }
    }
  ]
}
```

### 5.2 事件触发优先级（建议）

* 若能直接检测“动作发生”：车道 ID 变化开始 → `lane_change_*`
* 若是让行：紧急车 TTC 低于阈值 → `yield_emergency`
* 若是 VRU：行人进入冲突区/距离阈值 → `brake_for_vru`
* 若是信号：红灯停车开始/绿灯起步 → `stop_for_red_light` / `go_on_green`
* 若是左转：开始转向并跨越 junction → `unprotected_left_turn_start`

---

## 6. Topology Query（自动选点规则，避免写死 spawn index）

> 这是让 Codex 真能落地的关键：**每个 episode 不写死坐标/索引，而是写规则**。

### 6.1 基础工具函数（建议在 `topology_query.py` 实现）

* `get_waypoint_graph(map)`：构建 waypoint 邻接图（next/prev + lane change）
* `find_straight_segments(min_lanes, min_length_m)`
* `find_merge_zones()`：识别车道数变化、汇入/汇出（lane_count transitions）
* `find_signalized_junctions()`：junction 内存在 traffic light 控制
* `find_left_turn_candidates(unprotected=True)`：junction 有左转可行且有对向直行 lane
* `find_crosswalk_like_zones()`：如果地图缺乏显式人行横道数据，采用“路口入口 + sidewalk 邻近”近似
* `pick_spawn_near(waypoint, spawn_points, max_dist_m=40)`
* `spawn_behind_ego_on_same_lane(distance_m)`：紧急车规则

### 6.2 每类场景的 selector 逻辑（规范化）

* **T1 Merge**：选一个 merge zone；start 在 merge 前 180–260m；decision_point 在 merge 起点前 60–90m
* **T2 Lane Change**：选 `min_lanes>=3` 的直路段；decision_point 在段内 50–80m
* **T3 Signalized Intersection**：选 traffic_light junction 的入口 lane；decision_point 在停止线前 60–90m
* **T4 Unprotected Left Turn**：选可左转且对向有通行 lane 的 junction；decision_point 在停止线前 40–70m
* **T5 Emergency**：选直路段（≥200m）+ 可靠边；decision_point 在紧急车进入后方视域或 TTC<4s
* **T6 VRU**：选路口入口/斑马线近似点；VRU 在 ego 距离 25–45m 时进入冲突区

---

# 7. 六套 Suites（P1–P6）完整详细设计（36 Episodes）

> 说明：下面每个 Episode 都按同一模板列出：**目标、地图、选点规则、NPC 脚本、触发与事件、验证准则、可调参数**。
> 你可以直接把每段的 YAML 复制到 `configs/episodes/`。
> 为兼容不同安装的地图资产，每个 episode 给了 `preferred` 与 `fallback`。

---

## 7.1 Suite P1（城市多车道基础版，难度中）

### P1_T1_merge（合流）

**目标**：主路车流中等，ego 在匝道汇入时进行“加速插入/等待让行”的可解释决策。
**地图**：preferred Town04；fallback Town05/Town03
**选点规则**

* `decision_point_selector`: merge zone 前 70m（±20）
* `start_selector`: decision_point 后退 220m（±40）
* `route_selector`: 通过合流点后继续 500m
  **NPC**
* `lead_slow`：合流后同车道慢车（速度 60%）
* `mainline_flow`：TM 车辆中密度，速度略高于 ego
  **事件**
* 触发：`distance_to_merge_point <= 80m` → `event_type=merge`
* 期望动作：短时减速观察 → 加速并入 或 等待间隙
  **验证**
* ego 在 20s 内完成合流（lane_id 变化或 road_id 转换）
* 合流处无碰撞
  **可调参数**
* `merge_trigger_m`、`mainline_vehicle_count`、`lead_slow_speed_kmh`

```yaml
episode_id: "P1_T1_merge"
suite_id: "P1"
type_id: "T1"
town: { preferred: ["Town04"], fallback: ["Town05","Town03"] }
seed: 1001
ego: { target_speed_kmh: 65, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 220 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 80 }
  route_selector: { type: "through_merge_then_forward", forward_m: 500 }
npc_plan:
  traffic_manager: { vehicle_count: 28, walker_count: 0 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane_after_merge", distance_m: 40 }
      behavior: { type: "constant_speed", speed_kmh: 35 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 80 }
    expected_action: "merge"
    tags: ["merge","gap_acceptance"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 180 }
```

---

### P1_T2_lane_change（变道/超车）

**目标**：前方慢车 → ego 决定变道超车（右变道或左变道按道路结构）。
**地图**：preferred Town05；fallback Town10/Town03
**选点**

* 3 车道直路段 ≥350m
* decision_point 在段内 60m
  **NPC**
* `lead_slow`：同车道慢车（速度 50–60%）
* `adjacent_blocker`：相邻车道并行车辆（形成间隙判断）
  **事件**：`headway_m < 25 && lead_speed << ego_target` 或 lane change start → `lane_change_right/left`
  **验证**：完成一次 lane_id 改变并稳定 3s

```yaml
episode_id: "P1_T2_lane_change"
suite_id: "P1"
type_id: "T2"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 1002
ego: { target_speed_kmh: 50, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 3, offset_back_m: 180 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 60 }
  route_selector: { type: "forward", forward_m: 420 }
npc_plan:
  traffic_manager: { vehicle_count: 25, walker_count: 10 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 30 }
      behavior: { type: "constant_speed", speed_kmh: 25 }
    - id: "adjacent_blocker"
      kind: "vehicle"
      blueprint: "vehicle.tesla.model3"
      spawn_rule: { type: "adjacent_lane_parallel", distance_m: 0 }
      behavior: { type: "constant_speed", speed_kmh: 45 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 25 }
    expected_action: "lane_change"
    tags: ["lane_change","overtake","gap_acceptance"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 200 }
```

---

### P1_T3_signal_intersection（信号灯路口）

**目标**：接近红灯→减速停车；绿灯→起步通过。
**地图**：preferred Town10；fallback Town05/Town03
**NPC**：前车跟停跟走；横向车流若可控则添加
**事件**：红灯停车开始 → `stop_for_red_light`

```yaml
episode_id: "P1_T3_signal_intersection"
suite_id: "P1"
type_id: "T3"
town: { preferred: ["Town10"], fallback: ["Town05","Town03"] }
seed: 1003
ego: { target_speed_kmh: 40, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 160 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 70 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 250 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 25 }
  scripted_actors:
    - id: "lead_vehicle"
      kind: "vehicle"
      blueprint: "vehicle.lincoln.mkz_2020"
      spawn_rule: { type: "ahead_same_lane", distance_m: 18 }
      behavior: { type: "tm_autopilot" }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 70 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","stop_go"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

---

### P1_T4_unprotected_left（无保护左转）

**目标**：对向来车 → 等待间隙 → 左转。
**地图**：preferred Town01；fallback Town03/Town10
**NPC**：对向 2–3 辆，间隔可控
**事件**：开始左转 → `unprotected_left_turn_start`

```yaml
episode_id: "P1_T4_unprotected_left"
suite_id: "P1"
type_id: "T4"
town: { preferred: ["Town01"], fallback: ["Town03","Town10"] }
seed: 1004
ego: { target_speed_kmh: 35, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 120 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 50 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 220 }
npc_plan:
  traffic_manager: { vehicle_count: 18, walker_count: 15 }
  scripted_actors:
    - id: "oncoming_stream"
      kind: "vehicle_stream"
      spawn_rule: { type: "oncoming_lane_stream", count: 3, gap_s: [3.0, 5.0] }
      behavior: { type: "constant_speed", speed_kmh: 40 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 50 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","oncoming_gap"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

---

### P1_T5_emergency_yield（紧急车让行）

**目标**：后方救护车快速接近 → ego 减速/靠右让行。
**地图**：preferred Town05；fallback Town04/Town10
**NPC**：紧急车从后方 60–80m 生成，速度优势 +20%
**事件**：TTC<4s → `yield_emergency`

```yaml
episode_id: "P1_T5_emergency_yield"
suite_id: "P1"
type_id: "T5"
town: { preferred: ["Town05"], fallback: ["Town04","Town10"] }
seed: 1005
ego: { target_speed_kmh: 45, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 260, offset_back_m: 120 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 4.0 }
  route_selector: { type: "forward", forward_m: 350 }
npc_plan:
  traffic_manager: { vehicle_count: 20, walker_count: 10 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 70 }
      behavior: { type: "pursue_lane", speed_kmh: 70 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 4.0 }
    expected_action: "yield"
    tags: ["emergency","yield"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 150 }
```

---

### P1_T6_vru_crossing（行人/骑行者）

**目标**：行人进入冲突区 → ego 刹停/减速让行。
**地图**：preferred Town10；fallback Town03/Town05
**NPC**：walker 在 ego 距离 30–40m 时横穿
**事件**：walker 距离 < 12m 或进入 lane polygon → `brake_for_vru`

```yaml
episode_id: "P1_T6_vru_crossing"
suite_id: "P1"
type_id: "T6"
town: { preferred: ["Town10"], fallback: ["Town03","Town05"] }
seed: 1006
ego: { target_speed_kmh: 35, control_mode: "tm" }
topology_query:
  start_selector: { type: "crossing_zone_approach", offset_back_m: 140 }
  decision_point_selector: { type: "distance_to_crossing_zone", trigger_distance_m: 35 }
  route_selector: { type: "forward", forward_m: 220 }
npc_plan:
  traffic_manager: { vehicle_count: 18, walker_count: 20 }
  scripted_actors:
    - id: "crossing_ped"
      kind: "walker"
      blueprint: "walker.pedestrian.0001"
      spawn_rule: { type: "sidewalk_near_crossing", offset_m: 2.0 }
      behavior: { type: "cross_road_at_time", start_after_s: 8.0, speed_mps: 1.3 }
event_markers:
  - name: "vru_conflict"
    trigger: { type: "distance_to_walker_below", meters: 12 }
    expected_action: "brake_or_yield"
    tags: ["vru","yield","safety"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

---

## 7.2 Suite P2（更密集城市交互版，why 更有用）

> P2 与 P1 同构，但把冲突主体增加（对向车更密、盲区车更贴近、行人更“突然”但仍安全可复现）

> **配置方式**：复制 P1 的 6 个 YAML，换 `seed` + 换 `town preferred` + 提高特定 `npc_plan` 参数。下面给出差异化版本（完整 6 个都列出可直接用）。

（为节省你复制成本，我仍给 6 个完整 YAML——你后面要改参数也方便。）

### P2_T1_merge

```yaml
episode_id: "P2_T1_merge"
suite_id: "P2"
type_id: "T1"
town: { preferred: ["Town12","Town05"], fallback: ["Town04","Town03"] }
seed: 2001
ego: { target_speed_kmh: 70, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 240 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 90 }
  route_selector: { type: "through_merge_then_forward", forward_m: 550 }
npc_plan:
  traffic_manager: { vehicle_count: 34, walker_count: 0 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane_after_merge", distance_m: 35 }
      behavior: { type: "constant_speed", speed_kmh: 30 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 90 }
    expected_action: "merge"
    tags: ["merge","gap_acceptance","dense_traffic"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 200 }
```

### P2_T2_lane_change

```yaml
episode_id: "P2_T2_lane_change"
suite_id: "P2"
type_id: "T2"
town: { preferred: ["Town10"], fallback: ["Town05","Town03"] }
seed: 2002
ego: { target_speed_kmh: 55, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 3, offset_back_m: 200 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 70 }
  route_selector: { type: "forward", forward_m: 480 }
npc_plan:
  traffic_manager: { vehicle_count: 30, walker_count: 20 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 26 }
      behavior: { type: "constant_speed", speed_kmh: 22 }
    - id: "adjacent_blocker"
      kind: "vehicle"
      blueprint: "vehicle.nissan.patrol"
      spawn_rule: { type: "adjacent_lane_rear", distance_m: 12 }
      behavior: { type: "approach_then_hold_gap", speed_kmh: 60, hold_gap_m: 10 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 22 }
    expected_action: "lane_change"
    tags: ["lane_change","dense","blindspot"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 220 }
```

### P2_T3_signal_intersection

```yaml
episode_id: "P2_T3_signal_intersection"
suite_id: "P2"
type_id: "T3"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 2003
ego: { target_speed_kmh: 42, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 180 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 80 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 280 }
npc_plan:
  traffic_manager: { vehicle_count: 28, walker_count: 30 }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 80 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","multi_actor"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 140 }
```

### P2_T4_unprotected_left

```yaml
episode_id: "P2_T4_unprotected_left"
suite_id: "P2"
type_id: "T4"
town: { preferred: ["Town03"], fallback: ["Town01","Town10"] }
seed: 2004
ego: { target_speed_kmh: 35, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 140 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 55 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 240 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 20 }
  scripted_actors:
    - id: "oncoming_stream"
      kind: "vehicle_stream"
      spawn_rule: { type: "oncoming_lane_stream", count: 4, gap_s: [2.8, 4.2] }
      behavior: { type: "constant_speed", speed_kmh: 45 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 55 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","harder_gap"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 140 }
```

### P2_T5_emergency_yield

```yaml
episode_id: "P2_T5_emergency_yield"
suite_id: "P2"
type_id: "T5"
town: { preferred: ["Town12","Town10"], fallback: ["Town05","Town04"] }
seed: 2005
ego: { target_speed_kmh: 48, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 300, offset_back_m: 140 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 3.5 }
  route_selector: { type: "forward", forward_m: 380 }
npc_plan:
  traffic_manager: { vehicle_count: 26, walker_count: 20 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 60 }
      behavior: { type: "pursue_lane", speed_kmh: 75 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 3.5 }
    expected_action: "yield"
    tags: ["emergency","urgent"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P2_T6_vru_crossing

```yaml
episode_id: "P2_T6_vru_crossing"
suite_id: "P2"
type_id: "T6"
town: { preferred: ["Town03"], fallback: ["Town10","Town05"] }
seed: 2006
ego: { target_speed_kmh: 38, control_mode: "tm" }
topology_query:
  start_selector: { type: "crossing_zone_approach", offset_back_m: 160 }
  decision_point_selector: { type: "distance_to_crossing_zone", trigger_distance_m: 40 }
  route_selector: { type: "forward", forward_m: 240 }
npc_plan:
  traffic_manager: { vehicle_count: 20, walker_count: 35 }
  scripted_actors:
    - id: "crossing_ped"
      kind: "walker"
      blueprint: "walker.pedestrian.0004"
      spawn_rule: { type: "occluded_ped_popout", occluder: "parked_vehicle", offset_m: 1.5 }
      behavior: { type: "cross_road_at_time", start_after_s: 7.0, speed_mps: 1.5 }
event_markers:
  - name: "vru_conflict"
    trigger: { type: "distance_to_walker_below", meters: 11 }
    expected_action: "brake_or_yield"
    tags: ["vru","popout"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 130 }
```

---

## 7.3 Suite P3（高速/快速路强化版：速度更高、提前播报价值更大）

> 主要差异：target_speed 更高、合流/变道更“战略性”、VRU 替换为“施工/路侧人员”以保证可实现。

（同样给 6 个完整 YAML）

### P3_T1_merge

```yaml
episode_id: "P3_T1_merge"
suite_id: "P3"
type_id: "T1"
town: { preferred: ["Town04"], fallback: ["Town12","Town05"] }
seed: 3001
ego: { target_speed_kmh: 80, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 260 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 95 }
  route_selector: { type: "through_merge_then_forward", forward_m: 650 }
npc_plan:
  traffic_manager: { vehicle_count: 30, walker_count: 0 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.carlamotors.carlacola"
      spawn_rule: { type: "ahead_same_lane_after_merge", distance_m: 45 }
      behavior: { type: "constant_speed", speed_kmh: 45 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 95 }
    expected_action: "merge"
    tags: ["merge","high_speed"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 260 }
```

### P3_T2_lane_change

```yaml
episode_id: "P3_T2_lane_change"
suite_id: "P3"
type_id: "T2"
town: { preferred: ["Town04"], fallback: ["Town12","Town05"] }
seed: 3002
ego: { target_speed_kmh: 85, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 3, offset_back_m: 240 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 80 }
  route_selector: { type: "forward", forward_m: 700 }
npc_plan:
  traffic_manager: { vehicle_count: 26, walker_count: 0 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 40 }
      behavior: { type: "constant_speed", speed_kmh: 55 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 35 }
    expected_action: "lane_change"
    tags: ["lane_change","high_speed"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 280 }
```

### P3_T3_signal_intersection（若高速无信号路口，fallback 到城市入口）

```yaml
episode_id: "P3_T3_signal_intersection"
suite_id: "P3"
type_id: "T3"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 3003
ego: { target_speed_kmh: 45, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 180 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 85 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 300 }
npc_plan:
  traffic_manager: { vehicle_count: 24, walker_count: 20 }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 85 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","anticipation"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 150 }
```

### P3_T4_unprotected_left

```yaml
episode_id: "P3_T4_unprotected_left"
suite_id: "P3"
type_id: "T4"
town: { preferred: ["Town03"], fallback: ["Town01","Town10"] }
seed: 3004
ego: { target_speed_kmh: 38, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 150 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 60 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 260 }
npc_plan:
  traffic_manager: { vehicle_count: 20, walker_count: 15 }
  scripted_actors:
    - id: "oncoming_stream"
      kind: "vehicle_stream"
      spawn_rule: { type: "oncoming_lane_stream", count: 3, gap_s: [3.0, 4.5] }
      behavior: { type: "constant_speed", speed_kmh: 50 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 60 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","risk_assessment"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P3_T5_emergency_yield

```yaml
episode_id: "P3_T5_emergency_yield"
suite_id: "P3"
type_id: "T5"
town: { preferred: ["Town04"], fallback: ["Town12","Town05"] }
seed: 3005
ego: { target_speed_kmh: 75, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 380, offset_back_m: 180 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 3.8 }
  route_selector: { type: "forward", forward_m: 650 }
npc_plan:
  traffic_manager: { vehicle_count: 24, walker_count: 0 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 80 }
      behavior: { type: "pursue_lane", speed_kmh: 110 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 3.8 }
    expected_action: "yield"
    tags: ["emergency","high_speed"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 240 }
```

### P3_T6_vru_roadwork（路侧工人/锥桶）

```yaml
episode_id: "P3_T6_vru_roadwork"
suite_id: "P3"
type_id: "T6"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 3006
ego: { target_speed_kmh: 55, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 260, offset_back_m: 160 }
  decision_point_selector: { type: "distance_to_roadwork", trigger_distance_m: 60 }
  route_selector: { type: "forward", forward_m: 420 }
npc_plan:
  traffic_manager: { vehicle_count: 20, walker_count: 0 }
  scripted_actors:
    - id: "roadworker"
      kind: "walker"
      blueprint: "walker.pedestrian.0010"
      spawn_rule: { type: "roadside_worker_zone", lateral_m: 2.0 }
      behavior: { type: "idle_or_slow_walk", speed_mps: 0.4 }
    - id: "cones"
      kind: "prop_group"
      spawn_rule: { type: "place_cones_along_lane", count: 8, spacing_m: 3.5 }
event_markers:
  - name: "roadwork_caution"
    trigger: { type: "distance_to_roadwork", meters: 60 }
    expected_action: "slow_down_or_shift"
    tags: ["vru","roadwork","safety_margin"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 180 }
```

---

## 7.4 Suite P4（路口密集版：更偏城市冲突）

> 主要差异：路口更多、VRU 更多、信号灯与左转更“拥挤但可控”。

> 配置同构：你可从 P2 复制改 `seed` 与 `town`，并微调 NPC 数量。这里给完整 6 个 YAML（保持可直接落地）。

（略去解释文字，避免过长——但每个 YAML 都含可调点，足够 Codex 实现与后续人工调整。）

### P4_T1_merge

```yaml
episode_id: "P4_T1_merge"
suite_id: "P4"
type_id: "T1"
town: { preferred: ["Town05"], fallback: ["Town04","Town03"] }
seed: 4001
ego: { target_speed_kmh: 60, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 210 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 85 }
  route_selector: { type: "through_merge_then_forward", forward_m: 480 }
npc_plan:
  traffic_manager: { vehicle_count: 32, walker_count: 15 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 85 }
    expected_action: "merge"
    tags: ["merge","urban_dense"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 180 }
```

### P4_T2_lane_change

```yaml
episode_id: "P4_T2_lane_change"
suite_id: "P4"
type_id: "T2"
town: { preferred: ["Town10"], fallback: ["Town05","Town03"] }
seed: 4002
ego: { target_speed_kmh: 48, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 3, offset_back_m: 190 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 65 }
  route_selector: { type: "forward", forward_m: 440 }
npc_plan:
  traffic_manager: { vehicle_count: 30, walker_count: 30 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 24 }
      behavior: { type: "constant_speed", speed_kmh: 20 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 20 }
    expected_action: "lane_change"
    tags: ["lane_change","pre_intersection"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 200 }
```

### P4_T3_signal_intersection

```yaml
episode_id: "P4_T3_signal_intersection"
suite_id: "P4"
type_id: "T3"
town: { preferred: ["Town10"], fallback: ["Town05","Town03"] }
seed: 4003
ego: { target_speed_kmh: 38, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 200 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 90 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 260 }
npc_plan:
  traffic_manager: { vehicle_count: 34, walker_count: 40 }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 90 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","dense_peds"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

### P4_T4_unprotected_left

```yaml
episode_id: "P4_T4_unprotected_left"
suite_id: "P4"
type_id: "T4"
town: { preferred: ["Town01"], fallback: ["Town03","Town10"] }
seed: 4004
ego: { target_speed_kmh: 32, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 150 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 55 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 220 }
npc_plan:
  traffic_manager: { vehicle_count: 26, walker_count: 30 }
  scripted_actors:
    - id: "oncoming_stream"
      kind: "vehicle_stream"
      spawn_rule: { type: "oncoming_lane_stream", count: 4, gap_s: [3.2, 5.0] }
      behavior: { type: "constant_speed", speed_kmh: 42 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 55 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","urban"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

### P4_T5_emergency_yield

```yaml
episode_id: "P4_T5_emergency_yield"
suite_id: "P4"
type_id: "T5"
town: { preferred: ["Town10"], fallback: ["Town05","Town04"] }
seed: 4005
ego: { target_speed_kmh: 42, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 260, offset_back_m: 120 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 4.2 }
  route_selector: { type: "forward", forward_m: 320 }
npc_plan:
  traffic_manager: { vehicle_count: 28, walker_count: 35 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 65 }
      behavior: { type: "pursue_lane", speed_kmh: 68 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 4.2 }
    expected_action: "yield"
    tags: ["emergency","city"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

### P4_T6_vru_crossing

```yaml
episode_id: "P4_T6_vru_crossing"
suite_id: "P4"
type_id: "T6"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 4006
ego: { target_speed_kmh: 30, control_mode: "tm" }
topology_query:
  start_selector: { type: "crossing_zone_approach", offset_back_m: 150 }
  decision_point_selector: { type: "distance_to_crossing_zone", trigger_distance_m: 38 }
  route_selector: { type: "forward", forward_m: 200 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 45 }
  scripted_actors:
    - id: "crossing_ped"
      kind: "walker"
      blueprint: "walker.pedestrian.0016"
      spawn_rule: { type: "sidewalk_near_crossing", offset_m: 1.0 }
      behavior: { type: "cross_road_at_time", start_after_s: 6.5, speed_mps: 1.4 }
event_markers:
  - name: "vru_conflict"
    trigger: { type: "distance_to_walker_below", meters: 10 }
    expected_action: "brake_or_yield"
    tags: ["vru","dense"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 100 }
```

---

## 7.5 Suite P5（对比版：更“平稳”，用于校准解释收益）

> P5 让冲突更少、间隙更大、动作更“可预测”，用来检验：解释是否仍然提升信任/降低负荷，或是否在低压力情境下效应减弱。

### P5_T1_merge

```yaml
episode_id: "P5_T1_merge"
suite_id: "P5"
type_id: "T1"
town: { preferred: ["Town03"], fallback: ["Town04","Town05"] }
seed: 5001
ego: { target_speed_kmh: 55, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 240 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 75 }
  route_selector: { type: "through_merge_then_forward", forward_m: 450 }
npc_plan:
  traffic_manager: { vehicle_count: 18, walker_count: 10 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 75 }
    expected_action: "merge"
    tags: ["merge","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P5_T2_lane_change

```yaml
episode_id: "P5_T2_lane_change"
suite_id: "P5"
type_id: "T2"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 5002
ego: { target_speed_kmh: 45, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 3, offset_back_m: 200 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 55 }
  route_selector: { type: "forward", forward_m: 380 }
npc_plan:
  traffic_manager: { vehicle_count: 18, walker_count: 10 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 35 }
      behavior: { type: "constant_speed", speed_kmh: 28 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 28 }
    expected_action: "lane_change"
    tags: ["lane_change","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P5_T3_signal_intersection

```yaml
episode_id: "P5_T3_signal_intersection"
suite_id: "P5"
type_id: "T3"
town: { preferred: ["Town10"], fallback: ["Town05","Town03"] }
seed: 5003
ego: { target_speed_kmh: 35, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 170 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 75 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 220 }
npc_plan:
  traffic_manager: { vehicle_count: 16, walker_count: 15 }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 75 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 110 }
```

### P5_T4_unprotected_left

```yaml
episode_id: "P5_T4_unprotected_left"
suite_id: "P5"
type_id: "T4"
town: { preferred: ["Town01"], fallback: ["Town03","Town10"] }
seed: 5004
ego: { target_speed_kmh: 30, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 140 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 50 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 200 }
npc_plan:
  traffic_manager: { vehicle_count: 14, walker_count: 10 }
  scripted_actors:
    - id: "oncoming_stream"
      kind: "vehicle_stream"
      spawn_rule: { type: "oncoming_lane_stream", count: 2, gap_s: [5.0, 7.0] }
      behavior: { type: "constant_speed", speed_kmh: 38 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 50 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 100 }
```

### P5_T5_emergency_yield

```yaml
episode_id: "P5_T5_emergency_yield"
suite_id: "P5"
type_id: "T5"
town: { preferred: ["Town05"], fallback: ["Town10","Town04"] }
seed: 5005
ego: { target_speed_kmh: 40, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 240, offset_back_m: 120 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 4.8 }
  route_selector: { type: "forward", forward_m: 280 }
npc_plan:
  traffic_manager: { vehicle_count: 16, walker_count: 10 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 90 }
      behavior: { type: "pursue_lane", speed_kmh: 60 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 4.8 }
    expected_action: "yield"
    tags: ["emergency","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 110 }
```

### P5_T6_vru_crossing

```yaml
episode_id: "P5_T6_vru_crossing"
suite_id: "P5"
type_id: "T6"
town: { preferred: ["Town03"], fallback: ["Town10","Town05"] }
seed: 5006
ego: { target_speed_kmh: 28, control_mode: "tm" }
topology_query:
  start_selector: { type: "crossing_zone_approach", offset_back_m: 150 }
  decision_point_selector: { type: "distance_to_crossing_zone", trigger_distance_m: 45 }
  route_selector: { type: "forward", forward_m: 200 }
npc_plan:
  traffic_manager: { vehicle_count: 12, walker_count: 20 }
  scripted_actors:
    - id: "crossing_ped"
      kind: "walker"
      blueprint: "walker.pedestrian.0002"
      spawn_rule: { type: "sidewalk_near_crossing", offset_m: 1.8 }
      behavior: { type: "cross_road_at_time", start_after_s: 9.0, speed_mps: 1.2 }
event_markers:
  - name: "vru_conflict"
    trigger: { type: "distance_to_walker_below", meters: 12 }
    expected_action: "brake_or_yield"
    tags: ["vru","low_stress"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 90 }
```

---

## 7.6 Suite P6（冗余替换包：用于某 episode 不稳定时“热替换”）

> P6 的定位是“备份套件”：当 P1–P5 某个 episode 在你的地图/版本/控制器上不稳定（例如无法稳定触发信号灯、左转失败、行人路径异常），P6 可以用来替换该 episode，保证实验进度。

### P6_T1_merge

```yaml
episode_id: "P6_T1_merge"
suite_id: "P6"
type_id: "T1"
town: { preferred: ["Town05"], fallback: ["Town04","Town03"] }
seed: 6001
ego: { target_speed_kmh: 62, control_mode: "tm" }
topology_query:
  start_selector: { type: "merge_start_offset", offset_back_m: 230 }
  decision_point_selector: { type: "merge_point", trigger_distance_m: 85 }
  route_selector: { type: "through_merge_then_forward", forward_m: 520 }
npc_plan:
  traffic_manager: { vehicle_count: 24, walker_count: 10 }
event_markers:
  - name: "merge_decision"
    trigger: { type: "distance_to_merge_point", meters: 85 }
    expected_action: "merge"
    tags: ["merge","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P6_T2_lane_change

```yaml
episode_id: "P6_T2_lane_change"
suite_id: "P6"
type_id: "T2"
town: { preferred: ["Town03"], fallback: ["Town05","Town10"] }
seed: 6002
ego: { target_speed_kmh: 46, control_mode: "tm" }
topology_query:
  start_selector: { type: "multi_lane_straight", min_lanes: 2, offset_back_m: 200 }
  decision_point_selector: { type: "straight_segment_point", offset_forward_m: 60 }
  route_selector: { type: "forward", forward_m: 360 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 15 }
  scripted_actors:
    - id: "lead_slow"
      kind: "vehicle"
      blueprint: "vehicle.audi.a2"
      spawn_rule: { type: "ahead_same_lane", distance_m: 28 }
      behavior: { type: "constant_speed", speed_kmh: 24 }
event_markers:
  - name: "lane_change_decision"
    trigger: { type: "headway_below", meters: 24 }
    expected_action: "lane_change"
    tags: ["lane_change","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 140 }
```

### P6_T3_signal_intersection

```yaml
episode_id: "P6_T3_signal_intersection"
suite_id: "P6"
type_id: "T3"
town: { preferred: ["Town05"], fallback: ["Town10","Town03"] }
seed: 6003
ego: { target_speed_kmh: 38, control_mode: "tm" }
topology_query:
  start_selector: { type: "signalized_junction_approach", offset_back_m: 180 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 80 }
  route_selector: { type: "pass_junction_then_forward", forward_m: 240 }
npc_plan:
  traffic_manager: { vehicle_count: 20, walker_count: 20 }
event_markers:
  - name: "approach_signal"
    trigger: { type: "distance_to_stop_line", meters: 80 }
    expected_action: "prepare_stop_or_go"
    tags: ["signal","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 110 }
```

### P6_T4_unprotected_left

```yaml
episode_id: "P6_T4_unprotected_left"
suite_id: "P6"
type_id: "T4"
town: { preferred: ["Town10"], fallback: ["Town03","Town01"] }
seed: 6004
ego: { target_speed_kmh: 34, control_mode: "tm" }
topology_query:
  start_selector: { type: "left_turn_candidate", offset_back_m: 150 }
  decision_point_selector: { type: "stop_line_distance", trigger_distance_m: 55 }
  route_selector: { type: "execute_left_turn_then_forward", forward_m: 240 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 25 }
event_markers:
  - name: "left_turn_decision"
    trigger: { type: "distance_to_stop_line", meters: 55 }
    expected_action: "wait_then_left_turn"
    tags: ["left_turn","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 120 }
```

### P6_T5_emergency_yield

```yaml
episode_id: "P6_T5_emergency_yield"
suite_id: "P6"
type_id: "T5"
town: { preferred: ["Town04"], fallback: ["Town05","Town10"] }
seed: 6005
ego: { target_speed_kmh: 52, control_mode: "tm" }
topology_query:
  start_selector: { type: "straight_segment", min_length_m: 300, offset_back_m: 150 }
  decision_point_selector: { type: "emergency_ttc", ttc_s: 4.0 }
  route_selector: { type: "forward", forward_m: 420 }
npc_plan:
  traffic_manager: { vehicle_count: 22, walker_count: 10 }
  scripted_actors:
    - id: "ambulance"
      kind: "vehicle"
      blueprint: "vehicle.ford.ambulance"
      spawn_rule: { type: "behind_same_lane", distance_m: 75 }
      behavior: { type: "pursue_lane", speed_kmh: 78 }
event_markers:
  - name: "emergency_approach"
    trigger: { type: "ttc_to_emergency_below", seconds: 4.0 }
    expected_action: "yield"
    tags: ["emergency","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 160 }
```

### P6_T6_vru_crossing

```yaml
episode_id: "P6_T6_vru_crossing"
suite_id: "P6"
type_id: "T6"
town: { preferred: ["Town10"], fallback: ["Town03","Town05"] }
seed: 6006
ego: { target_speed_kmh: 32, control_mode: "tm" }
topology_query:
  start_selector: { type: "crossing_zone_approach", offset_back_m: 150 }
  decision_point_selector: { type: "distance_to_crossing_zone", trigger_distance_m: 35 }
  route_selector: { type: "forward", forward_m: 220 }
npc_plan:
  traffic_manager: { vehicle_count: 18, walker_count: 25 }
  scripted_actors:
    - id: "crossing_ped"
      kind: "walker"
      blueprint: "walker.pedestrian.0012"
      spawn_rule: { type: "sidewalk_near_crossing", offset_m: 1.2 }
      behavior: { type: "cross_road_at_time", start_after_s: 7.5, speed_mps: 1.35 }
event_markers:
  - name: "vru_conflict"
    trigger: { type: "distance_to_walker_below", meters: 11 }
    expected_action: "brake_or_yield"
    tags: ["vru","backup"]
termination: { mode: "time_or_distance", max_time_s: 60, min_progress_m: 110 }
```

---

# 8. 质量控制与验收标准（每个 Episode 必须通过）

## 8.1 必须通过（Hard requirements）

* 录制成功：生成 `master_video.mp4` 或 `frames/`
* 事件存在：`events.json` 中至少 1 个 event
* 进度足够：ego 前进距离 ≥ `min_progress_m`
* 无碰撞：若 `fail_if_collision=true`，任意碰撞则 episode 判失败并重跑（同 seed or new seed 取决于策略）

## 8.2 建议通过（Soft requirements）

* 决策行为发生：例如 T2 必须发生一次 lane_id 变化；T3 必须出现停/走之一
* 决策点可解释：state_snapshot 有足够字段（headway、speed、gap、TTC 等）
* 时间码稳定：event 时间对齐可用于语音提前播报（voice_start_t >= 0）

## 8.3 自动重跑策略（建议）

* 同 seed 重跑最多 2 次（排除瞬态 spawn 冲突）
* 仍失败则：

  * 换 fallback town
  * 或换 seed（+100）
  * 或调用 P6 对应类型替换该 episode（保持同构）

---

# 9. 直接给 Codex 的实现提示词（可复制）

## 9.1 项目级提示词（一次性喂给 Codex）

```text
请实现一个 CARLA 0.9.16 “实验刺激生成器”项目，完全配置驱动。输入为 configs/globals.yaml 与 configs/episodes/*.yaml。

要求：
1) Windows 运行 CARLA server；Ubuntu 客户端连接 host/port。支持同步模式 fixed_delta_seconds。
2) 支持 36 个 episode（P1..P6 × T1..T6），每个 episode 运行后输出：master_video.mp4、events.json、run_metadata.json。
3) 禁止硬编码 spawn point index。必须实现 topology_query：根据 episode.topology_query 规则在 runtime 搜索可用 waypoint/路段（merge zone、多车道直线、信号灯路口、无保护左转路口、VRU crossing zone、直路段用于紧急车追越）。
4) NPC 由 TrafficManager 生成背景车流；额外 scripted_actors 按规则生成（lead_slow、adjacent_blocker、oncoming_stream、ambulance、crossing_ped、cones/roadworker 等）。
5) 实现 event_extractor：根据 trigger 条件与车辆状态提取 events.json（包含 voice_start_t=t-voice_lead_time、robot_precue_t=voice_start_t-0.5，并记录 state_snapshot）。
6) 提供批量入口：python -m runner --suite P1（跑 6 个 episode） 或 --all（跑 36 个）。
7) 提供 counterbalance.py：生成 32 被试的 6 Block 条件顺序 + Suite 分配（平衡拉丁方近似），输出 assignments.csv。
```

## 9.2 模块级提示词（分任务让 Codex 更稳）

```text
任务A：实现 config_loader（YAML merge + 校验 + 默认值应用）。
任务B：实现 topology_query（见 docs/scenario_design.md 第6节）并写单元测试：每个 selector 至少返回一个候选点，否则给出诊断信息。
任务C：实现 episode_builder：加载地图→spawn ego/sensors→spawn TM traffic→spawn scripted actors→run ticks→record frames。
任务D：实现 event_extractor：按 event_markers 与 trigger 条件生成 events.json。
任务E：实现 recorder：frames->mp4（ffmpeg），以及 run_metadata.json 版本快照。
```

---

## 你接下来怎么用这份文档（最短路径）

1. 先让 Codex 按 **Stage A（ego=TM）** 实现全链路，跑通 P1 一套。
2. 再跑全库 36 个 episode，看看哪些 episode 不稳定：用 P6 替换，或调参数。
3. 最后再把 ego 切到 Autoware（Stage B），保持 episode 配置不变，验证 event 是否仍能稳定触发。

下面是你要补充进文档的内容（**明确标注为“补充信息/新增”**），你可以直接复制粘贴到原文档中。我建议放在你文档的 **“语音与 CARLA 无原生声音的解决方案”**之后、或 **“输出物与数据管理”**之前（因为它会改变你对“要录制多少视频”的理解）。

---

## 补充信息（新增）：用“同一条母带视频 + 可选信息通道”实现 2×3 条件，无需重复录制视频

> **新增说明**：为降低仿真录制成本并提升条件控制一致性，本研究可采用“**单一母带视频（Master Video）+ 可选信息通道（语音与机器人提示）**”的刺激呈现策略。即：**同一段驾驶视频在 2×3 条件下保持完全一致**，仅在实验实施时按条件选择性播放（或不播放）语音提示，并控制机器人是否执行提示动作，从而实现 6 种实验条件。

### 1) 核心思路

对每个 Episode（例如 P1_T2_lane_change），仿真阶段只生成并保存一次：

* `master_video.mp4`：**仅含视觉**（不内嵌语音、不叠加提示 overlay）
* `events.json`：该 Episode 的事件时间线（例如决策点、触发点、建议播报时刻）
* `audio_assets/`：与该 Episode 对应的语音素材（离线生成）

  * `what.wav`（V1 用）
  * `whatwhy.wav`（V2 用；或拆成 `what.wav + why.wav` 便于拼接）
* （可选）`robot_timeline.json`：机器人提示动作时间线（本质可由 `events.json` 推导）

实验实施时，按照被试当次 Block 的条件（V×R），对同一个 `master_video.mp4` **选择性叠加信息通道**：

* 语音：播放/不播放、播放 What 或 What+Why
* 机器人：静止或执行“琥珀灯光+扭动”提示（在语音前 0.5s）

这样就能在**不重复渲染/录制视频**的情况下实现 6 条件对比。

---

### 2) 2×3 条件到“播放策略”的映射表

| 条件标识 | 语音策略                            | 机器人策略                                                           |
| ---- | ------------------------------- | --------------------------------------------------------------- |
| V0R0 | 不播放语音                           | 机器人静止                                                           |
| V0R1 | 不播放语音                           | **仍不建议**执行机器人提示（否则会引入“无语音但有提示”的额外信号）；若实验确实要保留 R1，则需在方案中明确其含义与假设 |
| V1R0 | 播放 `what.wav`                   | 机器人静止                                                           |
| V1R1 | 播放 `what.wav`                   | 语音前 0.5s：琥珀灯 + 扭动                                               |
| V2R0 | 播放 `whatwhy.wav`（或 what+why 拼接） | 机器人静止                                                           |
| V2R1 | 播放 `whatwhy.wav`（或 what+why 拼接） | 语音前 0.5s：琥珀灯 + 扭动                                               |

> 重要：你原始设计里 V0 是“无播报”，R1 是“语音播报前 0.5s 提示”。因此 **V0R1 在语义上会变得不自然**（因为没有“语音播报前”）。若你坚持 2×3 完全正交，可以保留 V0R1，但必须在方法中解释：R1 在 V0 下表示“在关键事件发生前仍给注意力提示，但不提供语音内容”。这会改变变量含义，需要你在论文里明确说明并讨论潜在影响。

---

### 3) 对“需要录制多少套视频”的影响

采用该策略后，**视频录制数量与条件数量解耦**：

* 你仍需要准备 **6 套 Suite × 每套 6 个 Episode = 36 段 master 视频** 来避免被试内重复观看同一片段（学习效应）。
* 但你**不需要**为 6 个条件分别生成 6 份视频版本。

  * 以前（若条件内嵌到视频）可能是 36 × 6 个最终视频
  * 现在：仍是 **36 个 master 视频** + 对应的语音素材与时间线文件

这能显著降低 CARLA 渲染与录制工作量，并提升刺激一致性（同一视觉输入下比较解释策略）。

---

### 4) 实施与同步要求（确保“信息通道”对齐）

为保证语音/机器人提示在播放阶段精确对齐，应对每个 Episode 输出：

* `events.json` 中至少包含：

  * `event_type`（例如 lane_change / merge / stop_for_red_light 等）
  * `t_event`（事件发生或决策点时间）
  * `t_voice_start = t_event - voice_lead_time_s`（例如提前 3s）
  * `t_robot_precue = t_voice_start - 0.5s`（R1 使用）
* 音频素材命名统一（例如 `what.wav`, `whatwhy.wav`），并记录在 `events.json` 的 `audio_id` 字段

实验播放端（SILAB 或你自研播放器）只需读取：

* 当前 Block 的 condition_id
* 该 Block 对应 Suite 的 6 段 Episode（master 视频 + events.json + 音频素材）

即可在播放时按条件进行：

* V0：mute
* V1：播放 what
* V2：播放 whatwhy
* R1：在 `t_robot_precue` 触发机器人提示

---

### 5) 方法学表述建议（避免审稿人质疑）

建议在方法中明确写出以下两点：

1. **视觉刺激恒定**：同一 Episode 的视觉输入在不同条件下保持一致，减少混杂变量，使差异更可归因于解释信息与提示方式。
2. **信息通道可控**：语音与机器人提示由统一时间线文件控制并在实验前完成校准，不在实验中实时生成，确保一致性与可复现。

---

### 6) 工程实现提示（给 Codex）

* 仿真生成阶段输出：

  * `master_video.mp4`
  * `events.json`（含 t_event/t_voice_start/t_robot_precue/audio_id）
* 实验播放阶段提供一个 `playback_config.json`：

  * `condition_id`（VxRy）
  * `episode_list`（6 段视频路径）
  * `audio_policy`（none/what/whatwhy）
  * `robot_policy`（none/amber_wiggle）
* 后期合成（可选）：若你想提前生成 6 个条件版 mp4（方便 SILAB 直接播放），则由脚本批处理：

  * 读取 `master_video.mp4` + `events.json` + `what/whatwhy.wav`
  * 输出 `conditioned_video_V1R1.mp4` 等
    但注意：如果你要做“真实机器人提示”，则 **视频里不应该出现提示 overlay**，以免与真实机器人重复。

---
