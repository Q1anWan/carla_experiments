## 改进方向

我们在多轮迭代后发现：当前“直接在 CARLA 地图上选点 + 即时生成路线并录制”的管线，对部分地图的拓扑/路口/车道信息处理不稳定，导致场景复现与触发事件不可靠。为提高稳定性与可控性，我们需要**重构整个场景生成管线**，将“场景设计（规划）”与“视频渲染（执行）”解耦。

### 总体目标

建立一条两阶段管线：

1. **二维轨迹规划阶段（Plan）**

   * 在 2D 地图上生成并编辑所有对象（Ego、NPC、VRU、紧急车等）的**时间参数化轨迹**（time-parameterized trajectories）。
   * 输出的轨迹同时表达**地图约束信息**（车道/路口/合流拓扑）与**运动信息**（位置、速度、朝向、加速度、车道变更时刻等）。
   * 这一阶段必须可视化、可编辑、可验证：在渲染前就能发现路线不可行、事件触发不明确、冲突过强/过弱等问题。

2. **渲染执行阶段（Render）**

   * 渲染阶段只负责“按轨迹回放”，不再做复杂的地图推理与决策。
   * 所有运动对象基于运动学/轨迹跟踪控制器（例如纯追踪 Pure Pursuit / Stanley / MPC 简化版）跟随规划轨迹，并在 CARLA 中生成视频与事件数据。
   * 渲染阶段应具备**强确定性**：同一套轨迹 + seed → 生成一致的视频与 `events.json`。

---

## 新的场景设计工作流（端到端）

### Step 0：地图预处理（Map Build）

* 从 CARLA OpenDRIVE / Map API 提取道路拓扑，生成 **2D Map Graph**（道路中心线、车道边界、路口连接、允许变道关系）。
* 输出：

  * `map_graph.json`（可序列化图结构）
  * `lane_centerlines.geojson`（给编辑器渲染）
  * `junction_areas.geojson`（路口区域）
  * `spawn_candidates.json`（可用 spawn/waypoint 候选集合）

> 验收：每张 Town 地图都能导出上述文件；编辑器能加载并显示。

### Step 1：轨迹规划（Trajectory Plan）

* 对每个 Episode（例如 P1_T2）生成轨迹草案：

  * Ego 轨迹：车道保持、变道、左转、停车等动作的时间点明确。
  * NPC/VRU 轨迹：以“事件触发”为中心设计（例如行人在 t=8s 横穿，救护车在 t=10s 从后方逼近）。
* 轨迹表示建议统一为：

  * `Trajectory(t) -> (x, y, yaw, v, a)`
  * 可选：`lane_id`、`s`（沿中心线弧长参数）、`behavior_tag`
* 输出：

  * `plan.json`（包含所有 actor 的轨迹、关键事件、参数）
  * `events_plan.json`（规划层事件：t_event、t_voice_start、t_robot_precue、expected_action）

> 验收：仅通过 2D 预览就能判断：是否有变道、是否能在路口停车、VRU 是否进入冲突区、紧急车是否会追上等。

### Step 2：规划验证与自动修复（Validate & Fix）

在进入渲染前必须自动跑一轮“规划级验证”：

* **地图可行性**：轨迹是否都落在可行驶区域（或 sidewalk for VRU），是否穿越不可达连接。
* **动力学合理性**：曲率/加速度/jerk 是否超阈值；变道横向速度是否合理。
* **冲突强度**：TTC、最小间距、是否必撞（除非你故意设计极限场景）。
* **事件可辨识**：每个 Episode 至少 1 个核心事件；并能从轨迹本身推导出 action（如 lane_id change）。

输出：

* `validation_report.json`
* 若可修复：给出建议参数（例如把行人 start_after_s +1.0、把 blocker 速度 -5km/h 等）

> 验收：任何不通过的 Episode 不能进入渲染队列（避免浪费 GPU 录制）。

### Step 3：渲染回放（Render Replay）

* 在 CARLA 中：

  * 生成 actor
  * 将轨迹作为“参考轨迹”
  * 用轨迹跟踪控制器按 tick 执行
* 同步录制：

  * `master_video.mp4`
  * `events.json`（从执行状态与轨迹事件对齐导出）
  * `run_metadata.json`

> 验收：轨迹回放误差（位置 RMS、lane deviation）低于阈值；事件时间偏差 < 200ms。

---

## 必要的“场景编辑工具”（必须做出来的能力）

### 工具 A：2D 场景编辑器（核心）

功能要求：

1. **地图底图**：显示车道中心线、车道边界、路口区域、变道连接、限速信息（若可得）。
2. **轨迹编辑**：

   * 画/拖拽 Ego 路线（按 lane centerline 吸附）
   * 插入动作段：变道、减速停车、左转、让行
   * 支持时间轴：调整每段速度曲线（v(t)）与关键时刻（t_event）
3. **多 actor 管理**：

   * 添加 NPC/VRU/紧急车（类型、尺寸、速度策略）
   * 每个 actor 有独立轨迹与时间偏移
4. **冲突可视化**：

   * 显示 TTC/最小距离曲线
   * 高亮“冲突窗口”（你需要触发解释的窗口）
5. **一键导出**：

   * `plan.json`、`events_plan.json`、`episode.yaml`（或直接覆盖 configs）

形式建议：

* 快速落地：Python + Qt / 或 Web（React + Canvas）
* 先做 MVP：只要能加载地图、画轨迹、导出 JSON 就行。

### 工具 B：轨迹编译器（Planner Compiler）

* 将“编辑器输出的高层路径”（例如一串 lane_id + action 节点）编译为细粒度轨迹点序列（固定 dt，例如 0.05s）
* 生成可跟踪的参考轨迹（含 yaw、曲率、速度曲线）

### 工具 C：批量验证器（CI 风格）

* 对 36 个 Episode 全部跑 Validate，输出汇总报告
* 自动标记不稳定的 episode，并建议替换到 P6 或调整参数

---

## 给 AI 的任务拆分（建议直接让它按此实现）

1. 实现 `map_exporter`：CARLA map → `map_graph.json + geojson`
2. 实现 `trajectory_schema`：定义 plan.json 的数据结构与版本管理
3. 实现 `planner_compiler`：高层节点 → 轨迹点序列（含速度曲线）
4. 实现 `validator`：可行性 + 动力学 + 冲突 + 事件可辨识
5. 实现 `replay_controller`：轨迹跟踪（pure pursuit/stanley）+ actor 控制接口
6. 实现 `renderer`：录制视频、导出 events/run metadata
7. 实现 `2d_editor_mvp`：加载地图、编辑 Ego 轨迹、添加 VRU、导出 plan

---

我们需要重构 CARLA 场景生成管线：现有“直接在 CARLA 地图上选点→即时录制”对部分地图拓扑不稳定。请按下面要求实现“Plan→Validate→Render”两阶段架构（CARLA 0.9.16，Win 3080 跑 server，Ubuntu 无 GPU 跑 client）。

目标：把“场景设计/轨迹规划”与“渲染录制/回放”解耦。渲染阶段只做轨迹回放与录制，不再做复杂地图推理，确保确定性与可复现。

必须实现的模块与产物：
1) Map Exporter（map_exporter.py）
- 从 CARLA Map/OpenDRIVE/Waypoints 导出 2D 拓扑：车道中心线、车道边界（可选）、junction 区域、lane connectivity（next/prev/left/right lane change）、spawn 候选点。
- 输出：map_graph.json、lane_centerlines.geojson、junction_areas.geojson、spawn_candidates.json。
2) 2D 轨迹规划数据结构（trajectory_schema.py）
- 定义 plan.json schema（版本号、episode_id、town、seed、dt、actors[]；每个 actor 的 trajectory 点序列：t,x,y,yaw,v,a,(optional)lane_id,s,tag；以及 events_plan[]：t_event,t_voice_start,t_robot_precue,event_type,expected_action,audio_id,state_targets）。
3) Planner Compiler（planner_compiler.py）
- 将“高层路径/动作节点”（如 lane sequence + action segments: keep_lane/lane_change/stop/turn_left/yield）编译成可跟踪的细粒度轨迹（固定 dt=0.05s），生成 yaw/曲率/速度曲线（v(t)）与关键事件时刻。
4) Validator（validator.py）
- 在渲染前对 plan.json 做自动验证并输出 validation_report.json：
  a) 地图可行性：轨迹点是否落在可行驶区域/sidewalk（VRU）附近（用 waypoint 投影误差阈值）。
  b) 动力学合理性：横向/纵向加速度、曲率、jerk 阈值。
  c) 冲突强度：最小间距、TTC；避免必撞（除非标记为极限场景）。
  d) 事件可辨识：每个 episode 至少 1 个核心事件；能从轨迹推导动作发生（如 lane_id change）。
- 不通过则给出可执行修复建议（如行人 start_after_s +1.0、blocker 速度 -5km/h）。
5) Replay Renderer（replay_controller.py + renderer.py）
- 在 CARLA 同步模式 fixed_delta_seconds=0.05 下回放 plan.json：
  - spawn ego/NPC/VRU，按各自 trajectory 用轨迹跟踪控制器（pure pursuit/Stanley）执行；或对非物理精度要求的 actor 使用 set_transform（但 ego 建议用控制器以更真实）。
  - 录制第一人称 RGB（1280x720@30fps 或可配置），输出 master_video.mp4（无音轨母带）+ run_metadata.json。
  - 将 events_plan 对齐执行时间，生成 events.json（含 t_event/voice_start_t=t_event-voice_lead_time, robot_precue_t=voice_start_t-0.5；并记录 state_snapshot）。
6) 2D 场景编辑器 MVP（可先命令行+可视化，或 Web/Qt）
- 能加载 lane_centerlines/junction_areas，显示 2D 地图；
- 支持编辑/生成 ego 高层路径（吸附中心线）、插入动作段（变道/停车/左转/让行）、设置速度曲线；
- 支持添加 NPC/VRU/紧急车的轨迹（至少：行人横穿、救护车追越、前车慢车、相邻车道 blocker）；
- 导出 plan.json + events_plan.json（供 validator/render 使用）。
7) 批量入口（runner.py）
- python -m runner plan --episode <id> 生成/编译 plan
- python -m runner validate --episode <id>
- python -m runner render --episode <id>
- python -m runner all --suite P1（批量跑 6 个）/ --all（跑 36 个）
- 失败自动重试策略：同 seed 重跑2次→换 fallback town→换 seed(+100)→可选替换到备份套件。

额外约束：
- 禁止硬编码 spawn point index；必须基于 map_graph 的 selector/规则选点。
- 所有关键参数配置化：globals.yaml + episodes.yaml（dt、duration、speed、traffic、阈值、voice_lead_time等）。
- 输出目录结构固定：outputs/<episode_id>/{master_video.mp4,plan.json,events.json,validation_report.json,run_metadata.json}。
- 写清晰日志与诊断信息（为什么某地图找不到 merge/信号灯路口/左转点等）。
请先实现最小闭环：map_exporter + trajectory_schema + planner_compiler(只支持 keep_lane + lane_change) + validator(基本可行性/事件存在) + renderer(回放+录制+events导出)，然后逐步扩展到6类场景与编辑器功能。
