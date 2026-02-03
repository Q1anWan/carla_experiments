# CARLA Experiment Client - Architecture and Interfaces

This document describes the program logic, module responsibilities, and public interfaces.
It is intended to help AI agents modify the project safely.

## Version History

| 版本 | 分支 | 特性 | 状态 |
|------|------|------|------|
| **V1** | `main` | 基于帧号的硬编码事件触发，Teleport 轨迹跟踪 | 稳定 |
| **V2** | `feature/v2-*` | 基于 scene_edit.json 的关键帧编辑器，线性插值轨迹 | 稳定 |
| **V3** | `feature/v3-director` | 条件触发的事件 DSL，Director 编排器，Traffic Manager + Override 混合驱动 | **当前开发** |

### V1 → V2 主要变更
- 新增 `interactive_editor.py` 可视化关键帧编辑器
- 新增 `scene_edit.json` 场景描述格式
- 轨迹生成: 硬编码 → 关键帧插值

### V2 → V3 主要变更
- 事件触发: 帧号硬编码 → 条件 DSL (`trigger.all`, `metric`, `op`, `value`)
- 驱动后端: 纯 TM → TM + ShortOverrideBackend 混合
- 新增 `director/` 模块: event_dsl, metrics, director, gate_checks
- 新增 `driver_backends/` 模块: tm_backend, short_override
- 红绿灯: 全绿覆盖 → 组协调 + 周期配置
- 遥测: 新增 `traffic_lights` 字段

---

## 1) Runtime data flow (Plan -> Validate -> Render)

1. `map_exporter` generates map assets (centerlines, junctions, spawn candidates).
2. `planner_compiler` compiles episode configs into `plan.json` + `events_plan.json`.
3. `validator` checks event presence, lane-change recognition, and map feasibility.
4. `renderer` replays trajectories in CARLA, records video and telemetry.
5. Outputs are written under `outputs/<episode_id>/` (plan, events, validation, video, telemetry).

## 2) Runtime data flow (V3 Director 模式)

V3 使用条件触发的事件编排系统，替代基于帧号的硬编码触发。

1. CLI 加载 V3 场景 YAML（含 `execution_mode: v3` 和 `events` DSL）。
2. CARLA 连接建立，配置同步模式 + 固定时间步长。
3. 场景构建:
   - 生成 ego、事件 Actor（cut_in_a/b/c 等）、背景交通。
   - 配置红绿灯时序（保持组协调）。
   - 初始化 `ScenarioDirector`，解析事件 DSL。
4. Prewarm 阶段 tick 世界。
5. 录制主循环（每帧）:
   - `director.tick()`: 计算指标 → 评估触发条件 → 执行事件动作。
   - `telemetry.tick()`: 记录 ego/actors/traffic_lights 状态。
   - 相机录制帧。
6. 输出: video, telemetry.json/csv, director_events.json, gate_report.json。
7. 清理 Actor，恢复世界设置。

### Director 事件状态机

```
pending ──(trigger)──▶ triggered ──(TM action)──▶ post ──▶ done
                              │
                              └──(fallback)──▶ override_active ──▶ post
```

## 2.1) Runtime data flow (legacy run_scenario)

1. CLI loads a scenario YAML and optional render preset.
2. CARLA connection is created and world settings are applied (sync + fixed delta).
3. Scenario is built: actors are spawned and TM parameters applied.
4. A short prewarm phase ticks the world before recording to avoid spawn drop.
5. Recording starts:
   - A camera sensor is attached to ego.
   - Each tick saves a frame and feeds event extraction.
6. Outputs are written (events, metadata, video).
7. Actors are cleaned up and world settings restored.

## 3) Entry points and CLI interfaces

- `cli.py` / `carla_experiment_client/cli.py`
  - Unified CLI (menu + subcommands).
  - Subcommands: `map`, `plan`, `validate`, `render`, `pipeline`, `preview`, `editor`, `suite`, `run-v3`, `compare`, `convert`.
  - `run-v3`: V3 Director 模式运行场景（条件触发事件）。
  - Supports headless editor export with `--headless`.

- `runner.py`
  - Plan/validate/render pipeline runner (non-interactive).
  - Args: `plan`, `validate`, `render`, `all`.

- `run_scenario.py`
  - Wrapper for `carla_experiment_client/run_scenario.py`.
  - Key args: `--scenario`, `--out`, `--render-preset`, `--render-presets`, `--host`, `--port`, `--tm-port`, `--timeout`.
  - Output: `runs/<timestamp>_<scenario>` with `scenario.yaml`, `run.log`, `events.json`, `run_metadata.json`, `master_video.mp4`.

- `render_variants.py`
  - Wrapper for `carla_experiment_client/render_variants.py`.
  - Args: `--run_dir`, `--audio-only`.
  - Outputs: 6 stimulus variants or audio-only assets + robot timeline.

- `debug_tools.py`
  - Subcommands: `check_connection`, `tick_sync`, `camera`, `scenario`, `events`, `audio`, `variants`.
  - Use for step-by-step validation without full runs.

## 4) Configuration files

- `configs/client.yaml`
  - Connection defaults: `host`, `port`, `tm_port`, `timeout`, `allow_version_mismatch`.

- `configs/globals.yaml`
  - Plan/validate/render defaults (dt, recording, weather, validation thresholds).

- `configs/episodes/*.yaml`
  - Episode-level planning inputs for the planner compiler.

- `configs/suites/*.yaml`
  - Suite collections of episode IDs for batch runs.

- `configs/scenarios/*.yaml`
  - Scenario runtime config.
  - Top-level fields: `id`, `map`, `weather`, `seed`, `duration`, `fps`, `fixed_delta_seconds`, `sync_mode`, `no_rendering_mode`, `ego_vehicle`, `camera`, `events`, `scenario`.
  - `camera`: `width`, `height`, `fov`, `fps`, `preset`, `transform`.
  - `events`: `voice_lead_time_s`, `robot_precue_lead_s`, `min_event_time_s`.
  - `scenario`: per-scenario params (trigger frames, speeds, counts, etc.).
  - Recording notes:
    - All actors spawn before recording.
    - `prewarm_seconds` ticks the world before frame 0 to avoid initial drop.

- `configs/render_presets.yaml`
  - Preset overrides for fast or final rendering.
  - Fields: `fps`, `duration`, `fixed_delta_seconds`, `camera`, `scenario_overrides`, `scale_frames`, `scale_min_event_time`.
  - `scale_frames` rescales all numeric `*_frame` / `*_frames` in `scenario`.

## 5) Core modules and interfaces

### `carla_experiment_client/planning/*`
- `map_exporter.py`: export map graph + geojson assets.
- `trajectory_schema.py`: plan/event schema + load/save helpers.
- `planner_compiler.py`: episode YAML -> time-parameterized trajectories.
- `validator.py`: feasibility + event checks prior to rendering.

### `carla_experiment_client/render/*`
- `replay_controller.py`: trajectory follower (teleport, MVP).
- `renderer.py`: replay + record video/telemetry/events.

### `carla_experiment_client/editor/*`
- `interactive_editor.py`: keyframe-based scene editor + export.
- `mvp.py`: 2D preview renderer for `plan.json`.

### `carla_experiment_client/director/*` (V3)
- `event_dsl.py`: 事件 DSL 解析器（YAML → 类型化 dataclass）。
  - `EventSpec`, `TriggerSpec`, `TriggerCondition`, `TMAction`, `OverrideFallback`。
  - `parse_events(raw_list)`: 解析 YAML 事件列表。
- `metrics.py`: Actor 间指标计算。
  - `ActorMetrics`: gap_m, ttc_s, lateral_offset_m, speeds。
  - `compute_metrics(ego, target, map_obj, frame, fps)`: 实时指标。
- `director.py`: 核心编排器 `ScenarioDirector`。
  - `tick(frame_index)`: 评估触发条件，执行事件动作。
  - `get_event_log()`: 导出事件日志。
- `gate_checks.py`: 质量门禁检查（TTC/gap 阈值）。

### `carla_experiment_client/driver_backends/*` (V3)
- `base.py`: `DriverBackend` 抽象基类。
  - `run_step(vehicle, frame, dt) -> VehicleControl | None`
  - `activate(vehicle)`, `deactivate(vehicle)`, `is_done`
- `tm_backend.py`: Traffic Manager 驱动后端。
  - `configure(vehicle, TMAction)`: 应用 TM 参数。
  - `force_lane_change(vehicle, direction)`: 强制换道。
- `short_override.py`: 短时物理接管控制器。
  - PID 速度控制 + quintic 横向轨迹。
  - 运动学约束: a_long, a_lat, jerk, steer_rate 限幅。

### `carla_experiment_client/control/*` (V3)
- `pid.py`: `PIDController` 通用 PID 控制器。
  - Anti-windup, 输出限幅, 微分滤波。

### `carla_experiment_client/carla_client.py`
- Responsibility: connect to CARLA, load map, apply sync settings, configure Traffic Manager.
- Key functions:
  - `connect_client(host, port, timeout, allow_version_mismatch)`
  - `load_world(client, map_name)`
  - `configure_world(world, sync_mode, fixed_delta_seconds, no_rendering_mode)`
  - `configure_traffic_manager(client, tm_port, sync_mode, seed)`
  - `setup_carla(...) -> CarlaWorldContext`
  - `restore_world(ctx)`

### `carla_experiment_client/config.py`
- Responsibility: dataclasses and config loading.
- Key functions:
  - `load_client_config(path)`
  - `load_scenario_config(path)`
  - `load_render_presets(path)`
  - `apply_render_preset(config, preset)`

### `carla_experiment_client/run_scenario.py`
- Responsibility: orchestrate a single run.
- Key functions:
  - `run_scenario(...) -> int`
  - `_write_metadata(out_dir, config, host, port, tm_port, server_version, client_version, allow_version_mismatch, render_preset)`
  - `_attach_file_logger(out_dir)`
- Implementation notes:
  - `prewarm_seconds` or `prewarm_frames` is applied before recording.
  - Logs are written to `run.log` in the run folder.

### `carla_experiment_client/sensors/camera_recorder.py`
- Responsibility: synchronous RGB camera recording.
- Key types/functions:
  - `CameraRecorder.start() / stop()`
  - `CameraRecorder.record_frames(frames_dir, num_frames, timeout, on_tick)`
  - `record_video(scenario_ctx, out_dir, on_tick)`

### `carla_experiment_client/events/extractor.py`
- Responsibility: generate decision events per tick.
- Key type: `EventExtractor`.
  - `tick(snapshot, frame_index)` updates speed, control, lane state, and emits events.
  - `finalize()` returns events list.
- Events include: `lane_change_left/right`, `brake_hard`, `slow_down`, `stop_for_red_light`, `yield_to_emergency`, `avoid_pedestrian`.

### `carla_experiment_client/telemetry/*`
- `recorder.py`: 遥测记录器 `TelemetryRecorder`。
  - `tick(snapshot, frame)`: 记录 ego、actors、traffic_lights 状态。
  - `save(output_dir)`: 输出 telemetry.json + telemetry.csv。
  - 红绿灯记录: id, state, pole_index, group_ids, timing (elapsed_time)。
- `sae_j670.py`: SAE J670 坐标系变换器。
  - `SAEJ670Transformer.compute_state(vehicle, dt) -> VehicleState`

### `carla_experiment_client/scenarios/*`
- Responsibility: spawn actors and define per-frame behavior.
- `BaseScenario` helper methods:
  - `_spawn_vehicle`, `_spawn_walker`, `_spawn_background_traffic`, `_apply_ego_tm`.
  - `find_spawn_point(...)` selects spawn points based on lane/junction constraints.
- Each scenario implements `build(world, tm, rng) -> ScenarioContext`.
- Scenario registry: `scenarios/registry.py` maps scenario id to class.
- V3 场景: 使用 `_configure_traffic_lights()` 配置红绿灯时序，保持组协调。

### `carla_experiment_client/weather.py`
- Responsibility: apply weather presets by name.

### `carla_experiment_client/video.py`
- Responsibility: encode frames to MP4 and mux audio (ffmpeg).

### `carla_experiment_client/audio/*`
- `tts.py`: text-to-speech (placeholder or edge-tts).
- `mux.py`: audio timeline rendering and mp4 mux.

### `carla_experiment_client/render_variants.py`
- Responsibility: generate 6 stimulus variants using events and master video.

### `carla_experiment_client/utils.py`
- Helpers: file I/O, command execution, json writing, clamp.

## 6) Outputs and artifacts

Plan/validate/render outputs:
- `outputs/<episode_id>/plan.json`
- `outputs/<episode_id>/events_plan.json`
- `outputs/<episode_id>/scene_edit.json` (editor output)
- `outputs/<episode_id>/validation_report.json`
- `outputs/<episode_id>/master_video.mp4`
- `outputs/<episode_id>/telemetry.json`
- `outputs/<episode_id>/telemetry.csv`
- `outputs/<episode_id>/events.json`
- `outputs/<episode_id>/run_metadata.json`

A run directory includes:
- `scenario.yaml`: scenario config snapshot.
- `run.log`: runtime logs.
- `events.json`: decision timeline (see `docs/events_schema.json`).
- `run_metadata.json`: run metadata (map, seed, fps, render_preset, versions).
- `frames/`: raw PNG frames (optional).
- `master_video.mp4`: visual-only master.

V3 run directory 额外包含:
- `director_events.json`: Director 事件状态日志。
- `gate_report.json`: 质量门禁检查结果。
- `telemetry.json`: 含 `traffic_lights` 字段记录红绿灯状态。

Variant rendering adds:
- `variants/voice{v}_robot{r}/stimulus.mp4` (if audio enabled)
- `audio_assets/` and `robot_timeline.csv` (audio-only mode)

## 7) Extension guidelines

- Add a new scenario:
  1) Create a new scenario class in `carla_experiment_client/scenarios/`.
  2) Register it in `scenarios/registry.py`.
  3) Add a YAML config under `configs/scenarios/`.
  4) Add a description doc in `docs/scenarios/` and reference it via `doc:`.

- Add a new event type:
  1) Update `events/extractor.py` mapping and detection logic.
  2) Update `docs/events_schema.json`.

- Add a render preset:
  1) Add a new preset under `configs/render_presets.yaml`.
  2) Ensure `scale_frames` is set if trigger frames must scale with duration/fps.
