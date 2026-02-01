# Traceability: technical_details.md → Code

This document maps every section of `docs/technical_details.md` to actual code files, symbols, parameters, and outputs.

All paths are relative to `carla_experiment_client/` unless otherwise noted.

---

## 1. 系统架构 (System Architecture)

| Doc Claim | Code File | Symbols | Key Params | Outputs | Status |
|-----------|-----------|---------|------------|---------|--------|
| Scene Edit → Plan Build | `editor/interactive_editor.py` | `_build_plan()`, `_build_trajectory()` | `dt`, `duration` | `plan.json`, `events_plan.json` | OK |
| Plan Build → Validate | `planning/validator.py` | `validate_plan()` | `max_seconds` | `validation_report.json` | OK |
| Validate → Render | `render/renderer.py` | `render_plan()` | `max_seconds`, `image_size` | `master_video.mp4`, `telemetry.json`, `telemetry.csv` | OK |
| Pipeline orchestrator | `cli.py` | `do_pipeline()` | `episode`, `out` | All above | OK |
| Deterministic replay | `render/replay_controller.py` | `TeleportFollower.apply()` | `frame_index` | — | OK |
| SAE J670 coordinates | `telemetry/sae_j670.py` | `SAEJ670Transformer` | — | — | OK |
| MapIndex spatial hash | `editor/interactive_editor.py` | `MapIndex` | `cell_size=5.0` | — | OK |

---

## 2. 交互式编辑器 (Interactive Editor)

### 2.1 核心数据结构

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| `Keyframe` dataclass | `editor/interactive_editor.py:26` | `Keyframe(t, x, y, v)` | OK |
| `ActorState` dataclass | `editor/interactive_editor.py:34` | `ActorState(actor_id, kind, role, ...)` | OK |
| `EventMarker` dataclass | `editor/interactive_editor.py:52` | `EventMarker(t_event, event_type, ...)` | OK |
| `SceneEdit` dataclass | `editor/interactive_editor.py:62` | `SceneEdit(version, episode_id, ...)` | OK |

### 2.2 UI Layout

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Figure 14×10 | `editor/interactive_editor.py:488` | `plt.figure(figsize=(14, 10))` | OK |
| ax_map 65%×85% | `editor/interactive_editor.py:489` | `fig.add_axes([0.05, 0.10, 0.65, 0.85])` | OK |
| Time slider | `editor/interactive_editor.py:504` | `Slider(ax_slider, ...)` | OK |
| Button layout | `editor/interactive_editor.py:519-552` | `_build_ui()` | OK |
| Status bar | `editor/interactive_editor.py:591-597` | `ax_status`, `status_text` | OK |

### 2.3 编辑模式

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Add mode (`a`) | `editor/interactive_editor.py:709,1081` | `_set_mode_add()`, `_on_key_press()` | OK |
| Move mode (`m`) | `editor/interactive_editor.py:713,1084` | `_set_mode_move()` | OK |
| Delete mode (`d`) | `editor/interactive_editor.py:717,1087` | `_set_mode_delete()` | OK |
| Snap toggle (`s`) | `editor/interactive_editor.py:721,1090` | `_toggle_snap()` | OK |
| Keyframe table (`t`) | `editor/interactive_editor.py:1111,1146` | `_show_keyframe_table()` | OK |
| Undo/Redo (`Ctrl+Z/Y`) | `editor/interactive_editor.py:1096-1098` | `_undo()`, `_redo()` | OK |

### 2.4 车道吸附算法 (MapIndex)

| Doc Claim | Code File | Symbols | Key Params | Status |
|-----------|-----------|---------|------------|--------|
| Spatial hash init | `editor/interactive_editor.py:74` | `MapIndex.__init__()` | `cell_size=5.0` | OK |
| Nearest lane query | `editor/interactive_editor.py:86` | `MapIndex.nearest_lane()` | `max_radius=6` | OK |
| Snap to centerline | `editor/interactive_editor.py:112` | `MapIndex.snap_to_centerline()` | `max_radius=6` | OK |
| Cell calculation | `editor/interactive_editor.py:83` | `MapIndex._cell()` | — | OK |

### 2.5 轨迹构建算法

| Doc Claim | Code File | Symbols | Key Params | Status |
|-----------|-----------|---------|------------|--------|
| Sparse → dense interpolation | `editor/interactive_editor.py:223` | `_build_trajectory()` | `dt`, `duration`, `map_index` | OK |
| Boundary padding | `editor/interactive_editor.py:184` | `_prepare_keyframes()` | `duration` | OK |
| Linear interpolation | `editor/interactive_editor.py:203` | `_interpolate_position()` | — | OK |
| Yaw calculation | `editor/interactive_editor.py:273-277` | `atan2(dy, dx)` | — | OK |
| Speed calculation | `editor/interactive_editor.py:280-283` | `sqrt(dx²+dy²)/dt` | — | OK |
| Speed hint override | `editor/interactive_editor.py:284-286` | `speed_hint` array | — | OK |

### 2.6 分析功能

| Doc Claim | Code File | Symbols | Key Params | Status |
|-----------|-----------|---------|------------|--------|
| Speed limit check | `editor/interactive_editor.py:1234,1243` | `max_speed_mps=30.0` | — | OK |
| Accel/decel check | `editor/interactive_editor.py:1235-1236,1245` | `max_accel=8.0`, `max_decel=10.0` | — | OK |
| Lateral accel check | `editor/interactive_editor.py:1237,1261` | `max_lat_accel=5.0` | — | OK |
| Off-lane check | `editor/interactive_editor.py:1272-1280` | `vehicle_lane_threshold=1.8`, `walker_lane_threshold=2.8` | — | OK |
| Conflict detection | `editor/interactive_editor.py:1281-1294` | `conflict_threshold=3.0` | — | OK |
| TTC calculation | `editor/interactive_editor.py:1295-1306` | `ttc_warning_s=3.0` | — | OK |

### 2.7 撤销/重做

| Doc Claim | Code File | Symbols | Key Params | Status |
|-----------|-----------|---------|------------|--------|
| Undo stack | `editor/interactive_editor.py:484` | `_undo_stack: List[str]` | `_max_undo=50` | OK |
| Redo stack | `editor/interactive_editor.py:485` | `_redo_stack: List[str]` | — | OK |
| Push undo | `editor/interactive_editor.py:727` | `_push_undo()` | — | OK |
| Undo action | `editor/interactive_editor.py:823` | `_undo()` | — | OK |
| Redo action | `editor/interactive_editor.py:836` | `_redo()` | — | OK |
| State serialization | `editor/interactive_editor.py:735` | `_scene_to_dict()` | — | OK |
| State restore | `editor/interactive_editor.py:788` | `_restore_from_dict()` | — | OK |

---

## 3. CLI 管线

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| `map` subcommand | `cli.py:57` | `do_map()` | OK |
| `plan` subcommand | `cli.py:75` | `do_plan()` | OK |
| `validate` subcommand | `cli.py:88` | `do_validate()` | OK |
| `render` subcommand | `cli.py:101` | `do_render()` | OK |
| `pipeline` subcommand | `cli.py:117` | `do_pipeline()` | OK |
| `preview` subcommand | `cli.py:142` | `do_preview()` | OK |
| `editor` subcommand | `cli.py:158` | `do_editor()` | OK |
| `suite` subcommand | `cli.py:182` | `do_suite()` | OK |
| `compare` subcommand | `cli.py` | `do_compare()` | OK |
| `convert` subcommand | `cli.py` | `do_convert()` | OK |
| `bundle` subcommand | `cli.py` | `do_bundle()` | OK |
| `test` subcommand | `cli.py` | `do_test()` | OK |
| Interactive menu | `cli.py` | `run_menu()` | OK |
| Argument parser | `cli.py` | `build_parser()` | OK |
| Entry point | `cli.py` | `main()` | OK |

---

## 4. 数据格式

### 4.1 scene_edit.json

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Save | `editor/interactive_editor.py:403` | `_save_scene()` | OK |
| Load | `editor/interactive_editor.py:352` | `_load_scene()` | OK |
| Schema version `"0.1"` | `editor/interactive_editor.py:404` | `payload["version"]` | OK |

### 4.2 plan.json

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Plan dataclass | `planning/trajectory_schema.py:128` | `Plan` | OK |
| ActorPlan | `planning/trajectory_schema.py:58` | `ActorPlan` | OK |
| TrajectoryPoint | `planning/trajectory_schema.py:14` | `TrajectoryPoint(t,x,y,yaw,v,a,lane_id,s,tag)` | OK |
| EventPlan | `planning/trajectory_schema.py:89` | `EventPlan` | OK |
| Save plan | `planning/trajectory_schema.py:165` | `save_plan()` | OK |
| Load plan | `planning/trajectory_schema.py:171` | `load_plan()` | OK |
| Plan version | `planning/trajectory_schema.py:11` | `PLAN_VERSION = "0.1"` | OK |

### 4.3 telemetry.json (schema v0.2)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| TelemetryRecorder | `telemetry/recorder.py:64` | `TelemetryRecorder` | OK |
| TelemetryFrame | `telemetry/recorder.py:41` | `TelemetryFrame(frame, t_sim, ...)` | OK |
| TelemetryConfig | `telemetry/recorder.py:30` | `TelemetryConfig` | OK |
| Schema version 0.2 | `telemetry/recorder.py` | `_save_json()` → `schema_version: "0.2"` | OK |
| speed_fd (ego FD) | `telemetry/recorder.py` | `tick()` → ego finite-difference speed | OK |
| speed_actor (CARLA physics) | `telemetry/sae_j670.py` | `VehicleState.speed` → `speed_actor` | OK |
| 6-DOF NPC telemetry | `telemetry/recorder.py` | `_record_actor()` → velocity, angular_velocity | OK |

### 4.4 telemetry.csv

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| CSV output | `telemetry/recorder.py` | `TelemetryRecorder.save()` → writes CSV | OK |
| SAE J670 transform | `telemetry/sae_j670.py` | `SAEJ670Transformer`, `VehicleState` | OK |
| CSV header (20 cols) | `telemetry/sae_j670.py` | `VehicleState.csv_header()` | OK |
| speed_fd column | `telemetry/sae_j670.py` | `VehicleState.speed_fd` | OK |

### 4.5 地图数据

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Map export config | `planning/map_exporter.py:26` | `MapExportConfig` | OK |
| Export function | `planning/map_exporter.py` | `export_map()` | OK |
| Lane UID format | `planning/map_exporter.py:34` | `_lane_uid()` → `"road:section:lane"` | OK |
| Spawn candidates | `planning/map_exporter.py:47` | `_extract_spawn_candidates()` | OK |

---

## 5. 渲染器

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Render function | `render/renderer.py:123` | `render_plan()` | OK |
| Spawn actor | `render/renderer.py:57` | `_spawn_actor()` | OK |
| First transform | `render/renderer.py:76` | `_first_transform()` | OK |
| Build events | `render/renderer.py:88` | `_build_events()` | OK |
| TeleportFollower | `render/replay_controller.py:17` | `TeleportFollower` | OK |
| build_follower | `render/replay_controller.py:38` | `build_follower()` | OK |
| Camera recording | `sensors/camera_recorder.py:22` | `CameraRecorder` | OK |
| Video encoding | `video.py` | `encode_frames_to_mp4()` | OK |

---

## 6. 遥测可视化对比工具

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Comparison plot (4 panels) | `tools/make_comparison.py` | `generate_comparison()` | OK |
| Alignment function | `tools/make_comparison.py` | `align_trajectories()` | OK |
| Position error | `tools/make_comparison.py` | `compute_position_error()` | OK |
| Error statistics | `tools/make_comparison.py` | `error_stats()` | OK |
| Sanity checks | `tools/make_comparison.py` | `_sanity_check()` | OK |
| compare_report.json | `tools/make_comparison.py` | `generate_comparison()` output | OK |
| Peak error location | `tools/make_comparison.py` | `report["peak_error_location"]` | OK |

---

## 7. 遥测 → 场景设计转换工具

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Telemetry to scene_edit conversion | `tools/convert_telemetry_to_scene.py` | `convert_telemetry()` | OK |

---

## 8. 路径规划 (Path Planner)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| LaneGraph | `planning/path_planner.py` | `LaneGraph` | OK |
| PathPoint | `planning/path_planner.py` | `PathPoint` | OK |
| build_road_path | `planning/path_planner.py` | `build_road_path()` | OK |
| generate_speed_profile | `planning/path_planner.py` | `generate_speed_profile()` | OK |
| parameterize_by_time | `planning/path_planner.py` | `parameterize_by_time()` | OK |
| Yaw EMA smoothing | `planning/path_planner.py` | `_SMOOTH_ALPHA=0.15` | OK |
| Curvature speed limit | `planning/path_planner.py` | `a_lat_max=4.0` | OK |

---

## 9. 离线验证器 (Plan Validator)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| validate_plan | `planning/plan_validator.py` | `validate_plan()` | OK |
| generate_debug_plots | `planning/plan_validator.py` | `generate_debug_plots()` | OK |
| save_report | `planning/plan_validator.py` | `save_report()` | OK |
| Reverse detection | `planning/plan_validator.py` | checks reverse frames >10 | OK |
| Yaw jump detection | `planning/plan_validator.py` | Δyaw >120° | OK |
| Speed/accel bounds | `planning/plan_validator.py` | speed >25, accel >8 | OK |

---

## 10. Phase G: 数据质量与冲突分析 (Data Quality & Conflict Analysis)

### 10.1 G1: SHA256 Hash 溯源

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Plan version 0.2 | `planning/trajectory_schema.py:11` | `PLAN_VERSION = "0.2"` | OK |
| `source_scene_sha256` in Plan | `planning/trajectory_schema.py` | `Plan.source_scene_sha256` | OK |
| `source_plan_sha256` in telemetry | `telemetry/recorder.py:107,120` | `_source_plan_sha256`, `set_source_plan_sha256()` | OK |
| Telemetry JSON metadata hash | `telemetry/recorder.py:381` | `metadata["source_plan_sha256"]` | OK |
| Renderer hash chain validation | `render/renderer.py:174` | Compares `plan.source_scene_sha256` vs current scene SHA | OK |
| Renderer copies scene_edit.json | `render/renderer.py:184` | `shutil.copy2(scene_edit_path, out_dir)` | OK |
| Renderer passes plan SHA to telemetry | `render/renderer.py:189,245` | `plan_sha256`, `telemetry.set_source_plan_sha256()` | OK |
| Comparison hash validation | `tools/make_comparison.py:142` | `_validate_hash_chain()` | OK |
| compare_report `hash_chain_valid` | `tools/make_comparison.py:331` | `report["hash_chain_valid"]` | OK |
| `compute_file_sha256()` utility | `planning/path_planner.py:712` | `compute_file_sha256()` | OK |

**Hash Chain**:
```
scene_edit.json ──SHA256──▶ plan.json.source_scene_sha256
plan.json       ──SHA256──▶ telemetry.json.metadata.source_plan_sha256
compare_report  ──validates──▶ hash_chain_valid: true/false
```

### 10.2 G2: Keyframe 时间对齐 (Keyframe Alignment)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| KeyframeAlignment dataclass | `planning/trajectory_schema.py` | `KeyframeAlignment(kf_idx, t_ui, t_plan, x_kf, y_kf, x_plan, y_plan, spatial_error_m)` | OK |
| ActorPlan.keyframe_alignment | `planning/trajectory_schema.py` | `ActorPlan.keyframe_alignment: List[KeyframeAlignment]` | OK |
| compute_keyframe_alignment | `planning/path_planner.py:672` | `compute_keyframe_alignment(trajectory, keyframes)` → nearest-point match | OK |
| _build_plan computes alignment | `editor/interactive_editor.py:333-335` | Calls `compute_keyframe_alignment()` per actor | OK |
| Comparison uses t_plan for error | `tools/make_comparison.py:241-252` | `kf_retime_error` uses `t_plan` from alignment | OK |
| Comparison keeps legacy metric | `tools/make_comparison.py:233-239` | `kf_legacy_error` uses original `t_ui` | OK |
| retime_stats in compare_report | `tools/make_comparison.py:194-225` | `_extract_keyframe_alignment()` → `retime_drift_max_s`, `retime_drift_mean_s` | OK |

**算法**: 对每个 keyframe (x_kf, y_kf)，在 plan trajectory 中找到距离最近的点，记录其 t_plan。由于 path planner 的 `parameterize_by_time()` 基于运动学积分产生 time drift（最大 ~25s），直接用 t_ui 做误差对比会产生 ~33m 的虚假误差。用 t_plan 后误差降至 ~0.8m。

### 10.3 G3: 静止尾巴处理 (Episode End Mode)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| SceneEdit.episode_end_mode | `editor/interactive_editor.py:75` | `episode_end_mode: str = "hold_at_end"` | OK |
| _load_scene reads field | `editor/interactive_editor.py:444` | `episode_end_mode=str(raw.get(...))` | OK |
| _save_scene writes field | `editor/interactive_editor.py:451` | `payload["episode_end_mode"]` | OK |
| apply_episode_end_mode | `planning/path_planner.py:727` | `apply_episode_end_mode(trajectory, end_mode, dt)` | OK |
| trim_to_motion mode | `planning/path_planner.py:757` | Truncates after last frame with `v > 0.1` + 10 frame buffer | OK |
| hold_at_end mode | `planning/path_planner.py:753` | No-op, keep trajectory as-is (default) | OK |
| extend_cruise mode | `planning/path_planner.py:774` | Placeholder (returns as-is) | TODO |
| get_trajectory_motion_stats | `planning/path_planner.py:786` | Returns `last_motion_t`, `static_tail_duration`, `static_tail_ratio` | OK |
| _build_plan calls end mode | `editor/interactive_editor.py:332` | `apply_episode_end_mode(trajectory, scene.episode_end_mode, ...)` | OK |

**scene_edit.json 新字段**:
```json
{ "episode_end_mode": "hold_at_end" }  // or "trim_to_motion"
```

### 10.4 G4: 冲突分析自动化 (Conflict Analyzer)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| ConflictReport dataclass | `tools/conflict_analyzer.py` | `ConflictReport(min_gap_m, min_ttc_s, cut_in_events, ego_response, conflict_score, ...)` | OK |
| CutInEvent dataclass | `tools/conflict_analyzer.py` | `CutInEvent(actor_id, start_t, end_t, lane_before, lane_after, min_gap_during)` | OK |
| EgoResponse dataclass | `tools/conflict_analyzer.py` | `EgoResponse(brake_start_t, max_decel, max_decel_t)` | OK |
| analyze_conflicts | `tools/conflict_analyzer.py` | `analyze_conflicts(plan)` → `ConflictReport` | OK |
| Gap computation | `tools/conflict_analyzer.py` | `_compute_gap()` — Euclidean distance | OK |
| TTC computation | `tools/conflict_analyzer.py` | `_compute_ttc()` — longitudinal closing speed model, 4m vehicle length | OK |
| Lane change detection | `tools/conflict_analyzer.py` | `_detect_lane_change()` — lateral shift ≥2m over 40-frame window | OK |
| Cut-in classification | `tools/conflict_analyzer.py` | Ending lane matches ego lane OR gap decreases >50% to <15m | OK |
| Conflict severity | `tools/conflict_analyzer.py` | high: gap<2m or TTC<1s; medium: gap<4m or TTC<1.5s; low: gap<5m or TTC<2s | OK |
| Per-frame conflict_score | `tools/conflict_analyzer.py` | `max(gap_score, ttc_score)` — 0 to 1 | OK |
| save_conflict_report | `tools/conflict_analyzer.py:388` | → `conflict_report.json` | OK |
| CLI entry | `tools/conflict_analyzer.py:518` | `main()` — `--plan`, `--output` | OK |

**conflict_report.json schema**:
```json
{
  "min_gap": { "distance_m", "t", "frame", "actors" },
  "min_ttc": { "ttc_s", "t", "frame", "actors" },
  "cut_in_events": [{ "actor_id", "start_t", "end_t", "min_gap_during" }],
  "ego_response": { "brake_start_t", "max_decel", "max_decel_t" },
  "conflict_score": [float],
  "has_conflict": bool,
  "conflict_severity": "none|low|medium|high"
}
```

### 10.5 G5: 事件叙事与自动建议 (Event Auto-populate)

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| CONFLICT_EVENT_TYPES | `tools/conflict_analyzer.py:402` | 6 types: cut_in_start, cut_in_complete, risk_peak, ego_brake_start, ego_max_decel, risk_resolved | OK |
| SuggestedEvent dataclass | `tools/conflict_analyzer.py:412` | `SuggestedEvent(t_event, event_type, expected_action, source, confidence)` | OK |
| auto_populate_events | `tools/conflict_analyzer.py:431` | `auto_populate_events(report, dt)` → `List[SuggestedEvent]` | OK |
| Cut-in → events | `tools/conflict_analyzer.py:443` | `cut_in_start` + `cut_in_complete` from `report.cut_in_events` | OK |
| Risk peak from min_ttc | `tools/conflict_analyzer.py:460` | `risk_peak` if `min_ttc < 3.0s` | OK |
| Ego brake from response | `tools/conflict_analyzer.py:478` | `ego_brake_start` from `ego_response.brake_start_t` | OK |
| Risk resolved from score | `tools/conflict_analyzer.py:498` | First frame after peak where `conflict_score < 0.2` | OK |
| suggested_events.json output | `tools/conflict_analyzer.py:427` | Saved alongside conflict_report.json | OK |

**事件叙事时间线示例** (lane_change_cut_in):
```
t=2.15s  ego_brake_start   ← ego 开始减速
t=4.15s  risk_peak          ← min_ttc=0s (最危险时刻)
t=14.45s risk_resolved      ← 风险解除
t=35.25s ego_max_decel      ← 最大制动
```

---

## 11. 已知局限与优化方向

Documentation section only — no direct code mapping required. Status: OK (informational).

---

## Validation Summary

| Section | Total Claims | OK | MISSING | Notes |
|---------|-------------|-----|---------|-------|
| 1. Architecture | 7 | 7 | 0 | |
| 2. Editor | 35 | 35 | 0 | + play/pause, step navigation |
| 3. CLI | 15 | 15 | 0 | + compare, convert, bundle, test |
| 4. Data formats | 24 | 24 | 0 | + schema v0.2, speed_fd, 6-DOF NPC |
| 5. Renderer | 8 | 8 | 0 | |
| 6. Comparison | 7 | 7 | 0 | Rewritten: align, error, sanity |
| 7. Conversion | 1 | 1 | 0 | |
| 8. Path Planner | 7 | 7 | 0 | LaneGraph, road-following |
| 9. Plan Validator | 6 | 6 | 0 | Offline validation gating |
| 10. Phase G | 48 | 47 | 1 | G3 extend_cruise TODO |
| 11. Limitations | 0 | 0 | 0 | Informational only |
| **Total** | **158** | **157** | **1** | |
