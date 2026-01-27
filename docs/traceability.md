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
| Interactive menu | `cli.py:216` | `run_menu()` | OK |
| Argument parser | `cli.py:368` | `build_parser()` | OK |
| Entry point | `cli.py:452` | `main()` | OK |

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

### 4.3 telemetry.json

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| TelemetryRecorder | `telemetry/recorder.py:63` | `TelemetryRecorder` | OK |
| TelemetryFrame | `telemetry/recorder.py:41` | `TelemetryFrame(frame, t_sim, ...)` | OK |
| TelemetryConfig | `telemetry/recorder.py:30` | `TelemetryConfig` | OK |

### 4.4 telemetry.csv

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| CSV output | `telemetry/recorder.py` | `TelemetryRecorder.save()` → writes CSV | OK |
| SAE J670 transform | `telemetry/sae_j670.py` | `SAEJ670Transformer`, `VehicleState` | OK |

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
| Comparison plot generation | — | — | **MISSING** |
| Summary statistics | — | — | **MISSING** |

**补齐方案**: Create `carla_experiment_client/tools/make_comparison.py` with:
- `generate_comparison(scene_path, csv_path, events_path, output_dir, actors_filter)` function
- Outputs: `comparison.png` (3-panel plot) + `summary.json` (mean/p95/max error)
- Verification: `python -m carla_experiment_client.tools.make_comparison --scene <path> --telemetry-csv <path> --output <dir>`

---

## 7. 遥测 → 场景设计转换工具

| Doc Claim | Code File | Symbols | Status |
|-----------|-----------|---------|--------|
| Telemetry to scene_edit conversion | — | — | **MISSING** |

**补齐方案**: Create `carla_experiment_client/tools/convert_telemetry_to_scene.py` with:
- `convert_telemetry(csv_path, json_path, events_path, output_path, town, duration, dt)` function
- Extracts ego keyframes from CSV, NPC keyframes from JSON, events from events.json
- Verification: `python -m carla_experiment_client.tools.convert_telemetry_to_scene --csv <path> --json <path> --events <path> --output <path>`

---

## 8. 已知局限与优化方向

Documentation section only — no direct code mapping required. Status: OK (informational).

---

## Validation Summary

| Section | Total Claims | OK | MISSING | Notes |
|---------|-------------|-----|---------|-------|
| 1. Architecture | 7 | 7 | 0 | |
| 2. Editor | 35 | 35 | 0 | |
| 3. CLI | 11 | 11 | 0 | |
| 4. Data formats | 17 | 17 | 0 | |
| 5. Renderer | 8 | 8 | 0 | |
| 6. Comparison | 2 | 0 | 2 | Need tools/make_comparison.py |
| 7. Conversion | 1 | 0 | 1 | Need tools/convert_telemetry_to_scene.py |
| 8. Limitations | 0 | 0 | 0 | Informational only |
| **Total** | **81** | **78** | **3** | |
