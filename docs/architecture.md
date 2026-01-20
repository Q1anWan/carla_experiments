# CARLA Experiment Client - Architecture and Interfaces

This document describes the program logic, module responsibilities, and public interfaces.
It is intended to help AI agents modify the project safely.

## 1) Runtime data flow (Plan -> Validate -> Render)

1. `map_exporter` generates map assets (centerlines, junctions, spawn candidates).
2. `planner_compiler` compiles episode configs into `plan.json` + `events_plan.json`.
3. `validator` checks event presence, lane-change recognition, and map feasibility.
4. `renderer` replays trajectories in CARLA, records video and telemetry.
5. Outputs are written under `outputs/<episode_id>/` (plan, events, validation, video, telemetry).

## 2) Runtime data flow (legacy run_scenario)

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
  - Subcommands: `map`, `plan`, `validate`, `render`, `pipeline`, `preview`, `editor`, `suite`.
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

### `carla_experiment_client/scenarios/*`
- Responsibility: spawn actors and define per-frame behavior.
- `BaseScenario` helper methods:
  - `_spawn_vehicle`, `_spawn_walker`, `_spawn_background_traffic`, `_apply_ego_tm`.
  - `find_spawn_point(...)` selects spawn points based on lane/junction constraints.
- Each scenario implements `build(world, tm, rng) -> ScenarioContext`.
- Scenario registry: `scenarios/registry.py` maps scenario id to class.

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
