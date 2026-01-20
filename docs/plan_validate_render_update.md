# Plan-Validate-Render Pipeline Update (MVP)

This document summarizes the recent MVP work to decouple planning from replay, and describes the current structure and workflow.

## Scope of Changes

- Added a **Plan -> Validate -> Render** pipeline with map export and a consistent plan schema.
- Introduced a **2D interactive editor** (Matplotlib-based) for multi-actor keyframe editing, timeline scrubbing, event markers, and basic conflict/feasibility overlays.
- Added a **unified CLI** with both subcommands and an interactive menu to run map export, plan/validate/render, preview, editor, and suites.
- Improved runtime robustness with **timeouts** and **logging** in long-running steps and ffmpeg encoding.
- Updated validation logic (lane-change window and lane width tolerance) to reduce false failures.

## Architecture Overview

**Data flow**
1) `map_exporter` generates map assets (centerlines, junctions, spawn candidates).
2) `planner_compiler` converts episode configs into `plan.json` and `events_plan.json`.
3) `validator` checks feasibility and basic event recognition before render.
4) `renderer` replays trajectories in CARLA and records `master_video.mp4` + telemetry + events.
5) `interactive_editor` edits multi-actor keyframes and exports editable `scene_edit.json`, plus replay-ready `plan.json/events_plan.json`.

**Key principles**
- Planning is deterministic and separate from rendering.
- Rendering is replay-only (no map reasoning during playback).
- All outputs are config/plan driven for reproducibility.

## Modules and Responsibilities

### Planning
- `carla_experiment_client/planning/trajectory_schema.py`
  - Defines `plan.json`/`events_plan.json` schema with versioning.
- `carla_experiment_client/planning/map_exporter.py`
  - Exports: `map_graph.json`, `lane_centerlines.geojson`, `junction_areas.geojson`, `spawn_candidates.json`.
- `carla_experiment_client/planning/planner_compiler.py`
  - Compiles episode YAML into time-parameterized trajectories.
  - Current MVP supports keep-lane + lane-change.
- `carla_experiment_client/planning/validator.py`
  - Checks event presence, lane change recognition, and map feasibility.
  - Uses lane width for tolerance and a configurable lane-change window.

### Rendering
- `carla_experiment_client/render/replay_controller.py`
  - Teleport follower for deterministic replay (MVP).
- `carla_experiment_client/render/renderer.py`
  - Spawns actors, replays trajectories, records video and telemetry.
  - Writes `events.json` aligned with `events_plan.json`.

### Editor
- `carla_experiment_client/editor/interactive_editor.py`
  - Matplotlib GUI: keyframes, time slider, event markers, actor management.
  - Conflict/feasibility overlays via map centerline distance and actor spacing.
  - Exports `scene_edit.json` + `plan.json` + `events_plan.json`.
  - Headless mode for automation (`--headless`) exports without GUI.
- `carla_experiment_client/editor/mvp.py`
  - Simple preview renderer for `plan.json` + map centerlines.

### CLI / Entry Points
- `carla_experiment_client/cli.py` and `cli.py`
  - Unified CLI with subcommands and interactive menu:
    - `map`, `plan`, `validate`, `render`, `pipeline`, `preview`, `editor`, `suite`.
  - Editor supports `--headless` for AI/automation runs.

## Output Artifacts

`outputs/<episode_id>/`
- `plan.json`
- `events_plan.json`
- `scene_edit.json` (editor only)
- `validation_report.json`
- `master_video.mp4`
- `telemetry.json`
- `telemetry.csv`
- `events.json`
- `run_metadata.json`

## How to Run (Examples)

```bash
# Map export
python -m carla_experiment_client.cli map --map Town05

# Plan / Validate / Render
python -m carla_experiment_client.cli pipeline --episode P1_T2_lane_change --quick

# Interactive editor (GUI)
python -m carla_experiment_client.cli editor --map-dir data/maps/Town05 --episode-id P1_T2_lane_change

# Editor (headless, automation)
python -m carla_experiment_client.cli editor --map-dir data/maps/Town05 --episode-id demo --headless
```

## Current Limitations (MVP)

- Editor does not yet snap to lane centerlines or enforce kinematic limits.
- Replay uses teleport follower (no tracking controller for ego yet).
- Validation is basic; collision/TTC/jerk checks are not implemented yet.

## Related Config Changes

- `configs/globals.yaml`: added `validation.lane_change_window_s`.

