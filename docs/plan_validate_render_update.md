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

- Replay uses teleport follower (no tracking controller for ego yet).
- Validation is basic; TTC checks run in editor only (not validator).

## Enhanced Editor Features

The interactive editor now includes:

**Lane Snapping**
- Toggle snap mode with `Snap` button or `s` key
- Keyframes automatically snap to nearest lane centerline when enabled

**Kinematic Analysis**
- Speed limit check (default: 30 m/s)
- Acceleration/deceleration limits (default: 8/-10 m/s²)
- Lateral acceleration / curvature check (default: 5 m/s²)
- Violations shown as purple triangles on map

**TTC Visualization**
- Time-to-collision warnings (default: < 3s)
- Shown as pink X markers on map

**Undo/Redo**
- Full undo/redo stack (up to 50 states)
- `Ctrl+Z` / `Ctrl+Y` or `Undo`/`Redo` buttons

**Keyboard Shortcuts**
- `a` - Add keyframe mode
- `m` - Move keyframe mode
- `d` - Delete keyframe mode
- `s` - Toggle snap to centerline
- `e` - Delete nearest event at current time
- `space` - Run analysis
- `r` - Reset view to fit all actors
- `h` - Show help in console
- `Delete` - Delete selected keyframe
- `Ctrl+Z/Y` - Undo/Redo
- `Ctrl+S` - Save scene
- `Ctrl+E` - Export plan
- `Scroll` - Zoom in/out

**Status Bar**
- Shows current mode and snap state
- Quick reference for keyboard shortcuts

## Scene Design Library

Pre-converted scene designs from validated telemetry runs are available in `outputs/scene_designs/`:

```
outputs/scene_designs/
├── lane_change_cut_in/
│   ├── scene_edit.json      # Editable keyframe scene
│   └── comparison.png       # Telemetry vs keyframe comparison
├── unprotected_left_turn/
├── yield_to_emergency/
├── red_light_conflict/
├── pedestrian_emerge/
└── highway_merge/
```

### Conversion Workflow

Scene designs were created by converting validated telemetry runs (`runs/final_fix_validation/`) to editor-compatible `scene_edit.json` format:

1. Extract ego trajectory from `telemetry.csv` (world_x, world_y, speed columns)
2. Identify key NPC actors from `telemetry.json` by role_name and distance to ego
3. Sample keyframes at regular intervals (every 10s for ego, key moments for NPCs)
4. Preserve events from original `events.json`
5. Validate against original telemetry with `comparison.png`

### Comparison Visualizations

Each `comparison.png` contains 3 panels:
- **Left**: XY trajectory overlay (solid = CSV telemetry, dashed = keyframe interpolation)
- **Center**: Ego speed comparison with event markers
- **Right**: Position interpolation error curve with mean/max statistics

Interpolation error statistics:

| Scenario | Mean Error | Max Error |
|----------|-----------|-----------|
| lane_change_cut_in | 1.26m | 5.63m |
| unprotected_left_turn | 2.27m | 8.28m |
| yield_to_emergency | 1.60m | 5.18m |
| red_light_conflict | 1.13m | 7.76m |
| pedestrian_emerge | 2.56m | 10.49m |
| highway_merge | 1.56m | 6.67m |

Errors are due to linear interpolation at curves; spline interpolation would reduce them.

## Editor Bug Fixes

- **View reset on load**: Editor now calls `_reset_view()` after loading actors, ensuring all trajectories are visible immediately instead of showing only a zoomed-in portion.
- **Keyframe table**: Press `t` to print a formatted keyframe table to the console for debugging.
- **Duplicate actor IDs**: Converted scenes from telemetry fixed duplicate "autopilot" actor IDs by appending `_2`, `_3` suffixes.

## Related Config Changes

- `configs/globals.yaml`: added `validation.lane_change_window_s`.

