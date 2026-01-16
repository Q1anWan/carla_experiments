# CARLA Experiment Client - AI Agent Reference

This document provides essential information for AI agents working with this codebase.

## Project Overview

**Purpose**: Generate driving scenario videos and experimental stimuli for human factors research using CARLA simulator.

**Architecture**: Client-server model where CARLA runs on Windows and the Python client runs on Linux (typically Ubuntu).

## Key Concepts

### Scenarios

Six driving scenarios, each as a self-contained package:

| Scenario ID | Map | Primary Event | Key Actor |
|-------------|-----|---------------|-----------|
| highway_merge | Town04 | vehicle_cut_in | merge_vehicle |
| lane_change_cut_in | Town05 | vehicle_cut_in | cut_in_vehicle |
| pedestrian_emerge | Town05 | avoid_pedestrian | walker |
| unprotected_left_turn | Town03 | yield_left_turn | oncoming_vehicle |
| red_light_conflict | Town03 | stop_for_red_light | cross_vehicle |
| yield_to_emergency | Town04 | yield_to_emergency | emergency |

### Coordinate Systems

- **CARLA World**: X forward, Y right, Z up (meters)
- **SAE J670 Vehicle**: X forward, Y left, Z up (automotive standard)
- Conversion: `SAE_Y = -CARLA_Y`

### Experiment Design

2x3 factorial design:
- **Voice Conditions**: V0 (silent), V1 (decision only), V2 (decision + reason)
- **Robot Conditions**: R0 (no robot), R1 (robot precue)

## Important Files

### Entry Points

| File | Purpose |
|------|---------|
| `run_scenario.py` | Execute scenario and record data |
| `render_variants.py` | Generate experiment stimulus variants |
| `debug_tools.py` | Step-by-step debugging utilities |

### Core Modules

| Path | Purpose |
|------|---------|
| `carla_experiment_client/scenarios/` | Scenario implementations |
| `carla_experiment_client/telemetry/` | SAE J670 telemetry recording |
| `carla_experiment_client/events/extractor.py` | Event detection logic |
| `carla_experiment_client/config.py` | Configuration dataclasses |

### Scenario Package Structure

```
scenarios/<scenario_id>/
├── __init__.py       # Exports scenario class
├── scenario.py       # Scenario implementation
├── config.yaml       # Short test config
├── config_30s.yaml   # Full experiment config
└── README.md         # Scenario documentation
```

## Common Tasks

### Run a Scenario

```bash
# By ID (uses config_30s.yaml)
python run_scenario.py --scenario highway_merge --out runs/test

# By config path
python run_scenario.py --scenario carla_experiment_client/scenarios/highway_merge/config_30s.yaml

# With render preset (for testing)
python run_scenario.py --scenario highway_merge --render-preset test
```

### Analyze Telemetry

```python
import json
import math

with open("runs/test/telemetry.json") as f:
    data = json.load(f)

for frame in data["frames"]:
    ego = frame.get("ego", {})
    ego_pos = ego.get("position", {})

    for actor in frame.get("actors", []):
        role = actor.get("role_name", "")
        apos = actor.get("position", {})
        dist = math.sqrt((ego_pos["x"] - apos["x"])**2 + (ego_pos["y"] - apos["y"])**2)
        print(f"Frame {frame['frame']}: {role} at {dist:.1f}m from ego")
```

---

## CRITICAL: Spawn Fallback Issues

### The Problem

CARLA's `world.try_spawn_actor()` may fail if the spawn location:
1. Collides with existing actors
2. Is not on a valid road surface
3. Is inside geometry

When `_spawn_vehicle()` in `base.py` fails all candidate positions, it falls back to **random map spawn points**, placing actors hundreds of meters away.

### Identifying Spawn Issues

Check `run.log` for warnings:
```
[WARNING] Spawn fallback used for merge_vehicle        # Minor offset used
[WARNING] Random spawn fallback used for merge_vehicle # CRITICAL: Random location!
```

### Solutions

#### 1. Use Waypoint Navigation (Recommended)

Instead of `offset_transform()`, use waypoint navigation to find valid road positions:

```python
# Find adjacent lane
ego_wp = world.get_map().get_waypoint(ego_spawn.location)
adjacent_wp = ego_wp.get_right_lane() or ego_wp.get_left_lane()

if adjacent_wp and adjacent_wp.lane_type == carla.LaneType.Driving:
    # Navigate ahead in adjacent lane
    ahead_wps = adjacent_wp.next(distance_m)
    if ahead_wps:
        spawn_transform = ahead_wps[0].transform
        spawn_transform.location.z += 0.3  # Ensure above ground

# For spawning behind ego
behind_wps = ego_wp.previous(distance_m)
if behind_wps:
    spawn_transform = behind_wps[0].transform
```

#### 2. Validate Spawn Distance

After spawning, verify the actor is at the intended distance:

```python
actual_dist = ego_spawn.location.distance(vehicle.get_location())
if actual_dist > 50.0:
    logging.warning("Actor spawned far from ego (%.1fm)", actual_dist)
```

#### 3. Walker Spawn Issues

Walkers **must** spawn on the navigation mesh. CARLA will silently move them to random navigation points if the position is invalid.

For reliable walker spawning:
1. Try to find sidewalk waypoints using `get_right_lane()`/`get_left_lane()`
2. Use `relocate_on_trigger: true` to respawn walker at trigger time
3. **IMPORTANT**: Add respawned walkers to `ctx.actors` for telemetry tracking:

```python
if spawn_success:
    ctx.actors.append(new_walker)
    ctx.actors.append(new_controller)
```

---

## Frame Scaling with Render Presets

### How It Works

When using render presets that change FPS, trigger frame values are **automatically scaled**:

```
Config FPS: 20, Duration: 60s → Total frames: 1200
Test Preset: 10 fps, Duration: 60s → Total frames: 600
Scale factor: 600/1200 = 0.5

trigger_frame: 800 (at 20fps) → 400 (at 10fps)
```

### Important Notes

- Frame scaling happens in `config.py::apply_render_preset()`
- Only parameters with "frame" in the key name are scaled
- Scale factor: `(new_duration * new_fps) / (old_duration * old_fps)`

### Verifying Triggers

Check `run.log` for trigger messages:
```
[INFO] Merge maneuver started at frame 200, steer=0.30, distance=9.8m
[INFO] Cut-in maneuver started at frame 400, steer=-0.25
[INFO] Left turn maneuver started at frame 390, steer=-0.20
```

---

## Scenario-Specific Notes

### highway_merge

**Key Issue**: merge_vehicle spawn in adjacent lane may fail due to lane configuration.

**Solution**: Use waypoint navigation to find valid adjacent lane position:
```python
merge_wp = waypoint.get_right_lane() or waypoint.get_left_lane()
if merge_wp:
    ahead_wps = merge_wp.next(ahead_m)
```

**Config Parameters**:
- `merge_trigger_frame`: When merge maneuver starts
- `merge_duration_frames`: How long the merge lasts
- `merge_steer`: Steering value during merge (sign auto-detected)

### lane_change_cut_in

**Key Issue**: cut_in_vehicle needs to be in the correct adjacent lane.

**Solution**: Waypoint navigation + auto steer direction detection:
```python
cut_in_on_right = (adjacent_wp == ego_wp.get_right_lane())
steer = -abs(base_steer) if cut_in_on_right else abs(base_steer)
```

**Config Parameters**:
- `cut_in_trigger_frame`: When cut-in starts
- `cut_in_steer`: Base steering value (direction auto-detected)
- `cut_in_relocate_on_trigger`: Set `false` to avoid sudden teleport

### pedestrian_emerge

**Key Issue**: Walker spawns at random navigation points.

**Solution**:
1. Try sidewalk waypoints first
2. Use `relocate_on_trigger: true`
3. Add respawned walker to `ctx.actors` for telemetry

**Config Parameters**:
- `trigger_frame` or `trigger_distance`: When walker starts crossing
- `walker_speed`: Walker movement speed
- `relocate_on_trigger`: Respawn walker relative to ego

### unprotected_left_turn

**Key Issue**: `use_autopilot_turn: true` doesn't actually force left turn.

**Solution**: Set `use_autopilot_turn: false` to use manual steering control.

**Config Parameters**:
- `use_autopilot_turn`: `false` for reliable turn
- `approach_frames`: Frames before turn starts
- `turn_frames`: Duration of turn maneuver
- `turn_steer`: Steering value (sign auto-detected by `turn_steer_auto`)

### yield_to_emergency

**Key Issue**: Emergency vehicle spawning behind ego fails with `offset_transform()`.

**Solution**: Use `waypoint.previous()` to find valid road position behind ego:
```python
prev_wps = ego_wp.previous(distance_behind)
if prev_wps:
    emergency_spawn = prev_wps[0].transform
```

**Config Parameters**:
- `emergency_spawn_frame`: When to spawn (delayed spawn)
- `emergency_spawn_distance_m`: Negative = behind ego
- `emergency_boost_frames`: How long to boost emergency speed

### red_light_conflict

**Key Issue**: cross_vehicle needs to be at correct position for timing.

**Solution**: Use junction waypoints and traffic light detection.

**Config Parameters**:
- `cross_vehicle_count`: Number of cross traffic vehicles
- `cross_trigger_frame`: When cross vehicle starts
- `traffic_light_threshold_m`: Distance to detect red light

---

## Telemetry Analysis Tips

### Check Key Actor Distances

```python
# Identify if scenario event can occur
for frame in data["frames"]:
    for actor in frame.get("actors", []):
        role = actor.get("role_name", "")
        if role == "merge_vehicle":  # or cut_in_vehicle, emergency, etc.
            # Calculate distance to ego
            dist = calculate_distance(frame["ego"]["position"], actor["position"])
            if dist < 30:  # Actor close enough for interaction
                print(f"Frame {frame['frame']}: {role} within interaction range")
```

### Detect Oscillation

```python
# High direction changes indicate oscillation
direction_changes = 0
for i in range(2, len(positions)):
    prev_dx = positions[i-1][0] - positions[i-2][0]
    curr_dx = positions[i][0] - positions[i-1][0]
    if prev_dx * curr_dx < 0:  # Sign change
        direction_changes += 1

if direction_changes > 100:
    print("Warning: High oscillation detected")
```

### Verify Turn Occurred

```python
# Check total yaw change
total_yaw = 0
for i in range(1, len(yaws)):
    diff = yaws[i] - yaws[i-1]
    while diff > 180: diff -= 360
    while diff < -180: diff += 360
    total_yaw += abs(diff)

if total_yaw > 45:
    print(f"Turn detected: {total_yaw:.1f} degrees total yaw change")
```

---

## Telemetry Data Structure

### Output Files

Each scenario run produces two telemetry files:
- `telemetry.json` - Complete data with ego vehicle and tracked actors
- `telemetry.csv` - Ego-only tabular format for analysis

### JSON Schema Overview

```json
{
  "metadata": {
    "fps": 20,
    "total_frames": 600,
    "coordinate_system": "SAE_J670",
    "units": {
      "position": "meters",
      "velocity": "m/s",
      "acceleration": "m/s^2",
      "angular_velocity": "deg/s"
    }
  },
  "frames": [
    {
      "frame": 100,
      "t_sim": 5.0,
      "t_world": 5.123,
      "dt": 0.05,
      "ego": { ... },
      "actors": [ ... ]
    }
  ]
}
```

### Ego Vehicle State

```json
{
  "position": {"x": 123.45, "y": -67.89, "z": 0.5},
  "velocity": {"vx": 12.5, "vy": 0.1, "vz": 0.0},
  "acceleration": {"ax": -1.2, "ay": 0.05, "az": 0.0},
  "angular_velocity": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 2.5},
  "orientation": {"roll": 0.0, "pitch": -0.5, "yaw": 45.0},
  "speed": 12.5,
  "control": {"throttle": 0.5, "brake": 0.0, "steer": 0.1}
}
```

### Actor State

```json
{
  "id": 42,
  "type": "vehicle",
  "type_id": "vehicle.tesla.model3",
  "role_name": "merge_vehicle",
  "position": {"x": 135.12, "y": -65.45, "z": 0.5},
  "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 47.5},
  "distance_to_ego": 12.34,
  "velocity": {"x": 13.2, "y": -0.5, "z": 0.0},
  "speed": 13.21
}
```

### CSV Columns (23 columns)

| Column | Type | Description |
|--------|------|-------------|
| frame | int | Frame index |
| t_sim | float | Simulation time (s) |
| world_x, world_y, world_z | float | World position (m) |
| vx, vy, vz | float | Velocity (m/s) |
| ax, ay, az | float | Acceleration (m/s²) |
| roll_rate, pitch_rate, yaw_rate | float | Angular velocity (deg/s) |
| roll, pitch, yaw | float | Orientation (deg) |
| speed | float | Total speed (m/s) |
| throttle, brake, steer | float | Control inputs |

### SAE J670 Coordinate System

```
        +Z (Up)
         |
         |    +X (Forward)
         |   /
         | /
    +Y __|/________
  (Left) O (Vehicle Center)
```

**Conversion from CARLA**: `SAE_Y = -CARLA_Y` (CARLA uses Y-right)

---

## Visualization Generation

### Visualization Module

Location: `carla_experiment_client/visualization/telemetry_map.py`

### Generate Visualizations

```bash
# Analyze single scenario
python -m carla_experiment_client.visualization.telemetry_map \
    --runs runs/test --scenario highway_merge --output ./analysis

# Analyze all scenarios in directory
python -m carla_experiment_client.visualization.telemetry_map \
    --runs runs/batch_20260116 --output ./analysis
```

### Output Files

For each scenario, generates:
- `{scenario}_trajectory.png` - 2D map with speed coloring
- `{scenario}_trajectory_accel.png` - 2D map with acceleration coloring
- `{scenario}_appearance.png` - Actor first-appearance analysis
- `{scenario}_summary.json` - Numeric summary statistics

### Visualization Contents

**Main Trajectory Plot (4 panels)**:
1. **Top-left**: 2D trajectory map with ego path (colored by speed/accel), actor paths, event markers
2. **Top-right**: Speed and acceleration time series with event markers
3. **Bottom-left**: Control inputs (steer/throttle/brake) with oscillation highlighting
4. **Bottom-right**: Actor distance to ego over time (first-appearance marked)

### Programmatic Usage

```python
from carla_experiment_client.visualization import generate_scenario_report

summary = generate_scenario_report(
    telemetry_path="runs/test/telemetry.json",
    events_path="runs/test/events.json",
    output_dir="./analysis",
    scenario_name="highway_merge"
)

print(f"Events: {summary['events_count']}")
print(f"Speed: {summary['speed_mean']:.1f} m/s")
print(f"Hard brakes: {summary['brake_hard_count']}")
```

### Summary Statistics

The `summary.json` includes:

```json
{
  "scenario": "highway_merge",
  "duration": 60.0,
  "total_frames": 600,
  "events_count": 2,
  "actors_count": 8,
  "steer_std": 0.0234,
  "steer_changes": 145,
  "lateral_accel_std": 0.156,
  "speed_mean": 15.2,
  "speed_max": 18.5,
  "accel_max": 2.1,
  "accel_min": -4.2,
  "brake_hard_count": 3,
  "key_actors": [
    {
      "role": "merge_vehicle",
      "type": "vehicle",
      "first_seen_t": 0.0,
      "first_seen_distance": 45.2,
      "min_distance": 5.9
    }
  ]
}
```

### Diagnosing Issues with Visualizations

| Issue | What to Check |
|-------|---------------|
| Actor not visible | `_appearance.png` - check first_seen_distance (should be <100m) |
| Event not occurring | `_trajectory.png` - check if actor path intersects ego |
| Oscillation | `_trajectory.png` (bottom-left) - yellow highlighted regions |
| Spawn failure | `_summary.json` - check first_seen_distance of key actors |

---

## Configuration Parameters

### Scenario Config

```yaml
id: scenario_name
map: Town04
seed: 42
duration: 60.0
fps: 20
fixed_delta_seconds: 0.05
events:
  min_event_time_s: 30.0
  voice_lead_time_s: 3.0
  robot_precue_lead_s: 0.5
scenario:
  trigger_frame: 380
  # ... scenario-specific params
```

### Important Parameters

| Parameter | Description |
|-----------|-------------|
| `ego_spawn_index` | Spawn point index (fallback if invalid) |
| `*_trigger_frame` | Frame when event triggers |
| `*_duration_frames` | Duration of triggered action |
| `relocate_on_trigger` | Teleport NPC relative to ego (avoid for realism) |
| `ego_speed_delta` | Traffic Manager speed difference (%) |
| `ego_follow_distance_m` | Following distance for smoother driving |
| `background_vehicle_count` | Number of background traffic vehicles |

---

## Error Handling

Common errors and solutions:

| Error | Solution |
|-------|----------|
| ModuleNotFoundError: carla | Add CARLA Python API to PYTHONPATH |
| Connection refused | Start CARLA server, check firewall |
| Random spawn fallback | Use waypoint navigation instead of offset_transform |
| Walker not tracked | Add respawned walker to ctx.actors |
| Version mismatch | Set `allow_version_mismatch: true` |
| Turn not occurring | Set `use_autopilot_turn: false` |

---

## Quick Reference

### Scenario IDs

- `highway_merge`
- `lane_change_cut_in`
- `pedestrian_emerge`
- `red_light_conflict`
- `unprotected_left_turn`
- `yield_to_emergency`

### Event Types

- `lane_change_left` / `lane_change_right`
- `brake_hard`
- `slow_down`
- `stop_for_red_light`
- `yield_to_emergency`
- `avoid_pedestrian`
- `vehicle_cut_in`
- `yield_left_turn`

### Maps

- Town03: Intersections, traffic lights (left turn, red light)
- Town04: Highway, multi-lane (merge, emergency)
- Town05: Urban, multi-lane (cut-in, pedestrian)

### Render Presets

| Preset | FPS | Duration | Resolution | Use Case |
|--------|-----|----------|------------|----------|
| fast | 10 | 6s | 640x360 | Quick syntax check |
| test | 10 | 60s | 640x360 | Full scenario test |
| final | 20 | 60s | 1280x720 | Production |

---

## Debugging Workflow

1. **Run with test preset**: `--render-preset test`
2. **Check run.log** for spawn warnings and trigger messages
3. **Analyze telemetry.json** for actor distances
4. **Identify issue**: spawn failure, timing, or control
5. **Fix**: Use waypoint navigation, adjust timing, or fix config
6. **Re-test** and verify telemetry shows expected behavior

### Key Log Messages to Watch

```bash
# Good - triggers fired
[INFO] Merge maneuver started at frame X
[INFO] Cut-in maneuver started at frame X
[INFO] Pedestrian trigger fired at frame X

# Bad - spawn issues
[WARNING] Random spawn fallback used for X
[WARNING] X spawned far from ego (Ym)

# Good - actor tracking
[INFO] Emergency vehicle spawned at frame X
[INFO] Walker track t=X: dist=Y pos=(...)
```
