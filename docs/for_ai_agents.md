# CARLA Experiment Client - AI Agent Reference

This document provides essential information for AI agents working with this codebase.

## Project Overview

**Purpose**: Generate driving scenario videos and experimental stimuli for human factors research using CARLA simulator.

**Architecture**: Client-server model where CARLA runs on Windows and the Python client runs on Linux (typically Ubuntu).

## Key Concepts

### Scenarios

Six driving scenarios, each as a self-contained package:

| Scenario ID | Map | Primary Event |
|-------------|-----|---------------|
| highway_merge | Town04 | vehicle_cut_in |
| lane_change_cut_in | Town05 | vehicle_cut_in |
| pedestrian_emerge | Town05 | avoid_pedestrian |
| unprotected_left_turn | Town03 | yield_left_turn |
| red_light_conflict | Town03 | stop_for_red_light |
| yield_to_emergency | Town04 | yield_to_emergency |

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

# With render preset
python run_scenario.py --scenario highway_merge --render-preset fast
```

### Add New Scenario

1. Create directory: `scenarios/new_scenario/`
2. Create files:
   - `scenario.py` with class inheriting from `BaseScenario`
   - `config.yaml` and `config_30s.yaml`
   - `__init__.py` exporting the class
   - `README.md` documentation
3. Register in `registry.py`

### Modify Event Detection

Edit `carla_experiment_client/events/extractor.py`:
- `tick()` method processes each frame
- `_emit()` creates events with proper timing
- Events are filtered by `min_event_time_s`

### Adjust Timing

Scenario timing uses frame numbers:
- `trigger_frame` - When event triggers
- `duration_frames` - How long action lasts
- Convert to time: `t = frame / fps`

When using render presets, frame values auto-scale.

## Configuration Parameters

### Scenario Config

```yaml
id: scenario_name
map: Town04
seed: 42
duration: 30.0
fps: 20
fixed_delta_seconds: 0.05
events:
  min_event_time_s: 8.0
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
| `trigger_frame` | Frame when main event triggers |
| `duration_frames` | Duration of triggered action |
| `relocate_on_trigger` | Teleport NPC relative to ego |
| `ego_brake_frame` | Frame to force ego braking |
| `background_vehicle_count` | Number of background vehicles |

## Data Formats

### telemetry.json

```json
{
  "metadata": {
    "fps": 20,
    "coordinate_system": "SAE_J670"
  },
  "frames": [
    {
      "frame": 0,
      "t_sim": 0.0,
      "ego": {
        "velocity": {"vx": 12.5, "vy": 0.1, "vz": 0.0},
        "acceleration": {"ax": -1.2, "ay": 0.05, "az": 0.0}
      }
    }
  ]
}
```

### events.json

```json
{
  "events": [
    {
      "t": 19.0,
      "type": "vehicle_cut_in",
      "decision_text": "Brake",
      "reason_text": "Vehicle cutting in ahead"
    }
  ]
}
```

## Debugging Tips

### Connection Issues

```bash
# Check CARLA connection
python debug_tools.py check_connection --host 192.168.31.32 --port 2000
```

### Event Detection

```bash
# Test event detection
python debug_tools.py events --scenario highway_merge --frames 600 --out debug/events.json
```

### Import Verification

```python
from carla_experiment_client.scenarios.registry import get_scenario_ids
print(get_scenario_ids())
```

## Registry Functions

```python
from carla_experiment_client.scenarios.registry import (
    get_scenario_ids,           # List all scenario IDs
    get_scenario_config_path,   # Get config path for scenario
    build_scenario,             # Build scenario from config
)
```

## Error Handling

Common errors and solutions:

| Error | Solution |
|-------|----------|
| ModuleNotFoundError: carla | Add CARLA Python API to PYTHONPATH |
| Connection refused | Start CARLA server, check firewall |
| Spawn point invalid | Scenario will use fallback search |
| Version mismatch | Set `allow_version_mismatch: true` |

## File Locations

| Type | Location |
|------|----------|
| Scenario code | `carla_experiment_client/scenarios/<id>/scenario.py` |
| Scenario config | `carla_experiment_client/scenarios/<id>/config*.yaml` |
| Event extractor | `carla_experiment_client/events/extractor.py` |
| Telemetry recorder | `carla_experiment_client/telemetry/recorder.py` |
| SAE J670 transformer | `carla_experiment_client/telemetry/sae_j670.py` |
| Output runs | `runs/<timestamp>/` |

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

- Town03: Intersections, traffic lights
- Town04: Highway, multi-lane
- Town05: Urban, multi-lane
