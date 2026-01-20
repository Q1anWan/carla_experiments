# CARLA Experiment Client

A reproducible, headless-friendly client for generating CARLA driving scenario videos and experimental stimuli for human factors research.

## Key Features

- **6 Driving Scenarios**: Highway merge, lane change cut-in, pedestrian emerge, unprotected left turn, red light conflict, yield to emergency
- **SAE J670 Telemetry**: Vehicle state recording in standard automotive coordinate system
- **Event Detection**: Automatic extraction of driving decision points
- **Experiment Variants**: Generate 6 stimulus variants (Voice x Robot conditions)
- **Waypoint Navigation**: Reliable actor spawning using CARLA's road network
- **Frame Scaling**: Automatic trigger timing adjustment across render presets

## Project Structure

```
carla_experiment_client/
├── carla_experiment_client/          # Main Python package
│   ├── scenarios/                    # Scenario packages (code + config + docs)
│   │   ├── highway_merge/
│   │   │   ├── scenario.py           # Scenario implementation
│   │   │   ├── config.yaml           # Short test config
│   │   │   ├── config_30s.yaml       # Full 30s config
│   │   │   └── README.md             # Scenario documentation
│   │   ├── lane_change_cut_in/
│   │   ├── pedestrian_emerge/
│   │   ├── red_light_conflict/
│   │   ├── unprotected_left_turn/
│   │   ├── yield_to_emergency/
│   │   ├── base.py                   # Base scenario class
│   │   └── registry.py               # Scenario factory
│   │
│   ├── telemetry/                    # Telemetry recording (NEW)
│   │   ├── recorder.py               # Per-tick data recording
│   │   └── sae_j670.py              # SAE J670 coordinate transformer
│   │
│   ├── events/                       # Event detection
│   │   └── extractor.py              # Decision event extraction
│   │
│   ├── audio/                        # Audio generation
│   │   ├── tts.py                    # Text-to-speech
│   │   └── mux.py                    # Audio-video muxing
│   │
│   ├── sensors/                      # Camera recording
│   │   └── camera_recorder.py        # RGB camera capture
│   │
│   ├── run_scenario.py               # Main execution pipeline
│   ├── render_variants.py            # Variant generation
│   ├── config.py                     # Configuration models
│   └── carla_client.py               # CARLA connection
│
├── configs/                          # Global configuration files
│   ├── client.yaml                   # CARLA server connection
│   ├── render_presets.yaml           # Quality/speed presets
│   └── natural_driving.yaml          # Traffic manager presets
│
├── docs/                             # Documentation
│   ├── api/                          # API documentation
│   │   ├── output_formats.md         # Output file descriptions
│   │   ├── telemetry_schema.md       # Telemetry data schema
│   │   ├── events_schema.md          # Event data schema
│   │   └── coordinate_systems.md     # Coordinate system guide
│   └── architecture.md               # System architecture
│
└── runs/                             # Output directory
```

## Quick Start

### Prerequisites

- Python 3.12+
- CARLA 0.9.x server (Windows or Linux)
- CARLA Python API in environment
- ffmpeg (for video encoding)
- Optional: edge-tts (for voice synthesis)

### Run a Scenario

```bash
# Activate environment
conda activate CARLA312

# Run using scenario ID (uses config_30s.yaml by default)
python run_scenario.py --scenario highway_merge --out runs/test

# Or specify config file directly
python run_scenario.py --scenario carla_experiment_client/scenarios/highway_merge/config_30s.yaml

# Use render preset for quick testing
python run_scenario.py --scenario highway_merge --render-preset fast --out runs/quick_test
```

### Output Files

Each run produces:

```
runs/test/
├── scenario.yaml          # Configuration used
├── run_metadata.json      # Execution metadata
├── events.json            # Detected events
├── telemetry.json         # Vehicle telemetry (SAE J670)
├── telemetry.csv          # Tabular telemetry
├── master_video.mp4       # Camera video
├── run.log                # Execution log
└── frames/                # Raw frames
```

## Telemetry Data

Vehicle state is recorded in SAE J670 coordinate system:

- **X-axis**: Forward (positive = front of vehicle)
- **Y-axis**: Left (positive = left side of vehicle)
- **Z-axis**: Up (positive = top of vehicle)

Recorded data includes:
- Position (world coordinates)
- Velocity (vx, vy, vz in m/s)
- Acceleration (ax, ay, az in m/s^2)
- Angular velocity (roll_rate, pitch_rate, yaw_rate in deg/s)
- Control inputs (throttle, brake, steer)

See [docs/api/telemetry_schema.md](docs/api/telemetry_schema.md) for full schema.

## Scenarios

| ID | Map | Target Event | Key Actor | Trigger Time |
|----|-----|--------------|-----------|--------------|
| highway_merge | Town04 | vehicle_cut_in | merge_vehicle | ~20s |
| lane_change_cut_in | Town05 | vehicle_cut_in | cut_in_vehicle | ~40s |
| pedestrian_emerge | Town05 | avoid_pedestrian | walker | ~38s |
| unprotected_left_turn | Town03 | yield_left_turn | oncoming_vehicle | ~39s |
| red_light_conflict | Town03 | stop_for_red_light | cross_vehicle | ~35s |
| yield_to_emergency | Town04 | yield_to_emergency | emergency | ~35s |

Each scenario has detailed documentation in its README.md file.

### Scenario Implementation Notes

- **Waypoint Navigation**: All scenarios use CARLA's waypoint system for reliable actor spawning in adjacent lanes
- **Auto Steer Detection**: merge and cut-in scenarios automatically detect steering direction based on lane position
- **Manual Steering**: unprotected_left_turn uses manual steering control (not autopilot) for reliable turn execution
- **Delayed Spawn**: yield_to_emergency spawns the emergency vehicle at trigger time using `waypoint.previous()`
- **Walker Respawn**: pedestrian_emerge respawns walker at trigger time and adds to ctx.actors for telemetry tracking

## Render Presets

Use presets to balance quality vs. speed:

| Preset | FPS | Duration | Resolution | Use Case |
|--------|-----|----------|------------|----------|
| fast | 10 | 6s | 640x360 | Quick syntax check |
| fast_long | 10 | 20s | 640x360 | Medium test |
| test | 10 | 60s | 640x360 | Full scenario test (recommended) |
| final | 20 | 60s | 1280x720 | Production quality |

```bash
# Quick test (6s)
python run_scenario.py --scenario highway_merge --render-preset fast

# Full scenario test (60s, events visible)
python run_scenario.py --scenario highway_merge --render-preset test

# Production quality
python run_scenario.py --scenario highway_merge --render-preset final
```

**Note**: Trigger frame values are automatically scaled when using presets with different FPS. For example, `trigger_frame: 800` at 20fps becomes 400 at 10fps.

## Interactive 2D Scene Editor (MVP)

The interactive editor loads exported map centerlines and lets you edit multi-actor keyframes with a time slider, add events, visualize conflicts/feasibility, and export `scene_edit.json` plus replay-ready `plan.json/events_plan.json`.

```bash
# Export map assets once
python -m carla_experiment_client.planning.map_exporter --map Town05 --out data/maps

# Launch the editor
python -m carla_experiment_client.editor.interactive_editor \
  --map-dir data/maps/Town05 \
  --episode-id P1_T2_lane_change \
  --out outputs
```

Editor controls (buttons on the right panel):
- Add/Move/Del keyframes (click/drag on the map)
- Time slider (bottom) to scrub trajectories
- Add Event with `Event` + `Action` fields
- Analyze (conflict + lane feasibility overlays)
- Save (scene_edit.json) and Export (plan.json/events_plan.json)

## Unified CLI (Interactive + Subcommands)

The unified CLI provides an interactive menu and command-line subcommands for map export, planning, validation, rendering, and the editor.

```bash
# Interactive menu
python -m carla_experiment_client.cli
python cli.py

# Subcommand examples
python -m carla_experiment_client.cli map --map Town05
python -m carla_experiment_client.cli editor --map-dir data/maps/Town05 --episode-id P1_T2_lane_change
python -m carla_experiment_client.cli pipeline --episode P1_T2_lane_change --quick
```

## Generate Experiment Variants

```bash
# Generate all 6 variants (V0/V1/V2 x R0/R1)
python render_variants.py --run_dir runs/test

# Generate only audio assets
python render_variants.py --run_dir runs/test --audio-only
```

## Configuration

### Client Config (configs/client.yaml)

```yaml
host: 192.168.31.32
port: 2000
tm_port: 8000
timeout: 10.0
allow_version_mismatch: true
```

### Scenario Config

Each scenario has its own config files:
- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

Parameters include map, weather, seed, FPS, camera settings, and scenario-specific triggers.

## API Documentation

- [Output Formats](docs/api/output_formats.md) - File structure and descriptions
- [Telemetry Schema](docs/api/telemetry_schema.md) - SAE J670 data format
- [Events Schema](docs/api/events_schema.md) - Event detection format
- [Coordinate Systems](docs/api/coordinate_systems.md) - Coordinate frame reference

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Version mismatch | Set `allow_version_mismatch: true` in client.yaml |
| Connection refused | Check firewall and CARLA server status |
| Missing ffmpeg | `sudo apt-get install ffmpeg` |
| No audio | Install `pip install edge-tts` for TTS |

### Spawn Issues

Check `run.log` for spawn warnings:

```bash
# Minor issue - actor spawned with small offset
[WARNING] Spawn fallback used for merge_vehicle

# Critical issue - actor at random location (hundreds of meters away)
[WARNING] Random spawn fallback used for merge_vehicle
```

**Solutions**:
1. Use waypoint navigation instead of `offset_transform()` for adjacent lane spawning
2. Use `waypoint.previous()` for spawning behind ego vehicle
3. Add spawn distance validation after spawning

### Scenario Events Not Occurring

1. **Check trigger timing**: Verify `run.log` shows trigger messages like `"Merge maneuver started at frame X"`
2. **Analyze telemetry**: Check actor distances in `telemetry.json` - key actors should be within 30m
3. **Verify config**: Ensure `use_autopilot_turn: false` for left turn scenario

### Telemetry Missing Actors

If dynamically spawned actors (e.g., respawned walker) are not in telemetry:
- Ensure new actors are added to `ctx.actors` after respawn

## Development

### Verify Setup

```bash
# Verify imports
python -c "from carla_experiment_client.scenarios.registry import get_scenario_ids; print(get_scenario_ids())"

# Check syntax
python -m py_compile carla_experiment_client/run_scenario.py
```

### Testing Workflow

1. Run scenario with test preset:
   ```bash
   python run_scenario.py --scenario highway_merge --render-preset test --out runs/test
   ```

2. Check `run.log` for spawn warnings and trigger messages

3. Analyze telemetry for actor distances:
   ```python
   import json, math
   with open("runs/test/telemetry.json") as f:
       data = json.load(f)
   # Check key actor distances
   ```

4. Verify video shows expected behavior

### Key Files for Modification

| Purpose | File |
|---------|------|
| Scenario logic | `scenarios/<id>/scenario.py` |
| Scenario config | `scenarios/<id>/config_30s.yaml` |
| Spawn helpers | `scenarios/base.py` |
| Event detection | `events/extractor.py` |
| Telemetry recording | `telemetry/recorder.py` |

## For AI Agents

See [docs/for_ai_agents.md](docs/for_ai_agents.md) for detailed reference including:
- Spawn fallback issues and solutions
- Frame scaling with render presets
- Scenario-specific notes and config parameters
- Telemetry analysis tips
- Debugging workflow

## License

Internal research project for HKUST(GZ).
