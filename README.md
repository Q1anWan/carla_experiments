# CARLA Experiment Client

A reproducible, headless-friendly client for generating CARLA driving scenario videos and experimental stimuli for human factors research.

## Key Features

- **6 Driving Scenarios**: Highway merge, lane change cut-in, pedestrian emerge, unprotected left turn, red light conflict, yield to emergency
- **SAE J670 Telemetry**: Vehicle state recording in standard automotive coordinate system
- **Event Detection**: Automatic extraction of driving decision points
- **Experiment Variants**: Generate 6 stimulus variants (Voice x Robot conditions)

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

| ID | Map | Target Event | Description |
|----|-----|--------------|-------------|
| highway_merge | Town04 | vehicle_cut_in | Vehicle merges into ego lane |
| lane_change_cut_in | Town05 | vehicle_cut_in | Vehicle cuts in from adjacent lane |
| pedestrian_emerge | Town05 | avoid_pedestrian | Pedestrian crosses from behind parked car |
| unprotected_left_turn | Town03 | yield_left_turn | Left turn with oncoming traffic |
| red_light_conflict | Town03 | stop_for_red_light | Approach to red traffic light |
| yield_to_emergency | Town04 | yield_to_emergency | Emergency vehicle approaching from rear |

Each scenario has detailed documentation in its README.md file.

## Render Presets

Use presets to balance quality vs. speed:

```bash
# Fast testing (10fps, 20s, 640x360)
python run_scenario.py --scenario highway_merge --render-preset fast

# Longer fast test (10fps, 20s)
python run_scenario.py --scenario highway_merge --render-preset fast_long

# Full quality (20fps, 30s, 1280x720)
python run_scenario.py --scenario highway_merge --render-preset final
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

- **Version mismatch**: Set `allow_version_mismatch: true` in client.yaml
- **Connection refused**: Check firewall and CARLA server status
- **Missing ffmpeg**: `sudo apt-get install ffmpeg`
- **No audio**: Install `pip install edge-tts` for TTS

## Development

```bash
# Verify imports
python -c "from carla_experiment_client.scenarios.registry import get_scenario_ids; print(get_scenario_ids())"

# Check syntax
python -m py_compile carla_experiment_client/run_scenario.py
```

## License

Internal research project for HKUST(GZ).
