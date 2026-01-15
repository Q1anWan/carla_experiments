# Output Data Formats

This document describes the data formats and file structure produced by the CARLA Experiment Client.

## Run Output Directory Structure

Each scenario run produces a self-contained output directory:

```
runs/<scenario_name>/
├── scenario.yaml          # Copy of the configuration used
├── run_metadata.json      # Execution metadata
├── events.json            # Detected driving events
├── telemetry.json         # Vehicle telemetry (SAE J1100)
├── telemetry.csv          # Vehicle telemetry (tabular format)
├── master_video.mp4       # Encoded video from camera
├── run.log                # Execution log
└── frames/                # Raw video frames (PNG)
    ├── 000000.png
    ├── 000001.png
    └── ...
```

## File Descriptions

### scenario.yaml

A copy of the scenario configuration file used for the run. This ensures reproducibility by preserving exact parameters.

### run_metadata.json

Execution context and environment information:

```json
{
  "scenario_id": "highway_merge",
  "scenario_config": "scenario.yaml",
  "map": "Town04",
  "seed": 42,
  "fps": 20,
  "duration": 30.0,
  "fixed_delta_seconds": 0.05,
  "sync_mode": true,
  "no_rendering_mode": false,
  "voice_lead_time_s": 3.0,
  "robot_precue_lead_s": 0.5,
  "min_event_time_s": 12.0,
  "host": "192.168.31.32",
  "port": 2000,
  "traffic_manager_port": 8000,
  "server_version": "0.9.15",
  "client_version": "0.9.15",
  "allow_version_mismatch": true,
  "render_preset": "fast_long"
}
```

### events.json

Detected driving events with timing information for experiment stimuli:

```json
{
  "events": [
    {
      "t": 19.0,
      "t_event": 19.0,
      "t_voice_start": 16.0,
      "t_robot_precue": 15.5,
      "type": "vehicle_cut_in",
      "decision_text": "Brake",
      "reason_text": "Vehicle cutting in ahead",
      "robot_precue_t": 15.5,
      "audio_id": "vehicle_cut_in_00"
    }
  ]
}
```

See [events_schema.md](events_schema.md) for detailed schema.

### telemetry.json

Per-frame vehicle state in SAE J1100 coordinate system:

```json
{
  "metadata": {
    "fps": 20,
    "total_frames": 600,
    "coordinate_system": "SAE_J1100",
    "coordinate_description": {
      "X": "Forward (positive = front of vehicle)",
      "Y": "Left (positive = left side of vehicle)",
      "Z": "Up (positive = top of vehicle)"
    },
    "units": {
      "position": "meters",
      "velocity": "m/s",
      "acceleration": "m/s^2",
      "angular_velocity": "deg/s",
      "angles": "degrees"
    }
  },
  "frames": [
    {
      "frame": 0,
      "t_sim": 0.0,
      "t_world": 0.0,
      "dt": 0.05,
      "ego": {
        "position": {"x": 123.45, "y": -67.89, "z": 0.5},
        "velocity": {"vx": 12.5, "vy": 0.1, "vz": 0.0},
        "acceleration": {"ax": -1.2, "ay": 0.05, "az": 0.0},
        "angular_velocity": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.02},
        "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 45.0},
        "speed": 12.5,
        "control": {"throttle": 0.5, "brake": 0.0, "steer": 0.1}
      },
      "actors": [...]
    }
  ]
}
```

See [telemetry_schema.md](telemetry_schema.md) for detailed schema.

### telemetry.csv

Tabular format of ego vehicle telemetry for easy analysis in spreadsheet software:

| Column | Description | Unit |
|--------|-------------|------|
| frame | Frame number | - |
| t_sim | Simulation time | seconds |
| t_world | World elapsed time | seconds |
| dt | Delta time | seconds |
| world_x | World X position | meters |
| world_y | World Y position | meters |
| world_z | World Z position | meters |
| vx | Longitudinal velocity | m/s |
| vy | Lateral velocity | m/s |
| vz | Vertical velocity | m/s |
| ax | Longitudinal acceleration | m/s^2 |
| ay | Lateral acceleration | m/s^2 |
| az | Vertical acceleration | m/s^2 |
| roll_rate | Roll angular velocity | deg/s |
| pitch_rate | Pitch angular velocity | deg/s |
| yaw_rate | Yaw angular velocity | deg/s |
| roll | Roll angle | degrees |
| pitch | Pitch angle | degrees |
| yaw | Yaw angle | degrees |
| speed | Total speed | m/s |
| throttle | Throttle input | 0-1 |
| brake | Brake input | 0-1 |
| steer | Steering input | -1 to 1 |

### master_video.mp4

Encoded video from the driver-view camera. Encoded at the configured FPS using H.264 codec.

### run.log

Text log of the execution process including:
- Connection status
- Scenario build progress
- Event detections
- Telemetry summary
- Any warnings or errors

### frames/

Directory containing individual PNG frames captured from the camera sensor. Frame filenames are zero-padded frame numbers (e.g., `000042.png`).

## Variant Outputs

When generating experiment variants, additional outputs may be created:

```
runs/<scenario_name>/
├── audio/
│   ├── voice_what.wav         # Decision-only narration (Voice V1)
│   └── voice_whatwhy.wav      # Decision + reason narration (Voice V2)
├── robot_timeline.csv         # Robot precue timing
└── variants/                  # Pre-encoded variants
    ├── voice0_robot0/         # Silent, no robot
    ├── voice1_robot0/         # Decision narration, no robot
    ├── voice1_robot1/         # Decision narration + robot
    ├── voice2_robot0/         # Decision+reason, no robot
    └── voice2_robot1/         # Decision+reason + robot
```
