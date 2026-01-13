# CARLA Remote Experiment Client

A reproducible, headless-friendly client project to generate CARLA scenario master videos and derive 6 experimental stimulus variants.

## Project layout

```
carla_experiment_client/
  carla_experiment_client/
    audio/
    events/
    sensors/
    scenarios/
    carla_client.py
    config.py
    debug_tools.py
    render_variants.py
    run_scenario.py
    utils.py
    video.py
  configs/
    scenarios/
    client.yaml
  docs/
  run_scenario.py
  render_variants.py
  debug_tools.py
```

## Module responsibilities

- `carla_experiment_client/carla_client.py`: Connect to CARLA, check versions, enable synchronous mode, configure Traffic Manager, and a minimal demo.
- `carla_experiment_client/sensors/camera_recorder.py`: Attach RGB camera and save frames per tick in sync mode.
- `carla_experiment_client/scenarios/*`: Six scenario classes with reproducible spawning and trigger callbacks.
- `carla_experiment_client/events/extractor.py`: Per-tick decision event extraction and `events.json` writing.
- `carla_experiment_client/audio/tts.py`: Text-to-speech with a stub fallback (tone) when edge-tts is not installed.
- `carla_experiment_client/audio/mux.py`: Build narration timeline and mux with video via ffmpeg.
- `carla_experiment_client/run_scenario.py`: CLI pipeline for master run (frames -> mp4, events, metadata).
- `carla_experiment_client/render_variants.py`: CLI pipeline to generate 6 variants (voice x robot).
- `carla_experiment_client/debug_tools.py`: Step-by-step debug CLI for connection, sync, camera, scenario, events, and audio.

## Requirements

- Python 3.12
- CARLA Python API available in the environment
- `ffmpeg` (required for mp4 encoding and audio muxing)
- Optional: `edge-tts` for real TTS (fallback is a short tone)

## Start CARLA server (Windows 11)

Example (PowerShell):

```
.\CarlaUE4.exe -RenderOffScreen -world-port=2000 -quality-level=Epic
```

Make sure the server is reachable from Ubuntu (`192.168.31.32:2000`).

## Run a scenario (Ubuntu client)

From `PythonAPI/examples`:

```
conda activate CARLA312
cd carla_experiment_client
./run_scenario.py --scenario configs/scenarios/highway_merge.yaml --out runs/20250101_120000
```

Outputs:

```
runs/<timestamp>/
  frames/
  master_video.mp4
  events.json
  run_metadata.json
```

## Render 6 stimulus variants

```
./render_variants.py --run_dir runs/20250101_120000
```

Outputs:

```
runs/<timestamp>/variants/voice{0|1|2}_robot{0|1}/
  stimulus.mp4
  events.json
  narration.wav   # only for voice 1/2
  robot_timeline.csv   # only for robot=1
```

To avoid duplicating video files, generate only audio assets and a robot timeline:

```
./render_variants.py --run_dir runs/20250101_120000 --audio-only
```

Outputs:

```
runs/<timestamp>/
  audio_assets/
    what.wav
    whatwhy.wav
  robot_timeline.csv
```

## Debugging modules (step-by-step)

```
./debug_tools.py check_connection --client-config configs/client.yaml
./debug_tools.py tick_sync --frames 50 --fixed-delta 0.05
./debug_tools.py camera --out debug/camera --frames 30 --encode
./debug_tools.py scenario --scenario configs/scenarios/highway_merge.yaml --frames 60
./debug_tools.py events --scenario configs/scenarios/highway_merge.yaml --frames 120 --out debug/events.json
./debug_tools.py audio --events debug/events.json --voice-level 2 --out debug/narration.wav
./debug_tools.py variants --run-dir runs/20250101_120000
```

Use these commands to isolate issues by module (connection → sync tick → sensor → scenario → events → audio → variants).

## Client config

Default client settings live in `configs/client.yaml`:

```
host: 192.168.31.32
port: 2000
tm_port: 8000
timeout: 10.0
allow_version_mismatch: true
```

You can override from CLI when needed:

```
./run_scenario.py --scenario configs/scenarios/highway_merge.yaml --host 127.0.0.1
./debug_tools.py check_connection --client-config configs/client.yaml --no-allow-version-mismatch
```

## Scenario configs

Sample configs live in `configs/scenarios/`:

- `highway_merge`
- `lane_change_cut_in`
- `unprotected_left_turn`
- `red_light_conflict`
- `yield_to_emergency`
- `pedestrian_emerge`

Each YAML supports:

- `map`, `seed`, `duration`, `fps`, `fixed_delta_seconds`, `sync_mode`, `no_rendering_mode`
- `weather` preset (e.g., `ClearNoon`, `CloudyNoon`, `WetNoon`)
- `camera` settings (`width`, `height`, `fov`, `fps`, `preset`)
- `scenario` parameters (trigger frames, steer values, etc.)

If a `spawn_index` is invalid, the scenario will fall back to a random spawn point.

## events.json schema

See `docs/events_schema.json` for the full schema.

Example event:

```
{
  "t": 5.2,
  "type": "brake_hard",
  "decision_text": "Brake hard",
  "reason_text": "Obstacle or conflict ahead",
  "robot_precue_t": 4.7
}
```

## Robot timeline

`robot_timeline.csv` includes two actions per event:

- `blink`: robot light flash
- `wiggle`: robot body wiggle

Columns: `t_start,t_end,action,intensity,notes`

Feed this timeline to your robot controller to trigger cues aligned to `robot_precue_t`.

## Troubleshooting

- Version mismatch: client and server must match exactly (CARLA x.y.z).
- If you must ignore mismatches (e.g., Windows/Linux build hash), set `allow_version_mismatch: true` in `configs/client.yaml`.
- Port unreachable: confirm Windows firewall and IP connectivity.
- Missing ffmpeg: install with `sudo apt-get install ffmpeg`.
- TTS: install `edge-tts` for real speech, otherwise a short tone is generated.
