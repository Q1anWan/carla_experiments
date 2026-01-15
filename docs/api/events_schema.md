# Events Data Schema

This document describes the event detection output format used for human factors experiments.

## Overview

The event extractor detects driving decision points during scenario execution. Each event includes:
- Timestamp information for synchronizing with experiment stimuli
- Event type classification
- Human-readable decision and reason texts
- Audio asset identifiers

## JSON Schema

### Root Object

```json
{
  "events": [
    { ... },
    { ... }
  ]
}
```

### Event Object

```json
{
  "t": 19.0,                           // Event detection time (seconds)
  "t_event": 19.0,                     // Same as t (for compatibility)
  "t_voice_start": 16.0,               // When voice narration should start
  "t_robot_precue": 15.5,              // When robot precue should activate
  "type": "vehicle_cut_in",            // Event type identifier
  "decision_text": "Brake",            // Short decision description
  "reason_text": "Vehicle cutting in", // Reason explanation
  "robot_precue_t": 15.5,              // Duplicate of t_robot_precue
  "audio_id": "vehicle_cut_in_00"      // Audio asset identifier
}
```

### Time Fields

| Field | Description | Formula |
|-------|-------------|---------|
| `t` | Event detection time | Event occurrence time |
| `t_event` | Event time (alias) | Same as `t` |
| `t_voice_start` | Voice narration start | `t - voice_lead_time_s` |
| `t_robot_precue` | Robot precue activation | `t - robot_precue_lead_s` |

### Event Types

| Type | Detection Trigger | Decision Text | Reason Text |
|------|-------------------|---------------|-------------|
| `lane_change_left` | Steering < -0.2 + lane ID change | "Change lane left" | "Need to adjust position" |
| `lane_change_right` | Steering > 0.2 + lane ID change | "Change lane right" | "Need to adjust position" |
| `brake_hard` | Acceleration < -3.5 m/s^2 | "Brake hard" | "Obstacle or conflict ahead" |
| `slow_down` | -3.5 < Acceleration < -1.5 m/s^2 | "Slow down" | "Traffic condition requires caution" |
| `stop_for_red_light` | Red traffic light + braking | "Stop" | "Red light ahead" |
| `yield_to_emergency` | Emergency vehicle within 50m | "Yield" | "Emergency vehicle approaching" |
| `avoid_pedestrian` | Walker within 20m + braking | "Brake" | "Pedestrian crossing ahead" |
| `vehicle_cut_in` | Vehicle crosses into ego's lane | "Brake" | "Vehicle cutting in ahead" |
| `yield_left_turn` | Left turn at junction + braking | "Yield" | "Oncoming traffic at intersection" |

## Event Timing Configuration

Events are filtered and timed using scenario configuration:

```yaml
events:
  min_event_time_s: 8.0      # Minimum time before first event
  voice_lead_time_s: 3.0     # Voice starts before event
  robot_precue_lead_s: 0.5   # Robot precue before voice
```

### Timing Diagram

```
Time:  |---------|---------|---------|
       ^         ^         ^         ^
    Robot     Voice    Decision    Event
    Precue    Start      Time      Time
   (t-3.5s)  (t-3.0s)   (t-0s)    (t)
```

## Event Detection Logic

### Vehicle Cut-In Detection

A vehicle cut-in is detected when:
1. A vehicle changes from an adjacent lane to ego's lane, OR
2. A vehicle appears close (<25m) in ego's lane without prior tracking

```python
# Lane ID change detection
if prev_lane_id != current_lane_id and current_lane_id == ego_lane_id:
    emit("vehicle_cut_in")

# Proximity-based detection (relocated vehicles)
if actor_id not in prev_distances and distance < 25.0:
    emit("vehicle_cut_in")
```

### Pedestrian Avoidance Detection

Triggered when:
1. A pedestrian is within 20m in front of ego
2. Ego is braking (brake > 0.1) or decelerating (accel < -0.5)

### Left Turn Yield Detection

Triggered when:
1. Ego is at or approaching a junction
2. Steering left (steer < -0.10)
3. Braking (brake > 0.05) or slow speed (< 8 m/s)

## Duplicate Filtering

Events are deduplicated using a cooldown period:
- Same event type within 2.0 seconds is suppressed
- Events before `min_event_time_s` are filtered out

## Usage in Experiments

### Voice Narration (V1/V2)

```python
for event in events:
    if voice_mode == "V1":  # Decision only
        text = event["decision_text"]
    elif voice_mode == "V2":  # Decision + reason
        text = f"{event['decision_text']}. {event['reason_text']}"
    play_audio_at(event["t_voice_start"], text)
```

### Robot Precue (R0/R1)

```python
for event in events:
    if robot_mode == "R1":
        activate_robot_at(event["t_robot_precue"])
```

## Example Output

```json
{
  "events": [
    {
      "t": 5.4,
      "t_event": 5.4,
      "t_voice_start": 2.4,
      "t_robot_precue": 1.9,
      "type": "brake_hard",
      "decision_text": "Brake hard",
      "reason_text": "Obstacle or conflict ahead",
      "robot_precue_t": 1.9,
      "audio_id": "brake_hard_00"
    },
    {
      "t": 19.0,
      "t_event": 19.0,
      "t_voice_start": 16.0,
      "t_robot_precue": 15.5,
      "type": "vehicle_cut_in",
      "decision_text": "Brake",
      "reason_text": "Vehicle cutting in ahead",
      "robot_precue_t": 15.5,
      "audio_id": "vehicle_cut_in_01"
    }
  ]
}
```
