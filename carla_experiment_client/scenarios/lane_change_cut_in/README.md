# Lane Change Cut-In Scenario

**Scenario ID:** `lane_change_cut_in`
**Map:** Town05
**Duration:** 30 seconds
**Target Event:** `vehicle_cut_in` at ~19s

## Overview

This scenario simulates an urban multi-lane driving situation where another vehicle abruptly cuts into the ego vehicle's lane. The scenario tests the driver's ability to respond to sudden lateral intrusions requiring emergency braking.

## Map Selection Rationale

**Town05** was selected for this scenario because:
- Multiple 3+ lane urban roads suitable for cut-in maneuvers
- Straight road segments allowing clear visibility of lane changes
- Dense urban environment with realistic traffic density
- Good mix of commercial and residential areas for varied background

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 5 | `min_lanes=3`, `forward_clear_m=150`, `avoid_junction=True` |

## NPC Elements

### Key Actors

1. **Cut-In Vehicle** (`cut_in_vehicle`)
   - Spawns on right adjacent lane
   - At trigger frame, relocates to 22m ahead and 5.5m right of ego
   - Executes aggressive lane change with `steer=-0.32` for 60 frames

2. **Lead Slow Vehicle** (`lead_slow`)
   - Spawns 30m ahead of ego
   - Speed reduced by 35% below speed limit
   - Creates following distance pressure

3. **Nearby Vehicles** (2 default)
   - Position 1: +10m forward, +3.5m right
   - Position 2: -6m backward, -3.5m left
   - Establish traffic context

4. **Background Traffic**
   - 26 vehicles, 8 walkers
   - Minimum 25m from key actors

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| 19.0 | 380 | Cut-in vehicle relocated and begins maneuver |
| 22.0 | 440 | Cut-in maneuver complete |
| 20.0 | 400 | Optional ego brake trigger |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Cut-In Trigger (`cut_in_trigger_frame: 380`)
- Cut-in vehicle teleports relative to current ego position
- Forward offset: 22.0m
- Right offset: 5.5m (one lane to the right)
- Autopilot disabled, aggressive steering begins

### Cut-In Maneuver Parameters
- `cut_in_duration_frames: 60` (3 seconds at 20fps)
- `cut_in_throttle: 0.65`
- `cut_in_steer: -0.32` (sharp left turn into ego lane)

### Ego Brake Event (`ego_brake_frame: 400`)
- Forces ego to brake hard at frame 400
- Duration: 20 frames
- Brake value: 1.0 (full brake)
- Resumes autopilot after completion

## Expected Event Detection

The event extractor should detect:
- `vehicle_cut_in` when cut-in vehicle crosses into ego's lane
- `brake_hard` when ego decelerates sharply
- `slow_down` during gradual deceleration phases

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cut_in_trigger_frame` | 380 | Frame when cut-in begins |
| `cut_in_duration_frames` | 60 | Duration of cut-in maneuver |
| `cut_in_relocate_forward_m` | 22.0 | Forward offset for relocation |
| `cut_in_relocate_right_m` | 5.5 | Lateral offset for relocation |
| `cut_in_steer` | -0.32 | Steering angle during maneuver |
| `lead_slow_distance_m` | 30.0 | Distance to lead vehicle |
| `background_vehicle_count` | 26 | Number of background vehicles |
