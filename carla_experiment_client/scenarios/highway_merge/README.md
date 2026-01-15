# Highway Merge Scenario

**Scenario ID:** `highway_merge`
**Map:** Town04
**Duration:** 30 seconds
**Target Event:** `vehicle_cut_in` at ~19s

## Overview

This scenario simulates a highway driving situation where the ego vehicle must respond to a vehicle merging into its lane from an adjacent lane. The scenario tests the driver's awareness of lateral traffic movements and braking response.

## Map Selection Rationale

**Town04** was selected for this scenario because:
- Wide multi-lane highway segments suitable for high-speed driving
- Long straight sections allowing vehicles to reach highway speeds
- Multiple merge opportunities with adjacent lanes
- Minimal intersection complexity to focus on lane merge behavior

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 10 | `min_lanes=2`, `forward_clear_m=120`, `avoid_junction=True` |

## NPC Elements

### Key Actors

1. **Merge Vehicle** (`merge_vehicle`)
   - Spawns on adjacent lane (right or left of ego)
   - At trigger frame, relocates to 25m ahead and 2.8m right of ego
   - Executes lane change maneuver with `steer=-0.50` for 120 frames

2. **Lead Slow Vehicle** (`lead_slow`)
   - Spawns 30m ahead of ego
   - Speed reduced by 40% below speed limit
   - Creates braking incentive for ego

3. **Nearby Vehicles** (2 default)
   - Position 1: +12m forward, +3.5m right
   - Position 2: -8m backward, -3.5m left
   - Provide traffic context

4. **Background Traffic**
   - 16 vehicles, 0 walkers
   - Minimum 25m from key actors

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| 19.0 | 380 | Merge vehicle relocated and begins lane change |
| 25.0 | 500 | Merge maneuver complete |
| 20.0 | 400 | Optional ego brake trigger |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Merge Trigger (`merge_trigger_frame: 380`)
- Merge vehicle teleports relative to current ego position
- Forward offset: 25.0m
- Right offset: 2.8m (places vehicle at lane boundary)
- Autopilot disabled, manual steering begins

### Merge Maneuver Parameters
- `merge_duration_frames: 120` (6 seconds at 20fps)
- `merge_throttle: 0.65`
- `merge_steer: 0.50` (direction auto-adjusted based on position)

### Ego Brake Event (`ego_brake_frame: 400`)
- Forces ego to brake hard at frame 400
- Duration: 20 frames
- Brake value: 1.0 (full brake)
- Resumes autopilot after completion

## Expected Event Detection

The event extractor should detect:
- `vehicle_cut_in` when merge vehicle crosses lane boundary
- `brake_hard` when ego decelerates sharply
- `slow_down` during gradual deceleration phases

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `merge_trigger_frame` | 380 | Frame when merge begins |
| `merge_duration_frames` | 120 | Duration of merge maneuver |
| `merge_relocate_forward_m` | 25.0 | Forward offset for relocation |
| `merge_relocate_right_m` | 2.8 | Lateral offset for relocation |
| `lead_slow_distance_m` | 30.0 | Distance to lead vehicle |
| `lead_slow_speed_delta` | 40.0 | Speed reduction percentage |
| `background_vehicle_count` | 16 | Number of background vehicles |
