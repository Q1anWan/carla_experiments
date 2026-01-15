# Unprotected Left Turn Scenario

**Scenario ID:** `unprotected_left_turn`
**Map:** Town03
**Duration:** 30 seconds
**Target Event:** `yield_left_turn` at ~12s

## Overview

This scenario simulates an unprotected left turn at an intersection where the ego vehicle must yield to oncoming traffic. The scenario tests the driver's gap acceptance judgment and yielding behavior.

## Map Selection Rationale

**Town03** was selected for this scenario because:
- Multiple 4-way intersections suitable for left turn scenarios
- Clear sight lines to oncoming traffic
- Varied intersection geometries (T-junctions and cross intersections)
- Traffic light controlled and uncontrolled intersections available

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 3 | Junction search within 90m, left turn waypoint selection |

### Dynamic Spawn Point Search
If the default index is unavailable, the scenario:
1. Samples up to 40 random spawn points
2. Finds junctions ahead within 90m
3. Analyzes junction waypoints for left turn opportunities
4. Selects spawn point with valid left turn path

## NPC Elements

### Key Actors

1. **Oncoming Vehicle**
   - Spawned on opposing lane approximately 30m ahead
   - Found by searching forward for opposing direction lanes
   - Autopilot enabled for realistic approach behavior

2. **Background Traffic**
   - 20 vehicles, 6 walkers
   - Minimum 25m from key actors

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| 12.0 | 240 | Ego begins approach to intersection |
| 16.0 | 320 | Ego starts left turn maneuver |
| 20.0 | 400 | Left turn complete or yielding |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Approach Phase (`approach_frames: 240`)
- Ego follows autopilot until frame 240
- Speed controlled by `ego_throttle: 0.45`

### Turn Phase (`turn_frames: 80`)
- At approach_frames, ego switches to manual control
- Executes left turn with `turn_steer: -0.25`
- Throttle reduced to `turn_throttle: 0.30`
- `turn_steer_auto: true` adjusts direction based on junction analysis

### Auto Turn Direction
- Analyzes junction waypoints to determine optimal turn direction
- Selects left-turning path (yaw difference < -15 degrees)
- Adjusts steering sign accordingly

## Expected Event Detection

The event extractor should detect:
- `yield_left_turn` when turning left at junction with braking/slow speed
- `brake_hard` if emergency braking is required
- `slow_down` during approach and turn phases

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `approach_frames` | 240 | Autopilot approach duration |
| `turn_frames` | 80 | Manual turn duration |
| `ego_throttle` | 0.45 | Approach throttle |
| `turn_throttle` | 0.30 | Turn throttle |
| `turn_steer` | -0.25 | Turn steering angle |
| `turn_steer_auto` | true | Auto-adjust turn direction |
| `background_vehicle_count` | 20 | Number of background vehicles |
