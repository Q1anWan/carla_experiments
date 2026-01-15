# Red Light Conflict Scenario

**Scenario ID:** `red_light_conflict`
**Map:** Town03
**Duration:** 30 seconds
**Target Event:** `stop_for_red_light` at ~18s

## Overview

This scenario simulates an intersection approach where the ego vehicle must stop for a red traffic light. A cross vehicle may also be present, creating a potential conflict situation. The scenario tests traffic signal compliance behavior.

## Map Selection Rationale

**Town03** was selected for this scenario because:
- Multiple signalized intersections with traffic lights
- Clear lane markings and stop lines
- Four-way intersections with cross traffic
- Urban environment with realistic signal timing

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 8 | Traffic light search, spawn 160m before stop line |

### Dynamic Spawn Point Search
If the default index is unavailable, the scenario:
1. Finds all traffic lights and their stop waypoints
2. Selects a random stop waypoint
3. Spawns ego vehicle `spawn_offset_m` (160m) before the stop line
4. Configures opposing traffic light for cross vehicle

## NPC Elements

### Key Actors

1. **Cross Vehicle**
   - Spawned at opposing traffic light's stop waypoint
   - Fallback: 8m right, 8m forward of ego
   - Initially stationary with hand brake
   - Released at `cross_release_frame` to enter intersection

2. **Background Traffic**
   - 24 vehicles, 10 walkers
   - Minimum 25m from key actors

### Traffic Light Control

The scenario manipulates traffic lights:
- **Ego's light:** Forced to RED for entire scenario
- **Cross light:** Set to GREEN to allow cross vehicle movement

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| ~10.0 | ~200 | Ego approaching intersection |
| 18.0 | 360 | Cross vehicle released |
| ~19.0 | ~380 | Ego should stop at red light |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Cross Vehicle Release (`cross_release_frame: 360`)
- Hand brake released at frame 360
- Cross vehicle enters intersection
- Creates potential conflict with ego

### Ego Brake Event (`ego_brake_frame: 400`)
- Forces ego to brake hard at frame 400
- Duration: 20 frames
- Brake value: 1.0 (full brake)

## Expected Event Detection

The event extractor should detect:
- `stop_for_red_light` when ego approaches red light with braking
- `brake_hard` when ego decelerates sharply
- `slow_down` during approach phase

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spawn_offset_m` | 160.0 | Distance before stop line |
| `cross_release_frame` | 360 | Frame when cross vehicle moves |
| `ego_speed_delta` | 40.0 | Speed reduction percentage |
| `background_vehicle_count` | 24 | Number of background vehicles |
| `background_walker_count` | 10 | Number of pedestrians |
