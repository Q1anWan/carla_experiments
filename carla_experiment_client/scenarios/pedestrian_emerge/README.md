# Pedestrian Emerge Scenario

**Scenario ID:** `pedestrian_emerge`
**Map:** Town05
**Duration:** 30 seconds
**Target Event:** `avoid_pedestrian` at ~12s

## Overview

This scenario simulates a "ghost pedestrian" situation where a pedestrian suddenly emerges from behind an occluding vehicle and crosses the road in front of the ego vehicle. The scenario tests the driver's reaction to unexpected pedestrian appearances.

## Map Selection Rationale

**Town05** was selected for this scenario because:
- Urban streets with sidewalks and pedestrian areas
- Parked vehicles along roadsides suitable for occlusion
- Moderate speed limits appropriate for pedestrian scenarios
- Realistic urban environment with crosswalks

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 5 | `min_lanes=2`, `forward_clear_m=120`, `avoid_junction=True` |

## NPC Elements

### Key Actors

1. **Pedestrian (Walker)**
   - Initial position: 20m ahead, 4m to the right of ego
   - At trigger: Relocates relative to ego and begins crossing
   - Walking speed: 1.8 m/s
   - Cross offset: -8m (crosses from right to left)

2. **Occluder Vehicle** (`occluder`)
   - Blueprint: `vehicle.volkswagen.t2` (large van)
   - Position: 18m forward, 3.5m right
   - Stationary with hand brake engaged
   - Blocks line of sight to pedestrian until ego approaches

3. **Background Traffic**
   - 20 vehicles, 12 walkers
   - Minimum 25m from key actors

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| 12.0 | 240 | Pedestrian relocates and begins crossing |
| ~14.0 | ~280 | Pedestrian enters ego's path |
| 20.0 | 400 | Optional ego brake trigger |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Pedestrian Trigger (`trigger_frame: 240`)
- When `relocate_on_trigger: true`, pedestrian relocates relative to ego
- Starts walking across the road at `walker_speed: 1.8` m/s
- Cross offset determines crossing direction and distance

### Trigger Distance Mode
- Alternative trigger: `trigger_distance: 18.0`
- Activates when ego is within 18m of pedestrian's initial position

### Ego Brake Event (`ego_brake_frame: 400`)
- Forces ego to brake hard at frame 400
- Duration: 20 frames
- Brake value: 1.0 (full brake)

## Expected Event Detection

The event extractor should detect:
- `avoid_pedestrian` when pedestrian enters ego's forward path
- `brake_hard` when ego decelerates sharply
- `slow_down` during gradual deceleration phases

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `trigger_frame` | 240 | Frame when pedestrian starts crossing |
| `trigger_distance` | 18.0 | Distance-based trigger (alternative) |
| `walker_start_ahead_m` | 20.0 | Initial forward position |
| `walker_side_offset_m` | 4.0 | Initial lateral position |
| `cross_offset` | -8.0 | Crossing distance (negative = left) |
| `walker_speed` | 1.8 | Walking speed (m/s) |
| `occluder_forward_m` | 18.0 | Occluder vehicle position |
