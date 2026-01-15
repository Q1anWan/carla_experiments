# Yield to Emergency Vehicle Scenario

**Scenario ID:** `yield_to_emergency`
**Map:** Town04
**Duration:** 30 seconds
**Target Event:** `yield_to_emergency` at ~18s

## Overview

This scenario simulates an emergency vehicle (ambulance) approaching from behind with lights and sirens active. The ego vehicle must yield by pulling over or slowing down. The scenario tests emergency vehicle awareness and yielding behavior.

## Map Selection Rationale

**Town04** was selected for this scenario because:
- Wide highway-style roads with shoulders for yielding
- Multiple lanes allowing vehicles to pull over
- Long straight sections for emergency vehicle approach
- Good visibility for rearward detection

## Spawn Point Configuration

| Actor | Default Index | Fallback Criteria |
|-------|---------------|-------------------|
| Ego Vehicle | 12 | `min_lanes=2`, `forward_clear_m=120`, `avoid_junction=True` |

## NPC Elements

### Key Actors

1. **Emergency Vehicle** (`emergency_vehicle`)
   - Blueprint: `vehicle.ford.ambulance` (with fallback to generic)
   - Initial position: 45m BEHIND ego
   - Speed delta: -30% (faster than ego initially)
   - At boost frame: Accelerates to catch up with ego
   - Light signals enabled (if supported by vehicle model)

2. **Background Traffic**
   - 22 vehicles, 8 walkers
   - Minimum 25m from key actors

### Actor Timeline

| Time (s) | Frame | Event |
|----------|-------|-------|
| 0.0 | 0 | Recording starts, all actors spawned |
| 1.0 | 20 | Prewarm complete |
| ~12.0 | ~240 | Emergency vehicle begins normal approach |
| 16.0 | 320 | Emergency vehicle boost begins |
| 20.0 | 400 | Emergency vehicle at close range |
| 30.0 | 600 | Recording ends |

## Trigger Conditions

### Emergency Boost Phase
- **Start:** `emergency_boost_start_frame: 320`
- **Duration:** `emergency_boost_frames: 80` (4 seconds at 20fps)
- **Throttle:** `emergency_throttle: 0.9` (aggressive acceleration)
- Effect: Emergency vehicle rapidly closes distance to ego

### Ego Brake Event (`ego_brake_frame: 400`)
- Forces ego to brake hard at frame 400
- Duration: 20 frames
- Brake value: 1.0 (full brake)
- Simulates yielding behavior

## Expected Event Detection

The event extractor should detect:
- `yield_to_emergency` when emergency vehicle approaches from rear
- `brake_hard` when ego yields by braking
- `slow_down` during deceleration phases
- `lane_change_right` if ego pulls over to shoulder

## Configuration Files

- `config.yaml` - Short test version (~18s)
- `config_30s.yaml` - Full experiment version (30s)

## Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `emergency_spawn_distance_m` | 45.0 | Initial distance behind ego |
| `emergency_speed_delta` | -30.0 | Speed difference (negative = faster) |
| `emergency_boost_start_frame` | 320 | Frame when boost begins |
| `emergency_boost_frames` | 80 | Duration of boost phase |
| `emergency_throttle` | 0.9 | Throttle during boost |
| `background_vehicle_count` | 22 | Number of background vehicles |
