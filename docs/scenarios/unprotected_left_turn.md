# Unprotected Left Turn

Config: `configs/scenarios/unprotected_left_turn_30s.yaml`
Map: Town03
Duration: ~30s, target event at ~20s

What happens:
- Ego approaches an unsignalized junction on a straight segment.
- Oncoming traffic is present in the opposing lane.
- Ego begins a manual left turn near 18s to create a conflict.

Actors:
- ego
- oncoming_vehicle
- background traffic

Key tuning:
- approach_frames, turn_frames, turn_steer, turn_steer_auto
- ego_throttle, turn_throttle
- ego_speed_delta, ego_follow_distance_m
- background_vehicle_count

Recording notes:
- All actors spawn before recording; runtime relocation is disabled.
- `prewarm_seconds` ticks the world before frame 0 to avoid spawn drop.
