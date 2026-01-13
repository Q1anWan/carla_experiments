# Yield to Emergency

Config: `configs/scenarios/yield_to_emergency_30s.yaml`
Map: Town04
Duration: ~30s, target event at ~20s

What happens:
- Ego cruises on a straight segment with surrounding traffic.
- An emergency vehicle approaches from behind with lights enabled.
- The emergency vehicle accelerates around 16s, creating a yield event near 20s.

Actors:
- ego
- emergency vehicle
- background traffic

Key tuning:
- emergency_spawn_distance_m, emergency_speed_delta
- emergency_boost_start_frame, emergency_boost_frames, emergency_throttle
- ego_speed_delta, ego_follow_distance_m
