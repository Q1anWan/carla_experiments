# Red Light Conflict

Config: `configs/scenarios/red_light_conflict_30s.yaml`
Map: Town03
Duration: ~30s, target event at ~20s

What happens:
- Ego approaches a signalized intersection with the ego light forced to red.
- Cross traffic is released around 18s on green.
- Ego is expected to stop or brake hard at the conflict point.

Actors:
- ego
- cross_vehicle
- background traffic

Key tuning:
- spawn_offset_m, cross_release_frame
- ego_speed_delta, ego_follow_distance_m
- background_vehicle_count

Recording notes:
- All actors spawn before recording; runtime relocation is disabled.
- `prewarm_seconds` ticks the world before frame 0 to avoid spawn drop.
