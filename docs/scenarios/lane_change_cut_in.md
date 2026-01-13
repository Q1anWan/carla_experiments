# Lane Change Cut-In

Config: `configs/scenarios/lane_change_cut_in_30s.yaml`
Map: Town05
Duration: ~30s, target event at ~20s

What happens:
- Ego cruises on a straight multi-lane segment with visible traffic.
- A cut-in vehicle moves into the ego lane around 18s.
- A slow lead vehicle ahead increases pressure for braking or lane change.

Actors:
- ego
- cut_in_vehicle
- lead_slow
- nearby vehicles and background traffic

Key tuning:
- cut_in_trigger_frame, cut_in_duration_frames
- lead_slow_distance_m, lead_slow_speed_delta
- ego_speed_delta, ego_follow_distance_m
- background_vehicle_count, nearby_vehicle_offsets

Recording notes:
- All actors spawn before recording; runtime relocation is disabled.
- `prewarm_seconds` ticks the world before frame 0 to avoid spawn drop.
