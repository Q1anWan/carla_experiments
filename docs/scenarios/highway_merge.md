# Highway Merge

Config: `configs/scenarios/highway_merge_30s.yaml`
Map: Town04
Duration: ~30s, target event at ~20s

What happens:
- Ego cruises on a multi-lane highway with nearby traffic present.
- A merge vehicle cuts into the ego lane around 18s.
- A slow lead vehicle reduces headway to provoke braking.

Actors:
- ego
- merge_vehicle
- lead_slow
- nearby vehicles and background traffic

Key tuning:
- merge_trigger_frame, merge_duration_frames
- lead_slow_distance_m, lead_slow_speed_delta
- ego_speed_delta, ego_follow_distance_m
- background_vehicle_count, nearby_vehicle_offsets
