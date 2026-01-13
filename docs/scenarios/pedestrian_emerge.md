# Pedestrian Emerge

Config: `configs/scenarios/pedestrian_emerge_30s.yaml`
Map: Town05
Duration: ~30s, target event at ~20s

What happens:
- Ego cruises with surrounding traffic and an occluding vehicle near the curb.
- A pedestrian starts crossing around 18s from behind the occluder.
- Ego is expected to brake to avoid the pedestrian.

Actors:
- ego
- pedestrian
- occluder_vehicle
- background traffic and walkers

Key tuning:
- trigger_frame, walker_start_ahead_m, walker_side_offset_m
- occluder_forward_m, occluder_side_offset_m, occluder_blueprint
- ego_speed_delta, ego_follow_distance_m
