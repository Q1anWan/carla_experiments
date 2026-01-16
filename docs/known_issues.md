# Known Issues / Notes (2026-01-16)

Context: Full-length runs executed in CARLA312 against server 192.168.31.32:2000.
Primary outputs: `runs/full_test_20260116_174612_r3` (red_light_conflict run at `runs/full_test_20260116_174612_r3/red_light_conflict_fix7`).

## Observed issues
- Event extractor emits many generic braking events (e.g., brake_hard counts are high in highway_merge, lane_change_cut_in, pedestrian_emerge). This may be too noisy for experiment stimuli.
- yield_to_emergency: emergency actor first telemetry frame can show position (0,0,0) at spawn frame; likely due to snapshot timing vs. spawn callback. Needs investigation if this affects distance-based analysis.
- pedestrian_emerge: walker relocation works, but initial spawn logs warn about being far from ego (expected with relocate); actor cleanup warnings appear after respawn.
- red_light_conflict: earlier attempts failed with actor destruction/timeouts when the world was already stressed; success after reloading Town03. If failures recur, reload the map before running.
- Client/server version mismatch warnings persist (client 3cfbfc0 vs server 2a087ff).

## Recent changes that require re-test
- Emergency detection now only accepts role_name == "emergency" (to avoid background false positives). Full-suite rerun needed to confirm event timelines.
- red_light_conflict now uses fixed spawn indices (ego 165, cross 10) and a closer light timing (red at frame 600, cross release at 640). Verify conflict visibility and event timing in video.

## Stability notes
- background_walker_count set to 0 in pedestrian_emerge, unprotected_left_turn, red_light_conflict to avoid tick stalls and destroyed-actor errors observed previously.
