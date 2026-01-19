"""Highway merge scenario."""

from __future__ import annotations

import logging
import random

import carla

from ..base import (
    BaseScenario,
    ScenarioContext,
    find_spawn_point,
    get_spawn_point_by_index,
    log_spawn,
    offset_transform,
    pick_spawn_point,
    right_vector,
)


class HighwayMergeScenario(BaseScenario):
    def build(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
    ) -> ScenarioContext:
        params = self.config.params
        spawn_points = world.get_map().get_spawn_points()
        ego_spawn = get_spawn_point_by_index(
            spawn_points, params.get("ego_spawn_index")
        )
        if ego_spawn is None and bool(params.get("fast_spawn")):
            ego_spawn = pick_spawn_point(spawn_points, rng)
        if ego_spawn is None:
            ego_spawn = find_spawn_point(
            world,
            rng,
            min_lanes=2,
            avoid_junction=True,
            forward_clear_m=120.0,
            avoid_traffic_lights=True,
            )
        ego = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter=self.config.ego_vehicle,
            transform=ego_spawn,
            role_name="ego",
            autopilot=True,
        )
        log_spawn(ego, "ego", ego_spawn)
        self._apply_ego_tm(tm, ego)

        nearby_vehicles: list[carla.Actor] = []
        nearby_offsets = self.config.params.get("nearby_vehicle_offsets") or [
            {"forward": 12.0, "right": 3.5},
            {"forward": -8.0, "right": -3.5},
        ]
        if isinstance(nearby_offsets, list):
            for index, offset in enumerate(nearby_offsets):
                if not isinstance(offset, dict):
                    continue
                forward = float(offset.get("forward", 0.0))
                right = float(offset.get("right", 0.0))
                transform = offset_transform(ego_spawn, forward=forward, right=right)
                try:
                    vehicle = self._spawn_vehicle(
                        world,
                        tm,
                        rng,
                        blueprint_filter="vehicle.*",
                        transform=transform,
                        role_name=f"nearby_vehicle_{index}",
                        autopilot=True,
                    )
                    log_spawn(vehicle, f"nearby_vehicle_{index}", transform)
                    nearby_vehicles.append(vehicle)
                except RuntimeError:
                    logging.warning("Failed to spawn nearby vehicle %d", index)

        # Find adjacent driving lane for merge vehicle using waypoint navigation
        waypoint = world.get_map().get_waypoint(ego_spawn.location)
        merge_wp = None
        right_wp = waypoint.get_right_lane()
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            merge_wp = right_wp
        else:
            left_wp = waypoint.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                merge_wp = left_wp

        # Position merge vehicle ahead in adjacent lane for natural approach
        merge_ahead_m = float(params.get("merge_vehicle_ahead_m", 8.0))
        if merge_wp:
            # Use waypoint navigation to find valid position ahead
            next_wps = merge_wp.next(merge_ahead_m)
            if next_wps:
                merge_spawn = next_wps[0].transform
                merge_spawn.location.z += 0.3  # Ensure above ground
            else:
                merge_spawn = merge_wp.transform
                merge_spawn.location.z += 0.3
        else:
            merge_spawn = offset_transform(ego_spawn, right=3.5, forward=merge_ahead_m)

        merge_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=merge_spawn,
            role_name="merge_vehicle",
            autopilot=True,
        )
        log_spawn(merge_vehicle, "merge_vehicle", merge_spawn)
        merge_speed_delta = params.get("merge_speed_delta")
        self._configure_vehicle_tm(
            tm,
            merge_vehicle,
            speed_delta=float(merge_speed_delta) if merge_speed_delta is not None else None,
            auto_lane_change=False,
        )

        # Validate merge vehicle is within reasonable distance
        merge_loc = merge_vehicle.get_location()
        if abs(merge_loc.x) < 0.1 and abs(merge_loc.y) < 0.1 and abs(merge_loc.z) < 0.1:
            merge_loc = merge_spawn.location
        merge_dist = ego_spawn.location.distance(merge_loc)
        if merge_dist > 50.0:
            logging.warning(
                "Merge vehicle spawned far from ego (%.1fm). Scenario may not work as expected.",
                merge_dist
            )

        lead_distance = float(params.get("lead_slow_distance_m", 35.0))
        lead_speed_delta = float(params.get("lead_slow_speed_delta", 30.0))
        lead_spawn = None
        lead_wp_candidates = waypoint.next(lead_distance)
        if lead_wp_candidates:
            lead_spawn = lead_wp_candidates[0].transform
            lead_spawn.location.z += 0.3
        else:
            lead_spawn = offset_transform(ego_spawn, forward=lead_distance)
        lead_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=lead_spawn,
            role_name="lead_slow",
            autopilot=True,
        )
        tm.vehicle_percentage_speed_difference(lead_vehicle, lead_speed_delta)
        log_spawn(lead_vehicle, "lead_slow", lead_spawn)

        background_vehicle_count = int(params.get("background_vehicle_count", 18))
        background_walker_count = int(params.get("background_walker_count", 10))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=[
                ego_spawn.location,
                merge_spawn.location,
                lead_spawn.location,
                *[vehicle.get_location() for vehicle in nearby_vehicles],
            ],
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, merge_vehicle, lead_vehicle] + nearby_vehicles + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        start_frame = int(params.get("merge_trigger_frame", self.config.fps * 2))
        duration_frames = int(params.get("merge_duration_frames", self.config.fps))
        throttle = float(params.get("merge_throttle", 0.55))
        base_steer = float(params.get("merge_steer", 0.2))
        relocate_on_trigger = bool(params.get("merge_relocate_on_trigger", False))
        relocate_forward = float(params.get("merge_relocate_forward_m", 8.0))
        relocate_right = float(params.get("merge_relocate_right_m", 3.5))

        steer = -abs(base_steer)

        def merge_trigger(frame: int) -> None:
            nonlocal steer
            if frame == start_frame:
                ego_transform = ego.get_transform()
                merge_loc = None
                if relocate_on_trigger:
                    relocate_transform = offset_transform(
                        ego_transform, forward=relocate_forward, right=relocate_right
                    )
                    merge_vehicle.set_transform(relocate_transform)
                    merge_loc = relocate_transform.location
                merge_vehicle.set_autopilot(False)
                if merge_loc is None:
                    merge_loc = merge_vehicle.get_transform().location
                ego_right = right_vector(ego_transform)
                relative = merge_loc - ego_transform.location
                right_dot = (
                    relative.x * ego_right.x
                    + relative.y * ego_right.y
                    + relative.z * ego_right.z
                )
                steer = -abs(base_steer) if right_dot > 0 else abs(base_steer)
                # Log merge start with vehicle positions
                dist = merge_loc.distance(ego_transform.location)
                logging.info("Merge maneuver started at frame %d, steer=%.2f, distance=%.1fm",
                             frame, steer, dist)
            if start_frame <= frame < start_frame + duration_frames:
                merge_vehicle.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer)
                )
            if frame == start_frame + duration_frames:
                merge_vehicle.set_autopilot(True, tm.get_port())
                logging.info("Merge maneuver completed at frame %d", frame)

        ctx.tick_callbacks.append(merge_trigger)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
