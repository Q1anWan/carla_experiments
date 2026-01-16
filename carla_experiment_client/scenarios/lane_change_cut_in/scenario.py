"""Lane change cut-in scenario."""

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
)


class LaneChangeCutInScenario(BaseScenario):
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
            min_lanes=3,
            avoid_junction=True,
            forward_clear_m=150.0,
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
            {"forward": 10.0, "right": 3.5},
            {"forward": -6.0, "right": -3.5},
        ]
        if isinstance(nearby_offsets, list):
            for index, offset in enumerate(nearby_offsets):
                if not isinstance(offset, dict):
                    continue
                forward = float(offset.get("forward", 0.0))
                right = float(offset.get("right", 0.0))
                transform = offset_transform(ego_spawn, forward=forward, right=right)
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

        # Use waypoint navigation to find valid adjacent lane position
        cut_in_ahead_m = float(params.get("cut_in_ahead_m", 12.0))
        ego_wp = world.get_map().get_waypoint(ego_spawn.location)

        # Find adjacent driving lane using waypoint navigation
        adjacent_wp = None
        cut_in_on_right = True
        right_wp = ego_wp.get_right_lane()
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            adjacent_wp = right_wp
            cut_in_on_right = True
        else:
            left_wp = ego_wp.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                adjacent_wp = left_wp
                cut_in_on_right = False

        if adjacent_wp:
            # Navigate ahead in adjacent lane
            ahead_wps = adjacent_wp.next(cut_in_ahead_m)
            if ahead_wps:
                cut_in_spawn = ahead_wps[0].transform
                cut_in_spawn.location.z += 0.3
            else:
                cut_in_spawn = adjacent_wp.transform
                cut_in_spawn.location.z += 0.3
        else:
            cut_in_spawn = offset_transform(ego_spawn, forward=cut_in_ahead_m, right=3.5)
            cut_in_on_right = True

        cutter = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=cut_in_spawn,
            role_name="cut_in_vehicle",
            autopilot=True,
        )
        log_spawn(cutter, "cut_in_vehicle", cut_in_spawn)

        # Validate cut-in vehicle distance
        cutter_loc = cutter.get_location()
        if abs(cutter_loc.x) < 0.1 and abs(cutter_loc.y) < 0.1 and abs(cutter_loc.z) < 0.1:
            cutter_loc = cut_in_spawn.location
        cutter_dist = ego_spawn.location.distance(cutter_loc)
        if cutter_dist > 50.0:
            logging.warning(
                "Cut-in vehicle spawned far from ego (%.1fm). Scenario may not work as expected.",
                cutter_dist
            )

        lead_distance = float(params.get("lead_slow_distance_m", 25.0))
        lead_speed_delta = float(params.get("lead_slow_speed_delta", 35.0))
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

        background_vehicle_count = int(params.get("background_vehicle_count", 20))
        background_walker_count = int(params.get("background_walker_count", 12))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=[
                ego_spawn.location,
                cut_in_spawn.location,
                lead_spawn.location,
                *[vehicle.get_location() for vehicle in nearby_vehicles],
            ],
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, cutter, lead_vehicle] + nearby_vehicles + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        start_frame = int(params.get("cut_in_trigger_frame", self.config.fps * 2))
        duration_frames = int(params.get("cut_in_duration_frames", self.config.fps))
        throttle = float(params.get("cut_in_throttle", 0.55))
        base_steer = float(params.get("cut_in_steer", -0.22))
        relocate_on_trigger = bool(params.get("cut_in_relocate_on_trigger", False))
        relocate_forward = float(params.get("cut_in_relocate_forward_m", 8.0))
        relocate_right = float(params.get("cut_in_relocate_right_m", 3.5))

        # Determine steer direction: steer left (negative) if on right, right (positive) if on left
        if cut_in_on_right:
            steer = -abs(base_steer)  # Steer left to cut into ego's lane
        else:
            steer = abs(base_steer)   # Steer right to cut into ego's lane

        def cut_in(frame: int) -> None:
            if frame == start_frame:
                if relocate_on_trigger:
                    ego_transform = ego.get_transform()
                    relocate_transform = offset_transform(
                        ego_transform, forward=relocate_forward, right=relocate_right
                    )
                    cutter.set_transform(relocate_transform)
                cutter.set_autopilot(False)
                logging.info("Cut-in maneuver started at frame %d, steer=%.2f", frame, steer)
            if start_frame <= frame < start_frame + duration_frames:
                cutter.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
            if frame == start_frame + duration_frames:
                cutter.set_autopilot(True, tm.get_port())
                logging.info("Cut-in maneuver completed at frame %d", frame)

        ctx.tick_callbacks.append(cut_in)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
