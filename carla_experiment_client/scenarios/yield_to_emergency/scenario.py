"""Yield to emergency vehicle scenario."""

from __future__ import annotations

import logging
import random

import carla

from ..base import (
    BaseScenario,
    ScenarioContext,
    _get_param_float,
    find_spawn_point,
    get_spawn_point_by_index,
    log_spawn,
    offset_transform,
    pick_spawn_point,
)


class YieldToEmergencyScenario(BaseScenario):
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

        # Emergency vehicle parameters - may spawn later
        emergency_spawn_frame = params.get("emergency_spawn_frame")
        emergency_distance = float(params.get("emergency_spawn_distance_m", -50.0))
        emergency_speed_delta = _get_param_float(params, "emergency_speed_delta")
        emergency_follow_distance = _get_param_float(params, "emergency_follow_distance_m")

        # State for delayed emergency spawn
        emergency_state = {"vehicle": None, "spawned": False}

        # Spawn nearby vehicles for traffic density
        nearby_vehicles: list[carla.Actor] = []
        nearby_offsets = params.get("nearby_vehicle_offsets") or []
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

        background_vehicle_count = int(params.get("background_vehicle_count", 20))
        background_walker_count = int(params.get("background_walker_count", 10))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        exclude_locs = [ego_spawn.location] + [v.get_location() for v in nearby_vehicles]
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=exclude_locs,
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego] + nearby_vehicles + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        boost_start = int(params.get("emergency_boost_start_frame", 0))
        boost_frames = int(params.get("emergency_boost_frames", self.config.fps * 2))
        throttle = float(params.get("emergency_throttle", 0.85))
        spawn_frame = int(emergency_spawn_frame) if emergency_spawn_frame else 0

        def spawn_and_control_emergency(frame: int) -> None:
            # Delayed spawn of emergency vehicle
            if frame == spawn_frame and not emergency_state["spawned"]:
                # Spawn emergency behind ego's CURRENT position using waypoint navigation
                ego_transform = ego.get_transform()
                ego_wp = world.get_map().get_waypoint(ego_transform.location)

                # Use waypoint.previous() to find valid road position behind ego
                distance_behind = abs(emergency_distance)  # Convert to positive
                prev_wps = ego_wp.previous(distance_behind)
                if prev_wps:
                    emergency_spawn = prev_wps[0].transform
                    emergency_spawn.location.z += 0.3  # Ensure above ground
                else:
                    # Fallback to offset_transform if waypoint nav fails
                    emergency_spawn = offset_transform(ego_transform, forward=emergency_distance)

                try:
                    emergency = self._spawn_vehicle(
                        world,
                        tm,
                        rng,
                        blueprint_filter="vehicle.ford.ambulance",
                        transform=emergency_spawn,
                        role_name="emergency",
                        autopilot=True,
                    )
                except RuntimeError:
                    emergency = self._spawn_vehicle(
                        world,
                        tm,
                        rng,
                        blueprint_filter="vehicle.*",
                        transform=emergency_spawn,
                        role_name="emergency",
                        autopilot=True,
                    )
                log_spawn(emergency, "emergency", emergency_spawn)

                # Validate emergency vehicle is within reasonable distance
                emergency_loc = emergency.get_location()
                if abs(emergency_loc.x) < 0.1 and abs(emergency_loc.y) < 0.1 and abs(emergency_loc.z) < 0.1:
                    emergency_loc = emergency_spawn.location
                actual_dist = ego_transform.location.distance(emergency_loc)
                if actual_dist > 100.0:
                    logging.warning(
                        "Emergency spawned far from ego (%.1fm vs intended %.1fm)",
                        actual_dist, distance_behind
                    )

                # Set emergency lights
                try:
                    lights = carla.VehicleLightState(
                        carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
                    )
                    emergency.set_light_state(lights)
                except (RuntimeError, TypeError, AttributeError):
                    pass

                # Configure TM for emergency
                self._configure_vehicle_tm(
                    tm,
                    emergency,
                    speed_delta=emergency_speed_delta,
                    follow_distance=emergency_follow_distance,
                )

                emergency_state["vehicle"] = emergency
                emergency_state["spawned"] = True
                ctx.tag_actor("emergency", emergency)
                ctx.actors.append(emergency)
                logging.info("Emergency vehicle spawned at frame %d", frame)

            # Boost emergency after spawn
            emergency = emergency_state["vehicle"]
            if emergency is None:
                return

            if frame == boost_start and emergency_state["spawned"]:
                emergency.set_autopilot(False)
            if boost_start <= frame < boost_start + boost_frames and emergency_state["spawned"]:
                emergency.apply_control(carla.VehicleControl(throttle=throttle))
            if frame == boost_start + boost_frames and emergency_state["spawned"]:
                emergency.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(spawn_and_control_emergency)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
