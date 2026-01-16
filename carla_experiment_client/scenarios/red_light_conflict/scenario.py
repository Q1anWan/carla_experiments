"""Red light conflict scenario.

Scenario design:
- Ego approaches an intersection with a traffic light
- Initially the light is GREEN allowing ego to approach
- At a specific frame (red_light_frame), the light turns RED
- Cross traffic vehicle is released to create conflict
- This creates a realistic "running red light" conflict scenario
"""

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


class RedLightConflictScenario(BaseScenario):
    def build(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
    ) -> ScenarioContext:
        def _normalize_yaw(yaw: float) -> float:
            while yaw > 180.0:
                yaw -= 360.0
            while yaw < -180.0:
                yaw += 360.0
            return yaw

        def _pick_cross_spawn(
            light: carla.TrafficLight,
            stop_wp: carla.Waypoint,
            lights: list[carla.TrafficLight],
            radius_m: float = 35.0,
        ) -> carla.Transform | None:
            stop_loc = stop_wp.transform.location
            candidates: list[carla.Transform] = []
            for other in lights:
                if other.id == light.id:
                    continue
                try:
                    other_wps = other.get_stop_waypoints()
                except RuntimeError:
                    continue
                for wp in other_wps:
                    if wp.transform.location.distance(stop_loc) <= radius_m:
                        candidates.append(wp.transform)
                        break
            if candidates:
                return rng.choice(candidates)
            return None

        map_obj = world.get_map()

        def _pick_cross_spawn_from_spawns(
            stop_wp: carla.Waypoint,
            spawn_points: list[carla.Transform],
            radius_m: float = 45.0,
            min_yaw_diff: float = 60.0,
        ) -> carla.Transform | None:
            stop_loc = stop_wp.transform.location
            ego_yaw = stop_wp.transform.rotation.yaw
            candidates: list[carla.Transform] = []
            for sp in spawn_points:
                if sp.location.distance(stop_loc) > radius_m:
                    continue
                wp = map_obj.get_waypoint(
                    sp.location, project_to_road=True, lane_type=carla.LaneType.Driving
                )
                if wp is None:
                    continue
                if abs(wp.transform.location.z - stop_loc.z) > 2.0:
                    continue
                yaw_diff = abs(_normalize_yaw(wp.transform.rotation.yaw - ego_yaw))
                if yaw_diff < min_yaw_diff or yaw_diff > 160.0:
                    continue
                candidates.append(wp.transform)
            if candidates:
                return rng.choice(candidates)
            return None

        spawn_points = map_obj.get_spawn_points()
        params = self.config.params
        spawn_offset_m = float(params.get("spawn_offset_m", 60.0))
        ego_spawn = get_spawn_point_by_index(
            spawn_points, params.get("ego_spawn_index")
        )
        traffic_light = None
        cross_spawn = get_spawn_point_by_index(
            spawn_points, params.get("cross_spawn_index")
        )
        if cross_spawn is not None:
            cross_spawn.location.z += 0.3
        if ego_spawn is None and bool(params.get("fast_spawn")):
            ego_spawn = pick_spawn_point(spawn_points, rng)
        lights = list(world.get_actors().filter("traffic.traffic_light"))
        if ego_spawn is None:
            rng.shuffle(lights)
            for light in lights:
                try:
                    stop_wps = light.get_stop_waypoints()
                except RuntimeError:
                    continue
                if not stop_wps:
                    continue
                stop_wp = rng.choice(stop_wps)
                previous = stop_wp.previous(spawn_offset_m)
                if not previous:
                    continue
                ego_spawn = previous[0].transform
                ego_spawn.location.z += 0.3
                traffic_light = light
                cross_spawn = _pick_cross_spawn(light, stop_wp, lights)
                if cross_spawn is None:
                    cross_spawn = _pick_cross_spawn_from_spawns(stop_wp, spawn_points)
                break
        elif lights:
            ego_wp = map_obj.get_waypoint(ego_spawn.location)
            if ego_wp is not None and spawn_offset_m > 0:
                ahead_wps = ego_wp.next(spawn_offset_m)
                if ahead_wps:
                    target_loc = ahead_wps[0].transform.location
                    for light in lights:
                        try:
                            stop_wps = light.get_stop_waypoints()
                        except RuntimeError:
                            continue
                        for stop_wp in stop_wps:
                            if stop_wp.transform.location.distance(target_loc) <= 25.0:
                                traffic_light = light
                                if cross_spawn is None:
                                    cross_spawn = _pick_cross_spawn(light, stop_wp, lights)
                                    if cross_spawn is None:
                                        cross_spawn = _pick_cross_spawn_from_spawns(stop_wp, spawn_points)
                                break
                        if traffic_light is not None:
                            break
            if traffic_light is None and cross_spawn is not None:
                for light in lights:
                    try:
                        stop_wps = light.get_stop_waypoints()
                    except RuntimeError:
                        continue
                    for stop_wp in stop_wps:
                        if stop_wp.transform.location.distance(cross_spawn.location) <= 25.0:
                            traffic_light = light
                            break
                    if traffic_light is not None:
                        break
        if ego_spawn is None:
            ego_spawn = find_spawn_point(
                world,
                rng,
                min_lanes=1,
                avoid_junction=True,
                forward_clear_m=80.0,
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

        # Spawn multiple cross traffic vehicles
        cross_vehicle_count = int(params.get("cross_vehicle_count", 1))
        cross_vehicle_spacing = float(params.get("cross_vehicle_spacing_m", 6.0))
        cross_vehicles: list[carla.Actor] = []

        if cross_spawn is None:
            cross_spawn = offset_transform(ego_spawn, right=8.0, forward=8.0)
        else:
            cross_spawn.location.z += 0.3

        for i in range(cross_vehicle_count):
            # Space cross vehicles behind each other
            spacing_offset = i * cross_vehicle_spacing
            cross_transform = offset_transform(cross_spawn, forward=-spacing_offset)
            try:
                cross_vehicle = self._spawn_vehicle(
                    world,
                    tm,
                    rng,
                    blueprint_filter="vehicle.*",
                    transform=cross_transform,
                    role_name=f"cross_vehicle_{i}",
                    autopilot=False,
                )
                log_spawn(cross_vehicle, f"cross_vehicle_{i}", cross_transform)
                cross_vehicles.append(cross_vehicle)
            except RuntimeError:
                logging.warning("Failed to spawn cross vehicle %d", i)

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

        background_vehicle_count = int(params.get("background_vehicle_count", 18))
        background_walker_count = int(params.get("background_walker_count", 10))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        exclude_locs = [ego_spawn.location, cross_spawn.location]
        exclude_locs += [v.get_location() for v in cross_vehicles]
        exclude_locs += [v.get_location() for v in nearby_vehicles]
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
            actors=[ego] + cross_vehicles + nearby_vehicles + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        cross_release_frame = params.get("cross_release_frame")
        # Frame when light turns red (default: same as cross_release - 20 frames for reaction)
        red_light_frame = int(params.get("red_light_frame", 0))
        # Track if light has been set to red
        light_state = {"turned_red": False, "cross_released": False}

        def dynamic_light_control(frame: int) -> None:
            nonlocal traffic_light
            light = traffic_light
            if light is None and frame >= max(red_light_frame - int(self.config.fps * 5), 0):
                try:
                    light = ego.get_traffic_light()
                except RuntimeError as e:
                    logging.warning("Traffic light lookup failed: %s", e)
                    return
                if light is not None:
                    traffic_light = light

            # Keep ALL cross vehicles stopped until release
            release_frame = int(cross_release_frame) if cross_release_frame else 0
            if frame < release_frame and not light_state["cross_released"]:
                for cv in cross_vehicles:
                    cv.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                    )
            elif frame == release_frame and not light_state["cross_released"]:
                for cv in cross_vehicles:
                    cv.set_autopilot(True, tm.get_port())
                light_state["cross_released"] = True
                logging.info("Released %d cross vehicles at frame %d", len(cross_vehicles), frame)

            if light is None:
                return

            try:
                # Phase 1: Keep light GREEN to allow ego to approach
                if frame < red_light_frame:
                    if not light_state["turned_red"]:
                        light.set_state(carla.TrafficLightState.Green)
                        light.set_green_time(red_light_frame / self.config.fps + 5.0)

                # Phase 2: Turn RED when ego is approaching intersection
                elif frame >= red_light_frame and not light_state["turned_red"]:
                    light.set_state(carla.TrafficLightState.Red)
                    light.set_red_time(self.config.duration + 5.0)
                    light_state["turned_red"] = True
                    logging.info("Traffic light turned RED at frame %d", frame)

            except RuntimeError as e:
                logging.warning("Traffic light control failed: %s", e)

        ctx.tick_callbacks.append(dynamic_light_control)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
