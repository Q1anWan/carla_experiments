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
        spawn_points = world.get_map().get_spawn_points()
        params = self.config.params
        spawn_offset_m = float(params.get("spawn_offset_m", 60.0))
        ego_spawn = get_spawn_point_by_index(
            spawn_points, params.get("ego_spawn_index")
        )
        traffic_light = None
        cross_spawn = None
        if ego_spawn is None and bool(params.get("fast_spawn")):
            ego_spawn = pick_spawn_point(spawn_points, rng)
        if ego_spawn is None:
            lights = list(world.get_actors().filter("traffic.traffic_light"))
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
                traffic_light = light
                group = [item for item in light.get_group() if item.id != light.id]
                rng.shuffle(group)
                for other in group:
                    try:
                        other_wps = other.get_stop_waypoints()
                    except RuntimeError:
                        continue
                    if not other_wps:
                        continue
                    cross_spawn = rng.choice(other_wps).transform
                    break
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
        log_spawn(ego, "ego")
        self._apply_ego_tm(tm, ego)

        # Spawn multiple cross traffic vehicles
        cross_vehicle_count = int(params.get("cross_vehicle_count", 1))
        cross_vehicle_spacing = float(params.get("cross_vehicle_spacing_m", 6.0))
        cross_vehicles: list[carla.Actor] = []

        if cross_spawn is None:
            cross_spawn = offset_transform(ego_spawn, right=8.0, forward=8.0)

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
                log_spawn(cross_vehicle, f"cross_vehicle_{i}")
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
                    log_spawn(vehicle, f"nearby_vehicle_{index}")
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
            light = traffic_light or ego.get_traffic_light()

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
                        # Keep cross traffic red
                        try:
                            group = light.get_group()
                        except AttributeError:
                            group = []
                        for other in group:
                            if other.id != light.id:
                                other.set_state(carla.TrafficLightState.Red)
                                other.set_red_time(red_light_frame / self.config.fps + 5.0)

                # Phase 2: Turn RED when ego is approaching intersection
                elif frame >= red_light_frame and not light_state["turned_red"]:
                    light.set_state(carla.TrafficLightState.Red)
                    light.set_red_time(self.config.duration + 5.0)
                    light_state["turned_red"] = True
                    logging.info("Traffic light turned RED at frame %d", frame)
                    # Turn cross traffic green
                    try:
                        group = light.get_group()
                    except AttributeError:
                        group = []
                    for other in group:
                        if other.id != light.id:
                            other.set_state(carla.TrafficLightState.Green)
                            other.set_green_time(self.config.duration + 5.0)

            except RuntimeError as e:
                logging.warning("Traffic light control failed: %s", e)

        ctx.tick_callbacks.append(dynamic_light_control)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
