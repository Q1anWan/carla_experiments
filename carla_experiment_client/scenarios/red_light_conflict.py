"""Red light conflict scenario."""

from __future__ import annotations

import random

import carla

from .base import (
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

        if cross_spawn is None:
            cross_spawn = offset_transform(ego_spawn, right=8.0, forward=8.0)
        cross_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=cross_spawn,
            role_name="cross_vehicle",
            autopilot=False,
        )
        log_spawn(cross_vehicle, "cross_vehicle")

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
                cross_spawn.location,
            ],
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, cross_vehicle] + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        cross_release_frame = params.get("cross_release_frame")

        def force_red(frame: int) -> None:
            light = traffic_light or ego.get_traffic_light()
            if light is None:
                if cross_release_frame is not None and frame == int(cross_release_frame):
                    cross_vehicle.set_autopilot(True, tm.get_port())
                return
            try:
                light.set_state(carla.TrafficLightState.Red)
                light.set_red_time(self.config.duration + 5.0)
                for other in light.get_group():
                    if other.id == light.id:
                        continue
                    other.set_state(carla.TrafficLightState.Green)
                    other.set_green_time(self.config.duration + 5.0)
            except RuntimeError:
                return
            if cross_release_frame is None:
                cross_vehicle.set_autopilot(True, tm.get_port())
                return
            release_frame = int(cross_release_frame)
            if frame < release_frame:
                cross_vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                )
            elif frame == release_frame:
                cross_vehicle.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(force_red)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
