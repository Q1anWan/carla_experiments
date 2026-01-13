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
        ego_spawn = get_spawn_point_by_index(
            spawn_points, self.config.params.get("ego_spawn_index")
        )
        traffic_light = None
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
                previous = stop_wp.previous(25.0)
                if not previous:
                    continue
                ego_spawn = previous[0].transform
                traffic_light = light
                break
        if ego_spawn is None:
            ego_spawn = find_spawn_point(
                world,
                rng,
                min_lanes=1,
                avoid_junction=True,
                forward_clear_m=30.0,
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

        cross_spawn = offset_transform(ego_spawn, right=8.0, forward=8.0)
        cross_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=cross_spawn,
            role_name="cross_vehicle",
            autopilot=True,
        )
        log_spawn(cross_vehicle, "cross_vehicle")

        background_vehicle_count = int(self.config.params.get("background_vehicle_count", 18))
        background_walker_count = int(self.config.params.get("background_walker_count", 10))
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

        def force_red(_: int) -> None:
            light = traffic_light or ego.get_traffic_light()
            if light is None:
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

        ctx.tick_callbacks.append(force_red)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
