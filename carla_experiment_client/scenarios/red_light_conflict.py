"""Red light conflict scenario."""

from __future__ import annotations

import random

import carla

from .base import (
    BaseScenario,
    ScenarioContext,
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
        ) or pick_spawn_point(spawn_points, rng)
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

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, cross_vehicle],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        def force_red(_: int) -> None:
            light = ego.get_traffic_light()
            if light is None:
                return
            try:
                light.set_state(carla.TrafficLightState.Red)
                light.set_red_time(self.config.duration + 5.0)
            except RuntimeError:
                return

        ctx.tick_callbacks.append(force_red)
        return ctx
