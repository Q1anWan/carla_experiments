"""Pedestrian emerge (ghost pedestrian) scenario."""

from __future__ import annotations

import random

import carla

from .base import (
    BaseScenario,
    ScenarioContext,
    get_spawn_point_by_index,
    log_spawn,
    pick_spawn_point,
    right_vector,
)


class PedestrianEmergeScenario(BaseScenario):
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

        walker_location = None
        for _ in range(10):
            candidate = world.get_random_location_from_navigation()
            if candidate is None:
                continue
            if candidate.distance(ego_spawn.location) < 30.0:
                walker_location = candidate
                break
        if walker_location is None:
            walker_location = ego_spawn.location + right_vector(ego_spawn) * 6.0

        walker_transform = carla.Transform(walker_location)
        walker, controller = self._spawn_walker(world, rng, walker_transform, speed=1.4)
        log_spawn(walker, "pedestrian")

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, walker, controller],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )
        ctx.tag_actor("pedestrian", walker)

        trigger_distance = float(self.config.params.get("trigger_distance", 12.0))
        target_offset = float(self.config.params.get("cross_offset", 8.0))
        started = {"value": False}
        target_location = walker_location + right_vector(ego_spawn) * target_offset

        def trigger(frame: int) -> None:
            if started["value"]:
                return
            ego_loc = ego.get_location()
            if ego_loc.distance(walker_location) <= trigger_distance:
                controller.start()
                controller.go_to_location(target_location)
                started["value"] = True

        ctx.tick_callbacks.append(trigger)
        return ctx
