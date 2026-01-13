"""Pedestrian emerge (ghost pedestrian) scenario."""

from __future__ import annotations

import random

import carla

from .base import (
    BaseScenario,
    ScenarioContext,
    find_spawn_point,
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
        ) or find_spawn_point(
            world,
            rng,
            min_lanes=2,
            avoid_junction=True,
            forward_clear_m=60.0,
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

        ahead_m = float(self.config.params.get("walker_start_ahead_m", 35.0))
        side_m = float(self.config.params.get("walker_side_offset_m", 6.0))
        walker_location = ego_spawn.location + ego_spawn.get_forward_vector() * ahead_m
        walker_location = walker_location + right_vector(ego_spawn) * side_m

        walker_transform = carla.Transform(walker_location)
        walker, controller = self._spawn_walker(world, rng, walker_transform, speed=1.4)
        log_spawn(walker, "pedestrian")

        background_vehicle_count = int(self.config.params.get("background_vehicle_count", 16))
        background_walker_count = int(self.config.params.get("background_walker_count", 12))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=[
                ego_spawn.location,
                walker_location,
            ],
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, walker, controller] + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )
        ctx.tag_actor("pedestrian", walker)

        trigger_distance = float(self.config.params.get("trigger_distance", 25.0))
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
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
