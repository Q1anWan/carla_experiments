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
    offset_transform,
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
        log_spawn(ego, "ego")
        self._apply_ego_tm(tm, ego)

        ahead_m = float(self.config.params.get("walker_start_ahead_m", 35.0))
        side_m = float(self.config.params.get("walker_side_offset_m", 6.0))
        relocate_on_trigger = bool(self.config.params.get("relocate_on_trigger", True))
        walker_location = ego_spawn.location + ego_spawn.get_forward_vector() * ahead_m
        walker_location = walker_location + right_vector(ego_spawn) * side_m

        walker_transform = carla.Transform(walker_location)
        walker, controller = self._spawn_walker(world, rng, walker_transform, speed=1.4)
        log_spawn(walker, "pedestrian")

        occluder_forward = float(self.config.params.get("occluder_forward_m", 18.0))
        occluder_side = float(self.config.params.get("occluder_side_offset_m", 3.5))
        occluder_bp = self.config.params.get("occluder_blueprint", "vehicle.*")
        occluder_transform = offset_transform(
            ego_spawn, forward=occluder_forward, right=occluder_side
        )
        try:
            occluder = self._spawn_vehicle(
                world,
                tm,
                rng,
                blueprint_filter=str(occluder_bp),
                transform=occluder_transform,
                role_name="occluder_vehicle",
                autopilot=False,
            )
        except RuntimeError:
            occluder = self._spawn_vehicle(
                world,
                tm,
                rng,
                blueprint_filter="vehicle.*",
                transform=occluder_transform,
                role_name="occluder_vehicle",
                autopilot=False,
            )
        occluder.apply_control(
            carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
        )
        log_spawn(occluder, "occluder_vehicle")

        background_vehicle_count = int(self.config.params.get("background_vehicle_count", 16))
        background_walker_count = int(self.config.params.get("background_walker_count", 12))
        background_min_distance = float(self.config.params.get("background_min_distance_m", 20.0))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=[
                ego_spawn.location,
                walker_location,
                occluder_transform.location,
            ],
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, walker, controller, occluder] + background,
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
        trigger_frame = self.config.params.get("trigger_frame")
        started = {"value": False}
        target_state = {
            "location": walker_location + right_vector(ego_spawn) * target_offset
        }

        def trigger(frame: int) -> None:
            if started["value"]:
                return
            if trigger_frame is not None and frame < int(trigger_frame):
                return
            if trigger_frame is None:
                ego_loc = ego.get_location()
                if ego_loc.distance(walker_location) > trigger_distance:
                    return
            if relocate_on_trigger:
                ego_transform = ego.get_transform()
                new_walker_location = (
                    ego_transform.location + ego_transform.get_forward_vector() * ahead_m
                )
                new_walker_location = new_walker_location + right_vector(ego_transform) * side_m
                walker.set_transform(carla.Transform(new_walker_location))
                new_occluder_transform = offset_transform(
                    ego_transform, forward=occluder_forward, right=occluder_side
                )
                occluder.set_transform(new_occluder_transform)
                occluder.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                )
                target_state["location"] = new_walker_location + right_vector(ego_transform) * target_offset
            controller.start()
            controller.go_to_location(target_state["location"])
            started["value"] = True

        ctx.tick_callbacks.append(trigger)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
