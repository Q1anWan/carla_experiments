"""Pedestrian emerge (ghost pedestrian) scenario."""

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
    right_vector,
)


class PedestrianEmergeScenario(BaseScenario):
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
        log_spawn(ego, "ego")
        self._apply_ego_tm(tm, ego)

        ahead_m = float(params.get("walker_start_ahead_m", 35.0))
        side_m = float(params.get("walker_side_offset_m", 6.0))
        # Separate relocation distances (default to start distances)
        relocate_ahead_m = float(params.get("relocate_ahead_m", ahead_m))
        relocate_side_m = float(params.get("relocate_side_m", side_m))
        walker_speed = float(params.get("walker_speed", 1.4))
        relocate_on_trigger = bool(params.get("relocate_on_trigger", True))
        walker_location = ego_spawn.location + ego_spawn.get_forward_vector() * ahead_m
        walker_location = walker_location + right_vector(ego_spawn) * side_m

        walker_transform = carla.Transform(walker_location)
        walker, controller = self._spawn_walker(world, rng, walker_transform, speed=walker_speed)
        log_spawn(walker, "pedestrian")

        occluder_forward = float(params.get("occluder_forward_m", 18.0))
        occluder_side = float(params.get("occluder_side_offset_m", 3.5))
        occluder_bp = params.get("occluder_blueprint", "vehicle.*")
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

        background_vehicle_count = int(params.get("background_vehicle_count", 16))
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

        trigger_distance = float(params.get("trigger_distance", 25.0))
        target_offset = float(params.get("cross_offset", 8.0))
        trigger_frame = params.get("trigger_frame")
        started = {"value": False}
        target_state = {
            "location": walker_location + right_vector(ego_spawn) * target_offset
        }

        # Store mutable references for walker/controller that may be replaced
        walker_ref = {"walker": walker, "controller": controller}

        def trigger(frame: int) -> None:
            if started["value"]:
                return
            if trigger_frame is not None and frame < int(trigger_frame):
                return
            if trigger_frame is None:
                ego_loc = ego.get_location()
                if ego_loc.distance(walker_location) > trigger_distance:
                    return

            current_walker = walker_ref["walker"]
            current_controller = walker_ref["controller"]

            if relocate_on_trigger:
                ego_transform = ego.get_transform()
                new_walker_location = (
                    ego_transform.location + ego_transform.get_forward_vector() * relocate_ahead_m
                )
                new_walker_location = new_walker_location + right_vector(ego_transform) * relocate_side_m
                # CARLA walker teleportation is unreliable, so destroy and respawn
                # First destroy the old walker and controller
                if current_controller is not None:
                    try:
                        current_controller.stop()
                    except RuntimeError:
                        pass
                    current_controller.destroy()
                current_walker.destroy()
                # Spawn new walker at the relocated position
                new_transform = carla.Transform(new_walker_location)
                new_walker, new_controller = self._spawn_walker(world, rng, new_transform, speed=walker_speed)
                walker_ref["walker"] = new_walker
                walker_ref["controller"] = new_controller
                current_walker = new_walker
                current_controller = new_controller
                # Update actor list in context
                ctx.tag_actor("pedestrian", new_walker)
                logging.info("Pedestrian trigger fired at frame %d. Walker respawned at (%.1f, %.1f)",
                           frame, new_walker_location.x, new_walker_location.y)
                new_occluder_transform = offset_transform(
                    ego_transform, forward=occluder_forward, right=occluder_side
                )
                occluder.set_transform(new_occluder_transform)
                occluder.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                )
                target_state["location"] = new_walker_location + right_vector(ego_transform) * target_offset
            current_controller.start()
            current_controller.go_to_location(target_state["location"])
            # Set speed after start and go_to_location for proper initialization
            try:
                current_controller.set_max_speed(walker_speed)
            except RuntimeError:
                pass
            started["value"] = True

        ctx.tick_callbacks.append(trigger)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
