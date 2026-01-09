"""Highway merge scenario."""

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
    right_vector,
)


class HighwayMergeScenario(BaseScenario):
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

        waypoint = world.get_map().get_waypoint(ego_spawn.location)
        merge_wp = waypoint.get_right_lane() or waypoint.get_left_lane()
        if merge_wp:
            merge_spawn = merge_wp.transform
        else:
            merge_spawn = offset_transform(ego_spawn, right=3.5, forward=-5.0)
        merge_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=merge_spawn,
            role_name="merge_vehicle",
            autopilot=True,
        )
        log_spawn(merge_vehicle, "merge_vehicle")

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, merge_vehicle],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        start_frame = int(self.config.params.get("merge_trigger_frame", self.config.fps * 2))
        duration_frames = int(self.config.params.get("merge_duration_frames", self.config.fps))
        throttle = float(self.config.params.get("merge_throttle", 0.55))
        base_steer = float(self.config.params.get("merge_steer", 0.2))

        relative = merge_vehicle.get_transform().location - ego_spawn.location
        ego_right = right_vector(ego_spawn)
        right_dot = relative.x * ego_right.x + relative.y * ego_right.y + relative.z * ego_right.z
        steer = -abs(base_steer) if right_dot > 0 else abs(base_steer)

        def merge_trigger(frame: int) -> None:
            if frame == start_frame:
                merge_vehicle.set_autopilot(False)
            if start_frame <= frame < start_frame + duration_frames:
                merge_vehicle.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer)
                )
            if frame == start_frame + duration_frames:
                merge_vehicle.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(merge_trigger)
        return ctx
