"""Lane change cut-in scenario."""

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


class LaneChangeCutInScenario(BaseScenario):
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

        cut_in_spawn = offset_transform(ego_spawn, forward=12.0, right=3.5)
        cutter = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=cut_in_spawn,
            role_name="cut_in_vehicle",
            autopilot=True,
        )
        log_spawn(cutter, "cut_in_vehicle")

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, cutter],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        start_frame = int(self.config.params.get("cut_in_trigger_frame", self.config.fps * 2))
        duration_frames = int(self.config.params.get("cut_in_duration_frames", self.config.fps))
        throttle = float(self.config.params.get("cut_in_throttle", 0.55))
        steer = float(self.config.params.get("cut_in_steer", -0.22))

        def cut_in(frame: int) -> None:
            if frame == start_frame:
                cutter.set_autopilot(False)
            if start_frame <= frame < start_frame + duration_frames:
                cutter.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
            if frame == start_frame + duration_frames:
                cutter.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(cut_in)
        return ctx
