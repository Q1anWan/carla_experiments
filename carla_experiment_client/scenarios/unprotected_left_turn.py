"""Unprotected left turn scenario."""

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


class UnprotectedLeftTurnScenario(BaseScenario):
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
            autopilot=False,
        )
        log_spawn(ego, "ego")

        oncoming_spawn = offset_transform(ego_spawn, forward=30.0)
        oncoming = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=oncoming_spawn,
            role_name="oncoming_vehicle",
            autopilot=True,
        )
        log_spawn(oncoming, "oncoming_vehicle")

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, oncoming],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        approach_frames = int(self.config.params.get("approach_frames", self.config.fps * 2))
        turn_frames = int(self.config.params.get("turn_frames", self.config.fps * 2))
        throttle = float(self.config.params.get("ego_throttle", 0.5))
        turn_steer = float(self.config.params.get("turn_steer", -0.35))

        def control_ego(frame: int) -> None:
            if frame < approach_frames:
                ego.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0))
            elif frame < approach_frames + turn_frames:
                ego.apply_control(carla.VehicleControl(throttle=throttle * 0.8, steer=turn_steer))
            else:
                ego.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0))

        ctx.tick_callbacks.append(control_ego)
        return ctx
