"""Yield to emergency vehicle scenario."""

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


class YieldToEmergencyScenario(BaseScenario):
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

        emergency_spawn = offset_transform(ego_spawn, forward=-20.0)
        try:
            emergency = self._spawn_vehicle(
                world,
                tm,
                rng,
                blueprint_filter="vehicle.ford.ambulance",
                transform=emergency_spawn,
                role_name="emergency",
                autopilot=True,
            )
        except RuntimeError:
            emergency = self._spawn_vehicle(
                world,
                tm,
                rng,
                blueprint_filter="vehicle.*",
                transform=emergency_spawn,
                role_name="emergency",
                autopilot=True,
            )
        log_spawn(emergency, "emergency")

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, emergency],
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )
        ctx.tag_actor("emergency", emergency)

        boost_frames = int(self.config.params.get("emergency_boost_frames", self.config.fps))
        throttle = float(self.config.params.get("emergency_throttle", 0.75))

        def boost_emergency(frame: int) -> None:
            if frame < boost_frames:
                emergency.apply_control(carla.VehicleControl(throttle=throttle))

        ctx.tick_callbacks.append(boost_emergency)
        return ctx
