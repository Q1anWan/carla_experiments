"""Yield to emergency vehicle scenario."""

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
        ) or find_spawn_point(
            world,
            rng,
            min_lanes=2,
            avoid_junction=True,
            forward_clear_m=60.0,
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

        emergency_spawn = offset_transform(ego_spawn, forward=-25.0)
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

        try:
            emergency.set_light_state(
                carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
            )
        except RuntimeError:
            pass

        background_vehicle_count = int(self.config.params.get("background_vehicle_count", 20))
        background_walker_count = int(self.config.params.get("background_walker_count", 10))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=[
                ego_spawn.location,
                emergency_spawn.location,
            ],
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, emergency] + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )
        ctx.tag_actor("emergency", emergency)

        boost_frames = int(self.config.params.get("emergency_boost_frames", self.config.fps * 2))
        throttle = float(self.config.params.get("emergency_throttle", 0.8))

        def boost_emergency(frame: int) -> None:
            if frame < boost_frames:
                emergency.apply_control(carla.VehicleControl(throttle=throttle))

        ctx.tick_callbacks.append(boost_emergency)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
