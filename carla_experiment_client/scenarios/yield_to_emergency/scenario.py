"""Yield to emergency vehicle scenario."""

from __future__ import annotations

import random

import carla

from ..base import (
    BaseScenario,
    ScenarioContext,
    _get_param_float,
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

        emergency_distance = float(params.get("emergency_spawn_distance_m", 35.0))
        # Use positive forward to spawn BEHIND ego (offset_transform uses spawn orientation,
        # which may face opposite to world direction on some maps/spawn points)
        emergency_spawn = offset_transform(ego_spawn, forward=emergency_distance)
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
            lights = carla.VehicleLightState(
                carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
            )
            emergency.set_light_state(lights)
        except (RuntimeError, TypeError, AttributeError):
            pass
        self._configure_vehicle_tm(
            tm,
            emergency,
            speed_delta=_get_param_float(params, "emergency_speed_delta"),
            follow_distance=_get_param_float(params, "emergency_follow_distance_m"),
        )

        background_vehicle_count = int(params.get("background_vehicle_count", 20))
        background_walker_count = int(params.get("background_walker_count", 10))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
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
            min_distance=background_min_distance,
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

        boost_start = int(params.get("emergency_boost_start_frame", 0))
        boost_frames = int(params.get("emergency_boost_frames", self.config.fps * 2))
        throttle = float(params.get("emergency_throttle", 0.85))

        def boost_emergency(frame: int) -> None:
            if frame == boost_start:
                emergency.set_autopilot(False)
            if boost_start <= frame < boost_start + boost_frames:
                emergency.apply_control(carla.VehicleControl(throttle=throttle))
            if frame == boost_start + boost_frames:
                emergency.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(boost_emergency)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
