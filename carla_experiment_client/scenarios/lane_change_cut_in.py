"""Lane change cut-in scenario."""

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
        ) or find_spawn_point(
            world,
            rng,
            min_lanes=3,
            avoid_junction=True,
            forward_clear_m=150.0,
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

        nearby_vehicles: list[carla.Actor] = []
        nearby_offsets = self.config.params.get("nearby_vehicle_offsets") or [
            {"forward": 10.0, "right": 3.5},
            {"forward": -6.0, "right": -3.5},
        ]
        if isinstance(nearby_offsets, list):
            for index, offset in enumerate(nearby_offsets):
                if not isinstance(offset, dict):
                    continue
                forward = float(offset.get("forward", 0.0))
                right = float(offset.get("right", 0.0))
                transform = offset_transform(ego_spawn, forward=forward, right=right)
                vehicle = self._spawn_vehicle(
                    world,
                    tm,
                    rng,
                    blueprint_filter="vehicle.*",
                    transform=transform,
                    role_name=f"nearby_vehicle_{index}",
                    autopilot=True,
                )
                log_spawn(vehicle, f"nearby_vehicle_{index}")
                nearby_vehicles.append(vehicle)

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

        lead_distance = float(self.config.params.get("lead_slow_distance_m", 25.0))
        lead_speed_delta = float(self.config.params.get("lead_slow_speed_delta", 35.0))
        lead_spawn = offset_transform(ego_spawn, forward=lead_distance)
        lead_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=lead_spawn,
            role_name="lead_slow",
            autopilot=True,
        )
        tm.vehicle_percentage_speed_difference(lead_vehicle, lead_speed_delta)
        log_spawn(lead_vehicle, "lead_slow")

        background_vehicle_count = int(self.config.params.get("background_vehicle_count", 20))
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
                cut_in_spawn.location,
                lead_spawn.location,
                *[vehicle.get_location() for vehicle in nearby_vehicles],
            ],
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego, cutter, lead_vehicle] + nearby_vehicles + background,
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
        relocate_on_trigger = bool(self.config.params.get("cut_in_relocate_on_trigger", False))
        relocate_forward = float(self.config.params.get("cut_in_relocate_forward_m", 8.0))
        relocate_right = float(self.config.params.get("cut_in_relocate_right_m", 3.5))

        def cut_in(frame: int) -> None:
            if frame == start_frame:
                if relocate_on_trigger:
                    ego_transform = ego.get_transform()
                    relocate_transform = offset_transform(
                        ego_transform, forward=relocate_forward, right=relocate_right
                    )
                    cutter.set_transform(relocate_transform)
                cutter.set_autopilot(False)
            if start_frame <= frame < start_frame + duration_frames:
                cutter.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
            if frame == start_frame + duration_frames:
                cutter.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(cut_in)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
