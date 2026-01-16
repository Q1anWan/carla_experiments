"""Unprotected left turn scenario."""

from __future__ import annotations

import logging
import random
from typing import Optional

import carla

from ..base import (
    BaseScenario,
    ScenarioContext,
    find_spawn_point,
    get_spawn_point_by_index,
    log_spawn,
    offset_transform,
    pick_spawn_point,
)


def _normalize_yaw(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw


def _find_junction_ahead(
    map_obj: carla.Map, waypoint: carla.Waypoint, distance: float, step: float = 5.0
) -> Optional[carla.Junction]:
    traveled = 0.0
    current = waypoint
    while traveled < distance:
        next_wps = current.next(step)
        if not next_wps:
            return None
        current = next_wps[0]
        traveled += step
        if current.is_junction:
            return current.get_junction()
    return None


def _select_turn_yaw(
    junction: carla.Junction, entry_wp: carla.Waypoint
) -> Optional[float]:
    entry_yaw = entry_wp.transform.rotation.yaw
    candidates: list[float] = []
    for entry, exit_wp in junction.get_waypoints(carla.LaneType.Driving):
        if entry.road_id != entry_wp.road_id or entry.lane_id != entry_wp.lane_id:
            continue
        yaw_diff = _normalize_yaw(exit_wp.transform.rotation.yaw - entry_yaw)
        if abs(yaw_diff) < 25.0 or abs(yaw_diff) > 160.0:
            continue
        candidates.append(yaw_diff)
    left_candidates = [diff for diff in candidates if diff < -15.0]
    if left_candidates:
        return min(left_candidates)
    if candidates:
        return max(candidates, key=abs)
    return None


def _find_opposing_lane(waypoint: carla.Waypoint) -> Optional[carla.Waypoint]:
    for candidate in (waypoint.get_left_lane(), waypoint.get_right_lane()):
        if candidate is None:
            continue
        if candidate.lane_type != carla.LaneType.Driving:
            continue
        if candidate.lane_id * waypoint.lane_id < 0:
            return candidate
    return None


class UnprotectedLeftTurnScenario(BaseScenario):
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
        turn_sign = None
        if ego_spawn is None and bool(params.get("fast_spawn")):
            ego_spawn = pick_spawn_point(spawn_points, rng)
        if ego_spawn is None:
            candidate = None
            for sp in rng.sample(spawn_points, k=min(len(spawn_points), 40)):
                waypoint = world.get_map().get_waypoint(sp.location)
                if waypoint.is_junction:
                    continue
                junction = _find_junction_ahead(world.get_map(), waypoint, 90.0)
                if junction is None:
                    continue
                yaw_diff = _select_turn_yaw(junction, waypoint)
                if yaw_diff is None:
                    continue
                candidate = sp
                turn_sign = -1.0 if yaw_diff < 0 else 1.0
                break
            if candidate is None:
                candidate = find_spawn_point(
                    world,
                    rng,
                    min_lanes=1,
                    avoid_junction=True,
                    require_junction_ahead=True,
                    junction_ahead_m=90.0,
                    avoid_traffic_lights=True,
                )
            ego_spawn = candidate
        if turn_sign is None:
            waypoint = world.get_map().get_waypoint(ego_spawn.location)
            junction = _find_junction_ahead(world.get_map(), waypoint, 90.0)
            if junction is not None:
                yaw_diff = _select_turn_yaw(junction, waypoint)
                if yaw_diff is not None:
                    turn_sign = -1.0 if yaw_diff < 0 else 1.0

        ego = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter=self.config.ego_vehicle,
            transform=ego_spawn,
            role_name="ego",
            autopilot=True,
        )
        log_spawn(ego, "ego", ego_spawn)
        self._apply_ego_tm(tm, ego)

        # Spawn multiple oncoming vehicles for conflict
        oncoming_vehicle_count = int(params.get("oncoming_vehicle_count", 1))
        oncoming_spawn_distance = float(params.get("oncoming_spawn_distance_m", 30.0))
        oncoming_vehicle_spacing = float(params.get("oncoming_vehicle_spacing_m", 8.0))
        oncoming_vehicles: list[carla.Actor] = []

        ego_wp = world.get_map().get_waypoint(ego_spawn.location)
        oncoming_wp = None
        for forward_wp in ego_wp.next(oncoming_spawn_distance):
            oncoming_wp = _find_opposing_lane(forward_wp)
            if oncoming_wp:
                break

        for i in range(oncoming_vehicle_count):
            if oncoming_wp is not None:
                # Space oncoming vehicles along the opposing lane
                spaced_wps = oncoming_wp.previous(i * oncoming_vehicle_spacing)
                if spaced_wps:
                    oncoming_spawn = spaced_wps[0].transform
                else:
                    oncoming_spawn = oncoming_wp.transform
            else:
                oncoming_spawn = offset_transform(ego_spawn, forward=oncoming_spawn_distance + i * oncoming_vehicle_spacing)

            try:
                oncoming = self._spawn_vehicle(
                    world,
                    tm,
                    rng,
                    blueprint_filter="vehicle.*",
                    transform=oncoming_spawn,
                    role_name=f"oncoming_vehicle_{i}",
                    autopilot=True,
                )
                log_spawn(oncoming, f"oncoming_vehicle_{i}", oncoming_spawn)
                oncoming_vehicles.append(oncoming)
            except RuntimeError:
                logging.warning("Failed to spawn oncoming vehicle %d", i)

        # Spawn nearby vehicles for traffic density
        nearby_vehicles: list[carla.Actor] = []
        nearby_offsets = params.get("nearby_vehicle_offsets") or []
        if isinstance(nearby_offsets, list):
            for index, offset in enumerate(nearby_offsets):
                if not isinstance(offset, dict):
                    continue
                forward = float(offset.get("forward", 0.0))
                right = float(offset.get("right", 0.0))
                transform = offset_transform(ego_spawn, forward=forward, right=right)
                try:
                    vehicle = self._spawn_vehicle(
                        world,
                        tm,
                        rng,
                        blueprint_filter="vehicle.*",
                        transform=transform,
                        role_name=f"nearby_vehicle_{index}",
                        autopilot=True,
                    )
                    log_spawn(vehicle, f"nearby_vehicle_{index}", transform)
                    nearby_vehicles.append(vehicle)
                except RuntimeError:
                    logging.warning("Failed to spawn nearby vehicle %d", index)

        background_vehicle_count = int(params.get("background_vehicle_count", 16))
        background_walker_count = int(params.get("background_walker_count", 6))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        exclude_locs = [ego_spawn.location]
        exclude_locs += [v.get_location() for v in oncoming_vehicles]
        exclude_locs += [v.get_location() for v in nearby_vehicles]
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=exclude_locs,
            min_distance=background_min_distance,
        )

        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego] + oncoming_vehicles + nearby_vehicles + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        use_autopilot_turn = bool(params.get("use_autopilot_turn", False))
        approach_frames = int(params.get("approach_frames", self.config.fps * 2))
        turn_frames = int(params.get("turn_frames", self.config.fps * 2))
        throttle = float(params.get("ego_throttle", 0.5))
        turn_steer = float(params.get("turn_steer", -0.25))
        turn_throttle = float(params.get("turn_throttle", throttle * 0.7))
        auto_turn = bool(params.get("turn_steer_auto", True))
        if auto_turn and turn_sign is not None:
            turn_steer = abs(turn_steer) * turn_sign

        if use_autopilot_turn:
            # Use TM route planner to force left turn - ego stays on autopilot
            # Configure TM to prefer left turns at junctions
            tm.set_random_device_seed(self.config.seed or 42)
            try:
                # Force lane change left when possible
                tm.force_lane_change(ego, False)  # False = left
            except RuntimeError:
                pass
            logging.info("Using autopilot for left turn (TM-controlled)")

            def control_ego(frame: int) -> None:
                # Just slow down near the turn, let TM handle steering
                if approach_frames <= frame < approach_frames + turn_frames:
                    # Apply gentle braking during turn for safety
                    vel = ego.get_velocity()
                    speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
                    if speed > 5.0:  # If going faster than 5 m/s
                        ego.apply_control(carla.VehicleControl(throttle=0.1, brake=0.3))
        else:
            # Manual steering control (original behavior)
            def control_ego(frame: int) -> None:
                if frame == approach_frames:
                    ego.set_autopilot(False)
                    logging.info("Left turn maneuver started at frame %d, steer=%.2f", frame, turn_steer)
                if approach_frames <= frame < approach_frames + turn_frames:
                    ego.apply_control(
                        carla.VehicleControl(throttle=turn_throttle, steer=turn_steer)
                    )
                if frame == approach_frames + turn_frames:
                    ego.set_autopilot(True, tm.get_port())
                    logging.info("Left turn maneuver completed at frame %d", frame)

        ctx.tick_callbacks.append(control_ego)
        self._maybe_add_ego_brake(ctx, tm)
        return ctx
