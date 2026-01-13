"""Scenario base types and helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import random
from typing import Any, Callable, Dict, Iterable, Optional

import carla

from ..config import CameraConfig, ScenarioConfig


TickFn = Callable[[int], None]


@dataclass
class ScenarioContext:
    world: carla.World
    ego_vehicle: carla.Vehicle
    actors: list[carla.Actor]
    camera_config: CameraConfig
    fps: int
    duration: float
    fixed_delta_seconds: float
    seed: int
    scenario_id: str
    tick_callbacks: list[TickFn] = field(default_factory=list)
    tags: Dict[str, list[carla.Actor]] = field(default_factory=dict)

    def on_tick(self, frame_index: int) -> None:
        for callback in list(self.tick_callbacks):
            callback(frame_index)

    def tag_actor(self, tag: str, actor: carla.Actor) -> None:
        self.tags.setdefault(tag, []).append(actor)


class BaseScenario:
    def __init__(self, config: ScenarioConfig) -> None:
        self.config = config

    def build(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
    ) -> ScenarioContext:
        raise NotImplementedError

    def _spawn_vehicle(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
        *,
        blueprint_filter: str,
        transform: carla.Transform,
        role_name: str,
        autopilot: bool = True,
    ) -> carla.Vehicle:
        blueprint_library = world.get_blueprint_library()
        blueprints = blueprint_library.filter(blueprint_filter)
        if not blueprints:
            raise RuntimeError(f"No blueprints for '{blueprint_filter}'")
        blueprint = rng.choice(blueprints)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", role_name)
        candidates = [
            transform,
            offset_transform(transform, forward=2.0),
            offset_transform(transform, forward=-2.0),
            offset_transform(transform, right=2.0),
            offset_transform(transform, right=-2.0),
            offset_transform(transform, forward=4.0),
            offset_transform(transform, forward=-4.0),
            offset_transform(transform, right=4.0),
            offset_transform(transform, right=-4.0),
        ]
        vehicle = None
        for candidate in candidates:
            vehicle = world.try_spawn_actor(blueprint, candidate)
            if vehicle is not None:
                if candidate != transform:
                    logging.warning("Spawn fallback used for %s", role_name)
                break
        if vehicle is None:
            spawn_points = world.get_map().get_spawn_points()
            for _ in range(10):
                candidate = rng.choice(spawn_points)
                vehicle = world.try_spawn_actor(blueprint, candidate)
                if vehicle is not None:
                    logging.warning("Random spawn fallback used for %s", role_name)
                    break
        if vehicle is None:
            raise RuntimeError(f"Failed to spawn vehicle for {role_name}")
        if autopilot:
            vehicle.set_autopilot(True, tm.get_port())
        return vehicle

    def _spawn_walker(
        self,
        world: carla.World,
        rng: random.Random,
        transform: carla.Transform,
        *,
        speed: float = 1.3,
        role_name: str = "walker",
    ) -> tuple[carla.Actor, carla.Actor]:
        blueprint_library = world.get_blueprint_library()
        walkers = blueprint_library.filter("walker.pedestrian.*")
        if not walkers:
            raise RuntimeError("No walker blueprints available")
        walker_bp = rng.choice(walkers)
        if walker_bp.has_attribute("role_name"):
            walker_bp.set_attribute("role_name", role_name)
        walker = world.try_spawn_actor(walker_bp, transform)
        if walker is None:
            raise RuntimeError("Failed to spawn walker")
        controller_bp = blueprint_library.find("controller.ai.walker")
        controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
        controller.set_max_speed(speed)
        return walker, controller

    def _spawn_background_traffic(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
        *,
        vehicle_count: int,
        walker_count: int,
        exclude_locations: Iterable[carla.Location],
        min_distance: float = 15.0,
    ) -> list[carla.Actor]:
        actors: list[carla.Actor] = []
        if vehicle_count > 0:
            blueprint_library = world.get_blueprint_library()
            blueprints = blueprint_library.filter("vehicle.*")
            spawn_points = world.get_map().get_spawn_points()
            rng.shuffle(spawn_points)
            for sp in spawn_points:
                if len([a for a in actors if isinstance(a, carla.Vehicle)]) >= vehicle_count:
                    break
                if any(sp.location.distance(loc) < min_distance for loc in exclude_locations):
                    continue
                blueprint = rng.choice(blueprints)
                vehicle = world.try_spawn_actor(blueprint, sp)
                if vehicle is None:
                    continue
                vehicle.set_autopilot(True, tm.get_port())
                actors.append(vehicle)

        if walker_count > 0:
            for _ in range(walker_count * 3):
                if len([a for a in actors if a.type_id.startswith("walker.pedestrian")]) >= walker_count:
                    break
                location = world.get_random_location_from_navigation()
                if location is None:
                    continue
                if any(location.distance(loc) < min_distance for loc in exclude_locations):
                    continue
                walker_transform = carla.Transform(location)
                walker, controller = self._spawn_walker(world, rng, walker_transform, speed=1.4)
                controller.start()
                dest = world.get_random_location_from_navigation()
                if dest is not None:
                    controller.go_to_location(dest)
                actors.extend([walker, controller])
        return actors

    def _maybe_add_ego_brake(self, ctx: ScenarioContext, tm: carla.TrafficManager) -> None:
        params = self.config.params
        if "ego_brake_frame" not in params:
            return
        brake_frame = int(params.get("ego_brake_frame"))
        duration_frames = int(params.get("ego_brake_duration_frames", int(ctx.fps * 0.5)))
        brake_value = float(params.get("ego_brake_value", 1.0))
        resume_autopilot = bool(params.get("ego_brake_resume_autopilot", True))

        def apply_brake(frame_index: int) -> None:
            if frame_index == brake_frame:
                ctx.ego_vehicle.set_autopilot(False)
            if brake_frame <= frame_index < brake_frame + duration_frames:
                ctx.ego_vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=brake_value)
                )
            if resume_autopilot and frame_index == brake_frame + duration_frames:
                ctx.ego_vehicle.set_autopilot(True, tm.get_port())

        ctx.tick_callbacks.append(apply_brake)

    def _configure_vehicle_tm(
        self,
        tm: carla.TrafficManager,
        vehicle: carla.Vehicle,
        *,
        speed_delta: Optional[float] = None,
        follow_distance: Optional[float] = None,
        ignore_lights: Optional[float] = None,
        ignore_vehicles: Optional[float] = None,
        auto_lane_change: Optional[bool] = None,
    ) -> None:
        if speed_delta is not None:
            try:
                tm.vehicle_percentage_speed_difference(vehicle, float(speed_delta))
            except RuntimeError:
                logging.warning("TM speed delta failed for %s", vehicle.id)
        if follow_distance is not None:
            try:
                tm.distance_to_leading_vehicle(vehicle, float(follow_distance))
            except (AttributeError, RuntimeError):
                logging.warning("TM follow distance unsupported for %s", vehicle.id)
        if ignore_lights is not None:
            try:
                tm.ignore_lights_percentage(vehicle, float(ignore_lights))
            except (AttributeError, RuntimeError):
                logging.warning("TM ignore lights unsupported for %s", vehicle.id)
        if ignore_vehicles is not None:
            try:
                tm.ignore_vehicles_percentage(vehicle, float(ignore_vehicles))
            except (AttributeError, RuntimeError):
                logging.warning("TM ignore vehicles unsupported for %s", vehicle.id)
        if auto_lane_change is not None:
            try:
                tm.auto_lane_change(vehicle, bool(auto_lane_change))
            except (AttributeError, RuntimeError):
                logging.warning("TM auto lane change unsupported for %s", vehicle.id)

    def _apply_ego_tm(self, tm: carla.TrafficManager, ego: carla.Vehicle) -> None:
        params = self.config.params
        self._configure_vehicle_tm(
            tm,
            ego,
            speed_delta=_get_param_float(params, "ego_speed_delta"),
            follow_distance=_get_param_float(params, "ego_follow_distance_m"),
            ignore_lights=_get_param_float(params, "ego_ignore_lights_percentage"),
            ignore_vehicles=_get_param_float(params, "ego_ignore_vehicles_percentage"),
            auto_lane_change=_get_param_bool(params, "ego_auto_lane_change"),
        )


def pick_spawn_point(spawn_points: Iterable[carla.Transform], rng: random.Random) -> carla.Transform:
    points = list(spawn_points)
    if not points:
        raise RuntimeError("No spawn points available")
    return rng.choice(points)


def get_spawn_point_by_index(
    spawn_points: Iterable[carla.Transform], index: Optional[int]
) -> Optional[carla.Transform]:
    if index is None:
        return None
    points = list(spawn_points)
    if not points:
        return None
    if index < 0 or index >= len(points):
        return None
    return points[index]


def offset_transform(
    transform: carla.Transform,
    *,
    forward: float = 0.0,
    right: float = 0.0,
    up: float = 0.0,
) -> carla.Transform:
    location = transform.location
    location = location + transform.get_forward_vector() * forward
    location = location + right_vector(transform) * right
    location = location + carla.Location(z=up)
    return carla.Transform(location, transform.rotation)


def right_vector(transform: carla.Transform) -> carla.Vector3D:
    try:
        return transform.get_right_vector()
    except AttributeError:
        yaw = math.radians(transform.rotation.yaw)
        return carla.Vector3D(x=-math.sin(yaw), y=math.cos(yaw), z=0.0)


def log_spawn(actor: carla.Actor, label: str) -> None:
    logging.info("Spawned %s at %s", label, actor.get_transform().location)


def count_driving_lanes(waypoint: carla.Waypoint) -> int:
    count = 1
    left = waypoint.get_left_lane()
    while left and left.lane_type == carla.LaneType.Driving:
        count += 1
        left = left.get_left_lane()
    right = waypoint.get_right_lane()
    while right and right.lane_type == carla.LaneType.Driving:
        count += 1
        right = right.get_right_lane()
    return count


def has_junction_ahead(waypoint: carla.Waypoint, distance: float, step: float = 5.0) -> bool:
    traveled = 0.0
    current = waypoint
    while traveled < distance:
        next_wps = current.next(step)
        if not next_wps:
            return False
        current = next_wps[0]
        traveled += step
        if current.is_junction:
            return True
    return False


def collect_stop_waypoints(world: carla.World) -> list[carla.Waypoint]:
    stop_waypoints: list[carla.Waypoint] = []
    for light in world.get_actors().filter("traffic.traffic_light"):
        try:
            stop_waypoints.extend(light.get_stop_waypoints())
        except RuntimeError:
            continue
    return stop_waypoints


def is_near_stop_waypoint(
    stop_waypoints: Iterable[carla.Waypoint],
    location: carla.Location,
    threshold: float,
) -> bool:
    return any(wp.transform.location.distance(location) <= threshold for wp in stop_waypoints)


def find_spawn_point(
    world: carla.World,
    rng: random.Random,
    *,
    min_lanes: int = 1,
    avoid_junction: bool = True,
    forward_clear_m: float = 0.0,
    require_junction_ahead: bool = False,
    junction_ahead_m: float = 60.0,
    avoid_traffic_lights: bool = False,
    traffic_light_threshold_m: float = 30.0,
) -> carla.Transform:
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available")
    rng.shuffle(spawn_points)
    stop_waypoints = collect_stop_waypoints(world) if avoid_traffic_lights else []
    for sp in spawn_points:
        waypoint = world.get_map().get_waypoint(sp.location)
        if avoid_junction and waypoint.is_junction:
            continue
        if min_lanes > 1 and count_driving_lanes(waypoint) < min_lanes:
            continue
        if forward_clear_m and has_junction_ahead(waypoint, forward_clear_m):
            continue
        if require_junction_ahead and not has_junction_ahead(waypoint, junction_ahead_m):
            continue
        if avoid_traffic_lights and is_near_stop_waypoint(
            stop_waypoints, sp.location, traffic_light_threshold_m
        ):
            continue
        return sp
    return rng.choice(spawn_points)


def _get_param_float(params: Dict[str, Any], key: str) -> Optional[float]:
    if key not in params or params[key] is None:
        return None
    return float(params[key])


def _get_param_bool(params: Dict[str, Any], key: str) -> Optional[bool]:
    if key not in params:
        return None
    return bool(params[key])
