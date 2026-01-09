"""Scenario base types and helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import random
from typing import Callable, Dict, Iterable, Optional

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
