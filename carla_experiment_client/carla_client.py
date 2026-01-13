"""CARLA client connection helpers and a minimal sync demo."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import time
import random
from typing import Optional

import carla


@dataclass
class CarlaWorldContext:
    client: carla.Client
    world: carla.World
    traffic_manager: carla.TrafficManager
    original_settings: carla.WorldSettings
    server_version: str
    client_version: str


def connect_client(
    host: str,
    port: int,
    timeout: float,
    *,
    allow_version_mismatch: bool = False,
) -> tuple[carla.Client, str, str]:
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    try:
        server_version = client.get_server_version()
    except RuntimeError as exc:
        raise RuntimeError(
            f"Unable to reach CARLA server at {host}:{port}. Check IP/port and firewall."
        ) from exc
    client_version = client.get_client_version()
    if server_version != client_version and not allow_version_mismatch:
        raise RuntimeError(
            f"Version mismatch: server={server_version} client={client_version}."
        )
    if server_version != client_version and allow_version_mismatch:
        logging.warning(
            "Version mismatch ignored: server=%s client=%s", server_version, client_version
        )
    return client, server_version, client_version


def _map_matches(current_map: str, target_map: str) -> bool:
    if current_map == target_map:
        return True
    if current_map.endswith(target_map):
        return True
    return current_map.split("/")[-1] == target_map


def load_world(client: carla.Client, map_name: Optional[str]) -> carla.World:
    if not map_name:
        return client.get_world()
    current_world = client.get_world()
    try:
        current_map = current_world.get_map().name
    except RuntimeError:
        current_map = ""
    if current_map and _map_matches(current_map, map_name):
        logging.info("Map already loaded: %s", current_map)
        return current_world
    logging.info("Loading map %s", map_name)
    return client.load_world(map_name)


def configure_world(
    world: carla.World,
    *,
    sync_mode: bool,
    fixed_delta_seconds: float,
    no_rendering_mode: bool,
) -> carla.WorldSettings:
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = sync_mode
    settings.fixed_delta_seconds = fixed_delta_seconds if sync_mode else None
    settings.no_rendering_mode = no_rendering_mode
    if _settings_match(original, settings):
        logging.info(
            "World settings already match requested sync config "
            "(sync=%s fixed_delta=%.3f no_render=%s).",
            original.synchronous_mode,
            float(original.fixed_delta_seconds or 0.0),
            original.no_rendering_mode,
        )
        return original
    logging.info(
        "Applying world settings (sync=%s fixed_delta=%.3f no_render=%s)",
        settings.synchronous_mode,
        float(settings.fixed_delta_seconds or 0.0),
        settings.no_rendering_mode,
    )
    for attempt in range(3):
        try:
            start = time.monotonic()
            world.apply_settings(settings)
            logging.info("World settings applied in %.2fs", time.monotonic() - start)
            return original
        except RuntimeError as exc:
            logging.warning(
                "apply_settings attempt %d failed: %s", attempt + 1, exc
            )
            try:
                current = world.get_settings()
            except RuntimeError:
                current = None
            if current is not None and _settings_match(current, settings):
                logging.info("World settings already applied after retry.")
                return original
            time.sleep(2.0)
    raise RuntimeError(
        "Failed to apply world settings after retries. "
        "Check server load or restart CARLA server."
    )
    return original


def _settings_match(current: carla.WorldSettings, target: carla.WorldSettings) -> bool:
    return (
        current.synchronous_mode == target.synchronous_mode
        and current.no_rendering_mode == target.no_rendering_mode
        and _delta_match(current.fixed_delta_seconds, target.fixed_delta_seconds)
    )


def _delta_match(current: Optional[float], target: Optional[float]) -> bool:
    if current is None and target is None:
        return True
    if current is None or target is None:
        return False
    return abs(float(current) - float(target)) < 1e-4


def configure_traffic_manager(
    client: carla.Client,
    tm_port: int,
    *,
    sync_mode: bool,
    seed: Optional[int],
) -> carla.TrafficManager:
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(sync_mode)
    if seed is not None:
        tm.set_random_device_seed(int(seed))
    return tm


def setup_carla(
    host: str,
    port: int,
    timeout: float,
    tm_port: int,
    *,
    map_name: Optional[str],
    sync_mode: bool,
    fixed_delta_seconds: float,
    no_rendering_mode: bool,
    seed: Optional[int] = None,
    allow_version_mismatch: bool = False,
) -> CarlaWorldContext:
    client, server_version, client_version = connect_client(
        host,
        port,
        timeout,
        allow_version_mismatch=allow_version_mismatch,
    )
    world = load_world(client, map_name)
    original = configure_world(
        world,
        sync_mode=sync_mode,
        fixed_delta_seconds=fixed_delta_seconds,
        no_rendering_mode=no_rendering_mode,
    )
    tm = configure_traffic_manager(client, tm_port, sync_mode=sync_mode, seed=seed)
    return CarlaWorldContext(
        client=client,
        world=world,
        traffic_manager=tm,
        original_settings=original,
        server_version=server_version,
        client_version=client_version,
    )


def restore_world(ctx: CarlaWorldContext) -> None:
    try:
        world = ctx.client.get_world()
        world.apply_settings(ctx.original_settings)
    except RuntimeError as exc:
        logging.warning("Failed to restore world settings: %s", exc)
    try:
        ctx.traffic_manager.set_synchronous_mode(False)
    except RuntimeError as exc:
        logging.warning("Failed to restore Traffic Manager sync: %s", exc)


def demo_minimal(
    host: str = "127.0.0.1",
    port: int = 2000,
    timeout: float = 5.0,
    tm_port: int = 8000,
    map_name: Optional[str] = None,
    fixed_delta_seconds: float = 0.05,
) -> int:
    logging.basicConfig(level=logging.INFO)
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port,
        map_name=map_name,
        sync_mode=True,
        fixed_delta_seconds=fixed_delta_seconds,
        no_rendering_mode=False,
        seed=42,
    )

    actor_list: list[carla.Actor] = []
    try:
        world = ctx.world
        blueprint_library = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available on this map.")
        spawn_point = random.choice(spawn_points)
        ego_bp = blueprint_library.filter("vehicle.*")[0]
        ego = world.spawn_actor(ego_bp, spawn_point)
        actor_list.append(ego)
        ego.set_autopilot(True, ctx.traffic_manager.get_port())

        for _ in range(200):
            world.tick()

        logging.info("Demo finished.")
        return 0
    finally:
        for actor in actor_list:
            actor.destroy()
        restore_world(ctx)


if __name__ == "__main__":
    raise SystemExit(demo_minimal())
