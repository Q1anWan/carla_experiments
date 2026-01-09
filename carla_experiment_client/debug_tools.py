"""Step-by-step debug tools for the CARLA experiment client."""

from __future__ import annotations

import argparse
import json
import logging
import random
from pathlib import Path
from typing import Optional

import carla

from .carla_client import restore_world, setup_carla
from .config import (
    CameraConfig,
    apply_client_overrides,
    load_client_config,
    load_scenario_config,
)
from .events.extractor import EventExtractor
from .render_variants import render_variants as render_variants_impl
from .sensors.camera_recorder import CameraRecorder
from .scenarios.registry import build_scenario
from .utils import ensure_dir, write_json
from .video import encode_frames_to_mp4
from .audio.mux import render_narration
from .weather import apply_weather


def _spawn_ego(
    world: carla.World,
    tm: carla.TrafficManager,
    *,
    blueprint_filter: str,
    seed: int = 0,
) -> carla.Vehicle:
    rng = random.Random(seed)
    blueprint_library = world.get_blueprint_library()
    blueprints = blueprint_library.filter(blueprint_filter)
    if not blueprints:
        raise RuntimeError(f"No blueprints for '{blueprint_filter}'")
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available")
    transform = rng.choice(spawn_points)
    ego = world.spawn_actor(rng.choice(blueprints), transform)
    if ego is None:
        raise RuntimeError("Failed to spawn ego vehicle")
    ego.set_autopilot(True, tm.get_port())
    return ego


def check_connection(host: str, port: int, tm_port: int, timeout: float, *, allow_version_mismatch: bool) -> int:
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port=tm_port,
        map_name=None,
        sync_mode=False,
        fixed_delta_seconds=0.05,
        no_rendering_mode=False,
        allow_version_mismatch=allow_version_mismatch,
    )
    try:
        world = ctx.world
        logging.info("Server version: %s", ctx.server_version)
        logging.info("Client version: %s", ctx.client_version)
        logging.info("Map: %s", world.get_map().name)
        return 0
    finally:
        restore_world(ctx)


def tick_sync(
    host: str,
    port: int,
    tm_port: int,
    timeout: float,
    *,
    frames: int,
    fixed_delta_seconds: float,
    allow_version_mismatch: bool,
) -> int:
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port,
        map_name=None,
        sync_mode=True,
        fixed_delta_seconds=fixed_delta_seconds,
        no_rendering_mode=False,
        allow_version_mismatch=allow_version_mismatch,
    )
    try:
        for _ in range(frames):
            ctx.world.tick()
        logging.info("Ticked %d frames in sync mode", frames)
        return 0
    finally:
        restore_world(ctx)


def debug_camera(
    host: str,
    port: int,
    tm_port: int,
    timeout: float,
    *,
    out_dir: Path,
    frames: int,
    camera: CameraConfig,
    encode: bool,
    allow_version_mismatch: bool,
    weather: Optional[str],
    ego_blueprint: str,
    cleanup: bool,
) -> int:
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port,
        map_name=None,
        sync_mode=True,
        fixed_delta_seconds=1.0 / float(camera.fps),
        no_rendering_mode=False,
        allow_version_mismatch=allow_version_mismatch,
    )
    actors: list[carla.Actor] = []
    recorder: Optional[CameraRecorder] = None
    try:
        apply_weather(ctx.world, weather)
        ego = _spawn_ego(ctx.world, ctx.traffic_manager, blueprint_filter=ego_blueprint)
        actors.append(ego)
        recorder = CameraRecorder(ctx.world, ego, camera)
        recorder.start()
        ensure_dir(out_dir)
        frames_dir = out_dir / "frames"
        recorder.record_frames(frames_dir, frames)
        if encode:
            try:
                encode_frames_to_mp4(frames_dir, out_dir / "camera_debug.mp4", camera.fps)
            except RuntimeError as exc:
                logging.warning("Encode skipped: %s", exc)
        logging.info("Camera debug saved to %s", out_dir)
        return 0
    finally:
        if cleanup:
            if recorder is not None:
                recorder.stop()
            for actor in actors:
                if actor is None:
                    continue
                try:
                    actor.destroy()
                except RuntimeError:
                    pass
            actors.clear()
            restore_world(ctx)
        else:
            logging.warning("Cleanup skipped: actors may remain in the world.")


def debug_scenario(
    scenario_path: Path,
    host: str,
    port: int,
    tm_port: int,
    timeout: float,
    *,
    frames: int,
    allow_version_mismatch: bool,
    weather: Optional[str],
) -> int:
    config = load_scenario_config(scenario_path)
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port,
        map_name=config.map_name,
        sync_mode=config.sync_mode,
        fixed_delta_seconds=config.fixed_delta_seconds,
        no_rendering_mode=config.no_rendering_mode,
        seed=config.seed,
        allow_version_mismatch=allow_version_mismatch,
    )
    scenario_ctx = None
    try:
        apply_weather(ctx.world, weather or config.weather)
        scenario_ctx = build_scenario(ctx.world, ctx.traffic_manager, config)
        for frame in range(frames):
            ctx.world.tick()
            scenario_ctx.on_tick(frame)
        logging.info("Scenario build OK: %s", config.scenario_id)
        return 0
    finally:
        if scenario_ctx is not None:
            for actor in scenario_ctx.actors:
                actor.destroy()
        restore_world(ctx)


def debug_events(
    scenario_path: Path,
    host: str,
    port: int,
    tm_port: int,
    timeout: float,
    *,
    frames: int,
    out_path: Path,
    allow_version_mismatch: bool,
    weather: Optional[str],
) -> int:
    config = load_scenario_config(scenario_path)
    ctx = setup_carla(
        host,
        port,
        timeout,
        tm_port,
        map_name=config.map_name,
        sync_mode=config.sync_mode,
        fixed_delta_seconds=config.fixed_delta_seconds,
        no_rendering_mode=config.no_rendering_mode,
        seed=config.seed,
        allow_version_mismatch=allow_version_mismatch,
    )
    scenario_ctx = None
    try:
        apply_weather(ctx.world, weather or config.weather)
        scenario_ctx = build_scenario(ctx.world, ctx.traffic_manager, config)
        extractor = EventExtractor(
            world=ctx.world,
            ego_vehicle=scenario_ctx.ego_vehicle,
            map_obj=ctx.world.get_map(),
            fps=config.fps,
        )
        for frame in range(frames):
            ctx.world.tick()
            snapshot = ctx.world.get_snapshot()
            scenario_ctx.on_tick(frame)
            extractor.tick(snapshot, frame)
        events = extractor.finalize()
        write_json(out_path, {"events": events})
        logging.info("Event debug written to %s", out_path)
        return 0
    finally:
        if scenario_ctx is not None:
            for actor in scenario_ctx.actors:
                actor.destroy()
        restore_world(ctx)


def debug_audio(events_path: Path, *, voice_level: int, out_path: Path) -> int:
    events = json.loads(events_path.read_text()).get("events", [])
    ensure_dir(out_path.parent)
    rendered = render_narration(events, out_path, voice_level=voice_level)
    if rendered is None:
        logging.info("No audio generated (voice_level=0 or no events)")
        return 0
    logging.info("Narration saved to %s", out_path)
    return 0


def debug_variants(run_dir: Path) -> int:
    return render_variants_impl(run_dir)


def _add_client_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--client-config",
        type=Path,
        default=Path("configs/client.yaml"),
        help="Client YAML (host/port/tm_port/timeout)",
    )
    parser.add_argument("--host", help="CARLA host")
    parser.add_argument("--port", type=int, help="CARLA port")
    parser.add_argument("--tm-port", type=int, help="Traffic Manager port")
    parser.add_argument("--timeout", type=float, help="Client timeout")
    parser.add_argument(
        "--allow-version-mismatch",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Allow client/server version mismatch",
    )


def _resolve_client(args: argparse.Namespace) -> tuple[str, int, int, float, bool]:
    client_config = load_client_config(args.client_config)
    client_config = apply_client_overrides(
        client_config,
        host=args.host,
        port=args.port,
        tm_port=args.tm_port,
        timeout=args.timeout,
        allow_version_mismatch=args.allow_version_mismatch,
    )
    return (
        client_config.host,
        client_config.port,
        client_config.tm_port,
        client_config.timeout,
        client_config.allow_version_mismatch,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Debug tools for CARLA experiment client")
    subparsers = parser.add_subparsers(dest="command", required=True)

    conn = subparsers.add_parser("check_connection", help="Validate server reachability")
    _add_client_args(conn)

    tick = subparsers.add_parser("tick_sync", help="Tick a few frames in sync mode")
    _add_client_args(tick)
    tick.add_argument("--frames", type=int, default=50)
    tick.add_argument("--fixed-delta", type=float, default=0.05)

    cam = subparsers.add_parser("camera", help="Record a few camera frames")
    _add_client_args(cam)
    cam.add_argument("--out", type=Path, required=True)
    cam.add_argument("--frames", type=int, default=50)
    cam.add_argument("--width", type=int, default=1280)
    cam.add_argument("--height", type=int, default=720)
    cam.add_argument("--fov", type=float, default=90.0)
    cam.add_argument("--fps", type=int, default=20)
    cam.add_argument("--encode", action="store_true")
    cam.add_argument("--weather", help="Weather preset (e.g., ClearNoon)")
    cam.add_argument("--preset", default="driver", help="Camera preset (driver/hood/third_person/custom)")
    cam.add_argument(
        "--ego-blueprint",
        default="vehicle.tesla.model3",
        help="Ego vehicle blueprint for camera debug",
    )
    cam.add_argument(
        "--cleanup",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Cleanup actors and restore world settings",
    )

    scn = subparsers.add_parser("scenario", help="Build scenario and tick a few frames")
    scn.add_argument("--scenario", type=Path, required=True)
    _add_client_args(scn)
    scn.add_argument("--frames", type=int, default=60)
    scn.add_argument("--weather", help="Weather preset (e.g., ClearNoon)")

    ev = subparsers.add_parser("events", help="Extract events only (no video)")
    ev.add_argument("--scenario", type=Path, required=True)
    _add_client_args(ev)
    ev.add_argument("--frames", type=int, default=120)
    ev.add_argument("--out", type=Path, required=True)
    ev.add_argument("--weather", help="Weather preset (e.g., ClearNoon)")

    audio = subparsers.add_parser("audio", help="Render narration wav from events.json")
    audio.add_argument("--events", type=Path, required=True)
    audio.add_argument("--voice-level", type=int, default=2)
    audio.add_argument("--out", type=Path, required=True)

    variants = subparsers.add_parser("variants", help="Render all stimulus variants")
    variants.add_argument("--run-dir", type=Path, required=True)

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    if args.command == "check_connection":
        host, port, tm_port, timeout, allow_version_mismatch = _resolve_client(args)
        return check_connection(
            host,
            port,
            tm_port,
            timeout,
            allow_version_mismatch=allow_version_mismatch,
        )
    if args.command == "tick_sync":
        host, port, tm_port, timeout, allow_version_mismatch = _resolve_client(args)
        return tick_sync(
            host,
            port,
            tm_port,
            timeout,
            frames=args.frames,
            fixed_delta_seconds=args.fixed_delta,
            allow_version_mismatch=allow_version_mismatch,
        )
    if args.command == "camera":
        host, port, tm_port, timeout, allow_version_mismatch = _resolve_client(args)
        camera = CameraConfig(
            width=args.width,
            height=args.height,
            fov=args.fov,
            fps=args.fps,
            preset=args.preset,
        )
        return debug_camera(
            host,
            port,
            tm_port,
            timeout,
            out_dir=args.out,
            frames=args.frames,
            camera=camera,
            encode=args.encode,
            allow_version_mismatch=allow_version_mismatch,
            weather=args.weather,
            ego_blueprint=args.ego_blueprint,
            cleanup=args.cleanup,
        )
    if args.command == "scenario":
        host, port, tm_port, timeout, allow_version_mismatch = _resolve_client(args)
        return debug_scenario(
            args.scenario,
            host,
            port,
            tm_port,
            timeout,
            frames=args.frames,
            allow_version_mismatch=allow_version_mismatch,
            weather=args.weather,
        )
    if args.command == "events":
        host, port, tm_port, timeout, allow_version_mismatch = _resolve_client(args)
        return debug_events(
            args.scenario,
            host,
            port,
            tm_port,
            timeout,
            frames=args.frames,
            out_path=args.out,
            allow_version_mismatch=allow_version_mismatch,
            weather=args.weather,
        )
    if args.command == "audio":
        return debug_audio(args.events, voice_level=args.voice_level, out_path=args.out)
    if args.command == "variants":
        return debug_variants(args.run_dir)

    raise RuntimeError("Unknown command")


if __name__ == "__main__":
    raise SystemExit(main())
