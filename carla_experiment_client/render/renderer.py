"""Render plan.json by replaying trajectories in CARLA."""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, List

import yaml

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..carla_client import configure_world, connect_client, load_world
from ..config import CameraConfig
from ..sensors.camera_recorder import CameraRecorder
from ..telemetry import TelemetryRecorder
from ..utils import check_timeout, ensure_dir, write_json
from ..video import encode_frames_to_mp4
from ..weather import apply_weather
from ..planning.trajectory_schema import Plan, load_plan
from .replay_controller import build_follower


def _load_globals(path: Path) -> Dict[str, Any]:
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _build_camera_config(
    globals_cfg: Dict[str, Any],
    fps: int,
    *,
    image_size: tuple[int, int] | None = None,
) -> CameraConfig:
    recording = globals_cfg.get("recording", {})
    if image_size is None:
        width, height = recording.get("image_size", [1280, 720])
    else:
        width, height = image_size
    return CameraConfig(
        width=int(width),
        height=int(height),
        fov=float(recording.get("fov", 90)),
        fps=int(fps),
        preset="driver",
    )


def _spawn_actor(
    world: "carla.World",
    blueprint_id: str,
    role_name: str,
    transform: "carla.Transform",
) -> "carla.Actor":
    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find(blueprint_id) if blueprint_id else None
    if blueprint is None:
        candidates = blueprint_library.filter("vehicle.*")
        blueprint = candidates[0]
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role_name)
    actor = world.try_spawn_actor(blueprint, transform)
    if actor is None:
        raise RuntimeError(f"Failed to spawn actor {role_name} using {blueprint_id}")
    return actor


def _first_transform(
    plan: Plan,
    actor_plan: Any,
    map_obj: "carla.Map",
) -> "carla.Transform":
    point = actor_plan.trajectory[0]
    loc = carla.Location(x=point.x, y=point.y, z=0.0)
    wp = map_obj.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    z = wp.transform.location.z if wp else 0.0
    return carla.Transform(carla.Location(point.x, point.y, z + 0.2), carla.Rotation(yaw=point.yaw))


def _build_events(plan: Plan, globals_cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    events = []
    voice_lead = float(globals_cfg.get("events", {}).get("voice_lead_time_s", 3.0))
    robot_lead = float(globals_cfg.get("events", {}).get("robot_precue_lead_s", 0.5))
    ego = next((a for a in plan.actors if a.role == "ego"), None)
    for idx, event in enumerate(plan.events_plan):
        t_event = float(event.t_event)
        t_voice = float(event.t_voice_start) if event.t_voice_start is not None else max(0.0, t_event - voice_lead)
        t_robot = float(event.t_robot_precue) if event.t_robot_precue is not None else max(0.0, t_voice - robot_lead)
        snapshot = {}
        if ego:
            frame_idx = min(len(ego.trajectory) - 1, max(0, int(t_event / plan.dt)))
            point = ego.trajectory[frame_idx]
            snapshot = {
                "ego": {
                    "x": point.x,
                    "y": point.y,
                    "yaw": point.yaw,
                    "v": point.v,
                }
            }
        events.append(
            {
                "t_event": round(t_event, 3),
                "t_voice_start": round(t_voice, 3),
                "t_robot_precue": round(t_robot, 3),
                "event_type": event.event_type,
                "expected_action": event.expected_action,
                "audio_id": event.audio_id or f"{event.event_type}_{idx:02d}",
                "state_snapshot": snapshot,
            }
        )
    return events


def render_plan(
    plan_path: Path,
    globals_path: Path,
    out_dir: Path,
    *,
    max_seconds: float | None = None,
    max_tick_seconds: float = 10.0,
    image_size: tuple[int, int] | None = None,
    encode_timeout_s: float | None = None,
) -> None:
    if carla is None:
        raise RuntimeError("CARLA Python API is not available.")
    start_time = time.monotonic()
    globals_cfg = _load_globals(globals_path)
    plan = load_plan(plan_path)
    logging.info("Rendering plan %s (town=%s)", plan.episode_id, plan.town)

    carla_cfg = globals_cfg.get("carla", {})
    client, server_version, client_version = connect_client(
        str(carla_cfg.get("host", "127.0.0.1")),
        int(carla_cfg.get("port", 2000)),
        float(carla_cfg.get("timeout_s", 10.0)),
        allow_version_mismatch=bool(carla_cfg.get("allow_version_mismatch", True)),
    )
    world = load_world(client, plan.town)
    original_settings = configure_world(
        world,
        sync_mode=bool(carla_cfg.get("synchronous", True)),
        fixed_delta_seconds=float(carla_cfg.get("fixed_delta_seconds", plan.dt)),
        no_rendering_mode=False,
    )
    check_timeout(start_time, max_seconds, "renderer")

    apply_weather(world, globals_cfg.get("weather", {}).get("preset"))

    map_obj = world.get_map()
    actors = []
    followers = []
    ego_vehicle = None
    recorder = None
    try:
        for actor_plan in plan.actors:
            transform = _first_transform(plan, actor_plan, map_obj)
            actor = _spawn_actor(world, actor_plan.blueprint, actor_plan.role, transform)
            actors.append(actor)
            if actor_plan.role == "ego":
                ego_vehicle = actor
            lane_type = carla.LaneType.Driving
            if actor_plan.kind.lower() in {"walker", "vru", "pedestrian"}:
                lane_type = carla.LaneType.Sidewalk
            follower = build_follower(
                actor=actor,
                trajectory=actor_plan.trajectory,
                map_obj=map_obj,
                lane_type=lane_type,
                controller=actor_plan.controller,
            )
            followers.append(follower)

        if ego_vehicle is None:
            raise RuntimeError("Plan does not define an ego actor.")

        fps = int(round(1.0 / plan.dt))
        camera_config = _build_camera_config(globals_cfg, fps, image_size=image_size)
        recorder = CameraRecorder(world=world, ego_vehicle=ego_vehicle, config=camera_config)
        recorder.start()
        frames_dir = out_dir / "frames"
        ensure_dir(frames_dir)

        telemetry = TelemetryRecorder(world=world, ego_vehicle=ego_vehicle, fps=fps, tracked_actors=actors)

        total_frames = int(plan.duration / plan.dt)
        for frame_idx in range(total_frames):
            check_timeout(start_time, max_seconds, "renderer")
            for follower in followers:
                follower.apply(frame_idx)
            tick_start = time.monotonic()
            frame_id = world.tick()
            tick_duration = time.monotonic() - tick_start
            if tick_duration > 1.0:
                logging.warning("World tick took %.2fs", tick_duration)
            if tick_duration > max_tick_seconds:
                raise RuntimeError(
                    f"World tick timeout: {tick_duration:.2f}s exceeds {max_tick_seconds:.2f}s"
                )
            snapshot = world.get_snapshot()
            image = recorder._get_image(frame_id, timeout=5.0)
            image.save_to_disk(str(frames_dir / f"{frame_idx:06d}.png"))
            telemetry.tick(snapshot, frame_idx)
            if frame_idx == 0 or (frame_idx + 1) % 50 == 0 or frame_idx + 1 == total_frames:
                logging.info("Recorded frame %d/%d", frame_idx + 1, total_frames)

        telemetry.save(out_dir)

        events = _build_events(plan, globals_cfg)
        write_json(out_dir / "events.json", {"events": events})

        write_json(
            out_dir / "run_metadata.json",
            {
                "episode_id": plan.episode_id,
                "town": plan.town,
                "seed": plan.seed,
                "dt": plan.dt,
                "duration": plan.duration,
                "fps": fps,
                "server_version": server_version,
                "client_version": client_version,
            },
        )

        logging.info("Encoding master_video.mp4")
        encode_frames_to_mp4(
            frames_dir,
            out_dir / "master_video.mp4",
            fps,
            timeout_s=encode_timeout_s,
        )
    finally:
        if recorder is not None:
            recorder.stop()
        for actor in actors:
            try:
                actor.destroy()
            except RuntimeError:
                pass
        world.apply_settings(original_settings)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a plan.json in CARLA.")
    parser.add_argument("--plan", required=True, help="Path to plan.json")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--out", default=".", help="Output directory")
    parser.add_argument("--max-seconds", type=float, default=1200.0, help="Maximum runtime before abort")
    parser.add_argument("--max-tick-seconds", type=float, default=10.0, help="Max per-tick duration")
    parser.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    parser.add_argument("--encode-timeout", type=float, default=300.0)
    parser.add_argument("--quick", action="store_true", help="Use low resolution (640x360)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    image_size = tuple(args.image_size) if args.image_size else None
    if args.quick:
        image_size = (640, 360)
    render_plan(
        Path(args.plan),
        Path(args.globals),
        Path(args.out),
        max_seconds=args.max_seconds,
        max_tick_seconds=args.max_tick_seconds,
        image_size=image_size,
        encode_timeout_s=args.encode_timeout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
