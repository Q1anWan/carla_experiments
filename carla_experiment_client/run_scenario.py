"""Run a CARLA scenario and produce a master video + event log."""

from __future__ import annotations

import argparse
import logging
import os
import shutil
from pathlib import Path

import carla

from .carla_client import restore_world, setup_carla
from .config import (
    ScenarioConfig,
    apply_client_overrides,
    apply_render_preset,
    load_client_config,
    load_render_presets,
    load_scenario_config,
)
from .events.extractor import EventExtractor
from .scenarios.registry import build_scenario
from .sensors.camera_recorder import record_video
from .utils import ensure_dir, utc_timestamp, write_json
from .video import encode_frames_to_mp4
from .weather import apply_weather


def _attach_file_logger(out_dir: Path) -> None:
    logger = logging.getLogger()
    log_path = out_dir / "run.log"
    for handler in logger.handlers:
        if isinstance(handler, logging.FileHandler) and Path(handler.baseFilename) == log_path:
            return
    handler = logging.FileHandler(log_path, mode="a")
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
    logger.addHandler(handler)


def _write_metadata(
    out_dir: Path,
    config: ScenarioConfig,
    host: str,
    port: int,
    tm_port: int,
    server_version: str,
    client_version: str,
    allow_version_mismatch: bool,
    render_preset: str | None,
) -> None:
    payload = {
        "scenario_id": config.scenario_id,
        "scenario_config": "scenario.yaml",
        "map": config.map_name,
        "seed": config.seed,
        "fps": config.fps,
        "duration": config.duration,
        "fixed_delta_seconds": config.fixed_delta_seconds,
        "sync_mode": config.sync_mode,
        "no_rendering_mode": config.no_rendering_mode,
        "voice_lead_time_s": config.voice_lead_time_s,
        "robot_precue_lead_s": config.robot_precue_lead_s,
        "min_event_time_s": config.min_event_time_s,
        "host": host,
        "port": port,
        "traffic_manager_port": tm_port,
        "server_version": server_version,
        "client_version": client_version,
        "allow_version_mismatch": allow_version_mismatch,
        "render_preset": render_preset,
    }
    write_json(out_dir / "run_metadata.json", payload)


def run_scenario(
    scenario_path: Path,
    out_dir: Path,
    *,
    host: str,
    port: int,
    tm_port: int,
    timeout: float,
    allow_version_mismatch: bool,
    render_preset: str | None = None,
    render_presets_path: Path | None = None,
) -> int:
    config = load_scenario_config(scenario_path)
    if render_preset:
        if render_presets_path is None:
            raise ValueError("render_presets_path is required when render_preset is set")
        presets = load_render_presets(render_presets_path)
        if render_preset not in presets:
            raise ValueError(f"Unknown render preset: {render_preset}")
        config = apply_render_preset(config, presets[render_preset])
        logging.info("Applied render preset: %s", render_preset)
    ensure_dir(out_dir)
    shutil.copy2(scenario_path, out_dir / "scenario.yaml")
    _attach_file_logger(out_dir)
    logging.info("Run pid=%s", os.getpid())
    logging.info("Scenario config: %s", scenario_path)
    logging.info("Output dir: %s", out_dir)

    logging.info("Connecting to CARLA %s:%s", host, port)
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
        apply_weather(ctx.world, config.weather)
        logging.info("Building scenario %s", config.scenario_id)
        scenario_ctx = build_scenario(ctx.world, ctx.traffic_manager, config)
        logging.info("Scenario built: %s", config.scenario_id)
        extractor = EventExtractor(
            world=ctx.world,
            ego_vehicle=scenario_ctx.ego_vehicle,
            map_obj=ctx.world.get_map(),
            fps=config.fps,
            voice_lead_time_s=config.voice_lead_time_s,
            robot_precue_lead_s=config.robot_precue_lead_s,
            min_event_time_s=config.min_event_time_s,
        )

        def on_tick(snapshot: carla.WorldSnapshot, _: carla.Image, index: int) -> None:
            scenario_ctx.on_tick(index)
            extractor.tick(snapshot, index)

        prewarm_seconds = config.params.get("prewarm_seconds")
        if prewarm_seconds is not None:
            prewarm_frames = int(round(float(prewarm_seconds) * config.fps))
        else:
            prewarm_frames = int(
                config.params.get("prewarm_frames", max(1, int(config.fps * 0.5)))
            )
        if prewarm_frames > 0:
            logging.info("Prewarming %d frames before recording", prewarm_frames)
            for _ in range(prewarm_frames):
                ctx.world.tick()

        total_frames = int(scenario_ctx.duration * scenario_ctx.fps)
        logging.info("Recording %d frames at %d fps", total_frames, scenario_ctx.fps)
        frames_dir = record_video(scenario_ctx, out_dir, on_tick=on_tick)
        events = extractor.finalize()

        write_json(out_dir / "events.json", {"events": events})
        _write_metadata(
            out_dir,
            config,
            host,
            port,
            tm_port,
            ctx.server_version,
            ctx.client_version,
            allow_version_mismatch,
            render_preset,
        )

        logging.info("Encoding master_video.mp4")
        encode_frames_to_mp4(frames_dir, out_dir / "master_video.mp4", config.fps)
        logging.info("Run complete: %s", out_dir)
        return 0
    finally:
        if scenario_ctx is not None:
            for actor in scenario_ctx.actors:
                try:
                    actor.destroy()
                except RuntimeError:
                    logging.warning("Actor cleanup skipped (already destroyed).")
        restore_world(ctx)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a CARLA scenario.")
    parser.add_argument("--scenario", required=True, type=Path, help="Scenario YAML")
    parser.add_argument("--out", type=Path, help="Output directory")
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
        "--render-preset",
        help="Render preset name (e.g., fast, final)",
    )
    parser.add_argument(
        "--render-presets",
        type=Path,
        default=Path("configs/render_presets.yaml"),
        help="YAML file with render presets",
    )
    parser.add_argument(
        "--allow-version-mismatch",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Allow client/server version mismatch",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    out_dir = args.out
    if out_dir is None:
        out_dir = Path("runs") / utc_timestamp()

    client_config = load_client_config(args.client_config)
    client_config = apply_client_overrides(
        client_config,
        host=args.host,
        port=args.port,
        tm_port=args.tm_port,
        timeout=args.timeout,
        allow_version_mismatch=args.allow_version_mismatch,
    )

    return run_scenario(
        args.scenario,
        out_dir,
        host=client_config.host,
        port=client_config.port,
        tm_port=client_config.tm_port,
        timeout=client_config.timeout,
        allow_version_mismatch=client_config.allow_version_mismatch,
        render_preset=args.render_preset,
        render_presets_path=args.render_presets if args.render_preset else None,
    )


if __name__ == "__main__":
    raise SystemExit(main())
