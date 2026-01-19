"""Configuration models and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


@dataclass
class ClientConfig:
    host: str = "127.0.0.1"
    port: int = 2000
    tm_port: int = 8000
    timeout: float = 10.0
    allow_version_mismatch: bool = False


@dataclass
class CameraConfig:
    width: int = 1280
    height: int = 720
    fov: float = 90.0
    fps: int = 20
    preset: Optional[str] = "driver"
    transform: Dict[str, float] = field(
        default_factory=lambda: {
            "x": 0.8,
            "y": 0.0,
            "z": 1.3,
            "pitch": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
        }
    )


@dataclass
class ScenarioConfig:
    scenario_id: str
    map_name: Optional[str]
    weather: Optional[str]
    seed: int
    duration: float
    fps: int
    fixed_delta_seconds: float
    sync_mode: bool
    no_rendering_mode: bool
    ego_vehicle: str
    camera: CameraConfig
    voice_lead_time_s: float
    robot_precue_lead_s: float
    min_event_time_s: float
    enabled_event_types: Optional[list[str]] = None
    single_event_types: Optional[list[str]] = None
    params: Dict[str, Any] = field(default_factory=dict)


def load_render_presets(path: Path) -> Dict[str, Dict[str, Any]]:
    raw = yaml.safe_load(path.read_text())
    if raw is None:
        return {}
    if isinstance(raw, dict) and isinstance(raw.get("presets"), dict):
        return raw["presets"]
    if isinstance(raw, dict):
        return raw
    raise ValueError(f"Invalid render preset config: {path}")


def apply_render_preset(config: ScenarioConfig, preset: Dict[str, Any]) -> ScenarioConfig:
    if not preset:
        return config
    new_fps = int(preset.get("fps", config.fps))
    new_duration = float(preset.get("duration", config.duration))
    fixed_override = preset.get("fixed_delta_seconds")
    if fixed_override is None:
        new_fixed = config.fixed_delta_seconds
        if new_fps != config.fps:
            new_fixed = 1.0 / float(new_fps)
    else:
        new_fixed = float(fixed_override)

    camera_overrides = preset.get("camera") or {}
    new_camera = replace(
        config.camera,
        width=int(camera_overrides.get("width", config.camera.width)),
        height=int(camera_overrides.get("height", config.camera.height)),
        fov=float(camera_overrides.get("fov", config.camera.fov)),
        fps=int(camera_overrides.get("fps", new_fps)),
        preset=camera_overrides.get("preset", config.camera.preset),
        transform=camera_overrides.get("transform", config.camera.transform),
    )

    params = dict(config.params)
    frame_scale = None
    if new_duration != config.duration or new_fps != config.fps:
        total_old = max(1e-6, float(config.duration) * float(config.fps))
        total_new = float(new_duration) * float(new_fps)
        frame_scale = total_new / total_old

    if frame_scale is not None and bool(preset.get("scale_frames", True)):
        for key, value in list(params.items()):
            if isinstance(value, bool):
                continue
            if isinstance(value, (int, float)) and "frame" in key:
                params[key] = max(1, int(round(float(value) * frame_scale)))

    scenario_overrides = preset.get("scenario_overrides") or {}
    if isinstance(scenario_overrides, dict):
        params.update(scenario_overrides)

    min_event_time_s = config.min_event_time_s
    if frame_scale is not None and bool(preset.get("scale_min_event_time", True)):
        if config.duration > 0:
            min_event_time_s = round(
                config.min_event_time_s / config.duration * new_duration, 3
            )

    return ScenarioConfig(
        scenario_id=config.scenario_id,
        map_name=config.map_name,
        weather=config.weather,
        seed=config.seed,
        duration=new_duration,
        fps=new_fps,
        fixed_delta_seconds=new_fixed,
        sync_mode=bool(preset.get("sync_mode", config.sync_mode)),
        no_rendering_mode=bool(preset.get("no_rendering_mode", config.no_rendering_mode)),
        ego_vehicle=config.ego_vehicle,
        camera=new_camera,
        voice_lead_time_s=config.voice_lead_time_s,
        robot_precue_lead_s=config.robot_precue_lead_s,
        min_event_time_s=min_event_time_s,
        enabled_event_types=config.enabled_event_types,
        single_event_types=config.single_event_types,
        params=params,
    )


def load_client_config(path: Path) -> ClientConfig:
    raw = yaml.safe_load(path.read_text())
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError(f"Invalid client config: {path}")
    return ClientConfig(
        host=str(raw.get("host", "127.0.0.1")),
        port=int(raw.get("port", 2000)),
        tm_port=int(raw.get("tm_port", 8000)),
        timeout=float(raw.get("timeout", 10.0)),
        allow_version_mismatch=bool(raw.get("allow_version_mismatch", False)),
    )


def apply_client_overrides(
    config: ClientConfig,
    *,
    host: Optional[str] = None,
    port: Optional[int] = None,
    tm_port: Optional[int] = None,
    timeout: Optional[float] = None,
    allow_version_mismatch: Optional[bool] = None,
) -> ClientConfig:
    return ClientConfig(
        host=host if host is not None else config.host,
        port=port if port is not None else config.port,
        tm_port=tm_port if tm_port is not None else config.tm_port,
        timeout=timeout if timeout is not None else config.timeout,
        allow_version_mismatch=(
            allow_version_mismatch
            if allow_version_mismatch is not None
            else config.allow_version_mismatch
        ),
    )


def load_scenario_config(path: Path) -> ScenarioConfig:
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f"Invalid scenario config: {path}")

    camera_raw = raw.get("camera", {}) or {}
    camera = CameraConfig(
        width=int(camera_raw.get("width", 1280)),
        height=int(camera_raw.get("height", 720)),
        fov=float(camera_raw.get("fov", 90.0)),
        fps=int(camera_raw.get("fps", raw.get("fps", 20))),
        preset=camera_raw.get("preset", "driver"),
        transform=camera_raw.get(
            "transform",
            {
                "x": 0.8,
                "y": 0.0,
                "z": 1.3,
                "pitch": 0.0,
                "yaw": 0.0,
                "roll": 0.0,
            },
        ),
    )

    events_raw = raw.get("events", {}) or {}
    enabled_event_types = _parse_event_list(events_raw.get("enabled_types"))
    single_event_types = _parse_event_list(events_raw.get("single_event_types"))

    return ScenarioConfig(
        scenario_id=str(raw.get("id", "unknown")),
        map_name=raw.get("map"),
        weather=raw.get("weather"),
        seed=int(raw.get("seed", 0)),
        duration=float(raw.get("duration", 20.0)),
        fps=int(raw.get("fps", camera.fps)),
        fixed_delta_seconds=float(raw.get("fixed_delta_seconds", 0.05)),
        sync_mode=bool(raw.get("sync_mode", True)),
        no_rendering_mode=bool(raw.get("no_rendering_mode", False)),
        ego_vehicle=str(raw.get("ego_vehicle", "vehicle.tesla.model3")),
        camera=camera,
        voice_lead_time_s=float(events_raw.get("voice_lead_time_s", 3.0)),
        robot_precue_lead_s=float(events_raw.get("robot_precue_lead_s", 0.5)),
        min_event_time_s=float(events_raw.get("min_event_time_s", 0.0)),
        enabled_event_types=enabled_event_types,
        single_event_types=single_event_types,
        params=raw.get("scenario", {}) or {},
    )


def _parse_event_list(value: Any) -> Optional[list[str]]:
    if value is None:
        return None
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple, set)):
        return [str(item) for item in value]
    return None
