"""Weather helpers."""

from __future__ import annotations

from typing import Optional

import carla


def apply_weather(world: carla.World, weather_name: Optional[str]) -> None:
    if not weather_name:
        return
    try:
        weather = getattr(carla.WeatherParameters, weather_name)
    except AttributeError as exc:
        raise ValueError(f"Unknown weather preset: {weather_name}") from exc
    world.set_weather(weather)
