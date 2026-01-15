"""Scenario registry."""

from __future__ import annotations

import logging
import random
from pathlib import Path
from typing import Optional

import carla

from ..config import ScenarioConfig
from .highway_merge import HighwayMergeScenario
from .lane_change_cut_in import LaneChangeCutInScenario
from .pedestrian_emerge import PedestrianEmergeScenario
from .red_light_conflict import RedLightConflictScenario
from .unprotected_left_turn import UnprotectedLeftTurnScenario
from .yield_to_emergency import YieldToEmergencyScenario
from .base import ScenarioContext

# Base path for scenario packages
SCENARIOS_DIR = Path(__file__).parent

_SCENARIOS = {
    "highway_merge": HighwayMergeScenario,
    "lane_change_cut_in": LaneChangeCutInScenario,
    "unprotected_left_turn": UnprotectedLeftTurnScenario,
    "red_light_conflict": RedLightConflictScenario,
    "yield_to_emergency": YieldToEmergencyScenario,
    "pedestrian_emerge": PedestrianEmergeScenario,
}


def get_scenario_ids() -> list[str]:
    """Return list of registered scenario IDs."""
    return list(_SCENARIOS.keys())


def get_scenario_config_path(
    scenario_id: str,
    variant: str = "config_30s",
) -> Optional[Path]:
    """Get the path to a scenario's config file.

    Args:
        scenario_id: The scenario identifier (e.g., "highway_merge")
        variant: Config variant name without extension (default: "config_30s")
                 Use "config" for short test version, "config_30s" for full version.

    Returns:
        Path to the config YAML file, or None if not found.
    """
    if scenario_id not in _SCENARIOS:
        return None
    scenario_dir = SCENARIOS_DIR / scenario_id
    config_path = scenario_dir / f"{variant}.yaml"
    if config_path.exists():
        return config_path
    # Fallback to default config
    default_path = scenario_dir / "config.yaml"
    if default_path.exists():
        return default_path
    return None


def get_scenario_readme_path(scenario_id: str) -> Optional[Path]:
    """Get the path to a scenario's README documentation.

    Args:
        scenario_id: The scenario identifier (e.g., "highway_merge")

    Returns:
        Path to the README.md file, or None if not found.
    """
    if scenario_id not in _SCENARIOS:
        return None
    readme_path = SCENARIOS_DIR / scenario_id / "README.md"
    if readme_path.exists():
        return readme_path
    return None


def build_scenario(
    world: carla.World,
    tm: carla.TrafficManager,
    config: ScenarioConfig,
) -> ScenarioContext:
    """Build and initialize a scenario.

    Args:
        world: CARLA world instance
        tm: Traffic manager instance
        config: Scenario configuration

    Returns:
        ScenarioContext with initialized scenario

    Raises:
        ValueError: If scenario_id is not recognized
    """
    scenario_cls = _SCENARIOS.get(config.scenario_id)
    if scenario_cls is None:
        raise ValueError(f"Unknown scenario id: {config.scenario_id}")
    rng = random.Random(config.seed)
    scenario = scenario_cls(config)
    logging.info("Scenario build start: %s", config.scenario_id)
    ctx = scenario.build(world, tm, rng)
    logging.info("Scenario build complete: %s", config.scenario_id)
    return ctx
