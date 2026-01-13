"""Scenario registry."""

from __future__ import annotations

import logging
import random

import carla

from ..config import ScenarioConfig
from .highway_merge import HighwayMergeScenario
from .lane_change_cut_in import LaneChangeCutInScenario
from .pedestrian_emerge import PedestrianEmergeScenario
from .red_light_conflict import RedLightConflictScenario
from .unprotected_left_turn import UnprotectedLeftTurnScenario
from .yield_to_emergency import YieldToEmergencyScenario
from .base import ScenarioContext


_SCENARIOS = {
    "highway_merge": HighwayMergeScenario,
    "lane_change_cut_in": LaneChangeCutInScenario,
    "unprotected_left_turn": UnprotectedLeftTurnScenario,
    "red_light_conflict": RedLightConflictScenario,
    "yield_to_emergency": YieldToEmergencyScenario,
    "pedestrian_emerge": PedestrianEmergeScenario,
}


def build_scenario(
    world: carla.World,
    tm: carla.TrafficManager,
    config: ScenarioConfig,
) -> ScenarioContext:
    scenario_cls = _SCENARIOS.get(config.scenario_id)
    if scenario_cls is None:
        raise ValueError(f"Unknown scenario id: {config.scenario_id}")
    rng = random.Random(config.seed)
    scenario = scenario_cls(config)
    logging.info("Scenario build start: %s", config.scenario_id)
    ctx = scenario.build(world, tm, rng)
    logging.info("Scenario build complete: %s", config.scenario_id)
    return ctx
