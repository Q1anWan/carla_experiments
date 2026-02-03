"""Event DSL parser: YAML dict → typed dataclasses for the Director."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class TriggerCondition:
    """Single trigger condition: metric op value."""

    metric: str  # "gap_m", "ttc_s", "elapsed_s", "lateral_offset_m", etc.
    op: str  # "<=", ">=", "==", "<", ">"
    value: float


@dataclass
class TriggerSpec:
    """Trigger specification: all conditions must be true (AND)."""

    all: List[TriggerCondition] = field(default_factory=list)


@dataclass
class TMAction:
    """Traffic Manager parameter changes."""

    auto_lane_change: Optional[bool] = None
    speed_diff_pct: Optional[float] = None
    distance_to_leading_vehicle_m: Optional[float] = None
    ignore_lights_pct: Optional[float] = None
    ignore_vehicles_pct: Optional[float] = None
    force_lane_change: Optional[str] = None  # "left" | "right"


@dataclass
class OverrideFallback:
    """Short override controller configuration."""

    controller_override_window_s: float = 2.0
    lane_change_profile: str = "quintic"  # "quintic" | "cubic"
    target_speed_mps: Optional[float] = None


@dataclass
class EventSpec:
    """Full event specification from YAML DSL."""

    id: str
    type: str  # "cut_in", "merge", "brake", "yield", etc.
    actor: str  # tag name in ScenarioContext.tags → actor lookup
    trigger: TriggerSpec
    action_tm: Optional[TMAction] = None  # TM params before execute
    action_execute: Optional[TMAction] = None  # force_lane_change, etc.
    action_post: Optional[TMAction] = None  # restore after event
    fallback: Optional[OverrideFallback] = None
    # Quality gate thresholds
    gate_min_ttc_s: Optional[float] = None
    gate_max_gap_m: Optional[float] = None


def _parse_trigger_condition(raw: dict) -> TriggerCondition:
    """Parse a single trigger condition dict."""
    return TriggerCondition(
        metric=str(raw["metric"]),
        op=str(raw["op"]),
        value=float(raw["value"]),
    )


def _parse_trigger(raw: dict) -> TriggerSpec:
    """Parse trigger specification."""
    conditions = []
    for item in raw.get("all", []):
        conditions.append(_parse_trigger_condition(item))
    return TriggerSpec(all=conditions)


def _parse_tm_action(raw: Optional[dict]) -> Optional[TMAction]:
    """Parse TM action dict."""
    if raw is None:
        return None
    return TMAction(
        auto_lane_change=raw.get("auto_lane_change"),
        speed_diff_pct=_opt_float(raw.get("speed_diff_pct")),
        distance_to_leading_vehicle_m=_opt_float(raw.get("distance_to_leading_vehicle_m")),
        ignore_lights_pct=_opt_float(raw.get("ignore_lights_pct")),
        ignore_vehicles_pct=_opt_float(raw.get("ignore_vehicles_pct")),
        force_lane_change=raw.get("force_lane_change"),
    )


def _parse_fallback(raw: Optional[dict]) -> Optional[OverrideFallback]:
    """Parse fallback override configuration."""
    if raw is None:
        return None
    return OverrideFallback(
        controller_override_window_s=float(raw.get("controller_override_window_s", 2.0)),
        lane_change_profile=str(raw.get("lane_change_profile", "quintic")),
        target_speed_mps=_opt_float(raw.get("target_speed_mps")),
    )


def _opt_float(val: Any) -> Optional[float]:
    return float(val) if val is not None else None


def parse_events(raw: List[Dict[str, Any]]) -> List[EventSpec]:
    """Parse events list from YAML config.

    Args:
        raw: List of event dicts from YAML params["events"]

    Returns:
        List of validated EventSpec dataclasses

    Raises:
        ValueError: If required fields are missing
    """
    events = []
    for item in raw:
        if "id" not in item or "actor" not in item or "trigger" not in item:
            raise ValueError(
                f"Event missing required fields (id, actor, trigger): {item}"
            )

        action = item.get("action", {})
        events.append(
            EventSpec(
                id=str(item["id"]),
                type=str(item.get("type", "generic")),
                actor=str(item["actor"]),
                trigger=_parse_trigger(item["trigger"]),
                action_tm=_parse_tm_action(action.get("tm")),
                action_execute=_parse_tm_action(action.get("execute")),
                action_post=_parse_tm_action(action.get("post")),
                fallback=_parse_fallback(item.get("fallback")),
                gate_min_ttc_s=_opt_float(item.get("gate_min_ttc_s")),
                gate_max_gap_m=_opt_float(item.get("gate_max_gap_m")),
            )
        )
    return events
