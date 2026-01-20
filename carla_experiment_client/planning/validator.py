"""Validate plan.json for map feasibility and basic event presence."""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..carla_client import connect_client, load_world
from ..utils import check_timeout
from .trajectory_schema import Plan, load_plan


def _load_globals(path: Path) -> Dict[str, Any]:
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _validate_map_feasibility(
    plan: Plan,
    map_obj: "carla.Map",
    *,
    vehicle_threshold_m: float = 1.5,
    walker_threshold_m: float = 2.5,
    start_time: float,
    max_seconds: float | None,
) -> List[Dict[str, Any]]:
    violations = []
    for actor in plan.actors:
        lane_type = carla.LaneType.Driving
        threshold = vehicle_threshold_m
        if actor.kind.lower() in {"walker", "vru", "pedestrian"}:
            lane_type = carla.LaneType.Sidewalk
            threshold = walker_threshold_m
        for index, point in enumerate(actor.trajectory, start=1):
            loc = carla.Location(x=point.x, y=point.y, z=0.0)
            wp = map_obj.get_waypoint(loc, project_to_road=True, lane_type=lane_type)
            if wp is None:
                violations.append(
                    {
                        "type": "map_feasibility",
                        "actor_id": actor.actor_id,
                        "t": point.t,
                        "reason": "waypoint_not_found",
                    }
                )
                continue
            dist = loc.distance(wp.transform.location)
            lane_width = float(getattr(wp, "lane_width", 0.0) or 0.0)
            sample_threshold = threshold
            if lane_width > 0.0:
                sample_threshold = max(sample_threshold, lane_width * 0.5 + 0.2)
            if dist > sample_threshold:
                violations.append(
                    {
                        "type": "map_feasibility",
                        "actor_id": actor.actor_id,
                        "t": point.t,
                        "distance_m": round(dist, 3),
                        "threshold_m": round(sample_threshold, 3),
                        "lane_width_m": round(lane_width, 3),
                    }
                )
                break
            if index % 200 == 0:
                check_timeout(start_time, max_seconds, "validator")
    return violations


def _validate_event_presence(plan: Plan, min_events: int) -> List[Dict[str, Any]]:
    violations = []
    if len(plan.events_plan) < min_events:
        violations.append(
            {
                "type": "event_presence",
                "reason": "not_enough_events",
                "expected_min": min_events,
                "actual": len(plan.events_plan),
            }
        )
    return violations


def _validate_lane_change(plan: Plan, window_s: float) -> List[Dict[str, Any]]:
    violations = []
    ego = next((actor for actor in plan.actors if actor.role == "ego"), None)
    if ego is None or not ego.trajectory:
        return violations
    window_s = max(0.0, window_s)
    for event in plan.events_plan:
        if "lane_change" not in event.expected_action:
            continue
        t_event = event.t_event
        idx_event = max(0, min(len(ego.trajectory) - 1, int(t_event / plan.dt)))
        window_frames = int(window_s / plan.dt) if plan.dt > 0 else 0
        start_idx = max(0, idx_event - window_frames)
        end_idx = min(len(ego.trajectory) - 1, idx_event + window_frames)
        lane_event = ego.trajectory[idx_event].lane_id
        lane_ids = {
            point.lane_id
            for point in ego.trajectory[start_idx : end_idx + 1]
            if point.lane_id is not None
        }
        if len(lane_ids) <= 1:
            violations.append(
                {
                    "type": "event_recognition",
                    "reason": "lane_id_no_change",
                    "t_event": t_event,
                    "lane_event": lane_event,
                    "window_s": round(window_s, 3),
                }
            )
    return violations


def validate_plan(
    plan_path: Path,
    globals_path: Path,
    out_dir: Path,
    *,
    max_seconds: float | None = None,
) -> Dict[str, Any]:
    if carla is None:
        raise RuntimeError("CARLA Python API is not available.")
    start_time = time.monotonic()
    globals_cfg = _load_globals(globals_path)
    plan = load_plan(plan_path)
    logging.info("Validating plan %s (actors=%d)", plan.episode_id, len(plan.actors))

    carla_cfg = globals_cfg.get("carla", {})
    client, _, _ = connect_client(
        str(carla_cfg.get("host", "127.0.0.1")),
        int(carla_cfg.get("port", 2000)),
        float(carla_cfg.get("timeout_s", 10.0)),
        allow_version_mismatch=bool(carla_cfg.get("allow_version_mismatch", True)),
    )
    world = load_world(client, plan.town)
    map_obj = world.get_map()

    min_events = int(globals_cfg.get("events", {}).get("min_events_per_episode", 1))
    lane_change_window = float(globals_cfg.get("validation", {}).get("lane_change_window_s", 4.0))
    violations = []
    violations.extend(_validate_event_presence(plan, min_events))
    violations.extend(_validate_lane_change(plan, lane_change_window))
    check_timeout(start_time, max_seconds, "validator")
    violations.extend(
        _validate_map_feasibility(
            plan,
            map_obj,
            start_time=start_time,
            max_seconds=max_seconds,
        )
    )
    logging.info("Validation violations: %d", len(violations))

    report = {
        "episode_id": plan.episode_id,
        "status": "pass" if not violations else "fail",
        "violations": violations,
        "suggested_fixes": [],
    }
    if violations:
        report["suggested_fixes"].append("Adjust trajectory to stay within lane centerline tolerance.")
        if any(v["type"] == "event_presence" for v in violations):
            report["suggested_fixes"].append("Add or shift event markers to ensure at least one core event.")

    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / "validation_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")
    logging.info("Validation report saved: %s", report_path)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate plan.json before rendering.")
    parser.add_argument("--plan", required=True, help="Path to plan.json")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--out", default=".", help="Output directory")
    parser.add_argument("--max-seconds", type=float, default=300.0, help="Maximum runtime before abort")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    validate_plan(Path(args.plan), Path(args.globals), Path(args.out), max_seconds=args.max_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
