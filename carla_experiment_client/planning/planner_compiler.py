"""Planner compiler: high-level episode config -> time-parameterized plan.json."""

from __future__ import annotations

import argparse
import json
import logging
import math
import random
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..carla_client import connect_client, load_world
from ..utils import check_timeout
from .trajectory_schema import ActorPlan, EventPlan, Plan, TrajectoryPoint, save_events_plan, save_plan


def _load_yaml(path: Path) -> Dict[str, Any]:
    raw = yaml.safe_load(path.read_text())
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"Invalid YAML: {path}")
    return raw


def _select_town(episode_cfg: Dict[str, Any]) -> str:
    town_cfg = episode_cfg.get("town") or {}
    if isinstance(town_cfg, dict):
        preferred = town_cfg.get("preferred") or []
        if preferred:
            return str(preferred[0])
    return str(town_cfg) if town_cfg else "Town05"


def _count_driving_lanes(waypoint: "carla.Waypoint") -> int:
    count = 1
    visited_left: set[tuple[int, int, int]] = set()
    left = waypoint.get_left_lane()
    while left and left.lane_type == carla.LaneType.Driving:
        key = (left.road_id, left.section_id, left.lane_id)
        if key in visited_left:
            break
        visited_left.add(key)
        count += 1
        left = left.get_left_lane()
    visited_right: set[tuple[int, int, int]] = set()
    right = waypoint.get_right_lane()
    while right and right.lane_type == carla.LaneType.Driving:
        key = (right.road_id, right.section_id, right.lane_id)
        if key in visited_right:
            break
        visited_right.add(key)
        count += 1
        right = right.get_right_lane()
    return count


def _has_junction_ahead(waypoint: "carla.Waypoint", distance: float, step: float = 5.0) -> bool:
    traveled = 0.0
    current = waypoint
    visited: set[tuple[int, int, int]] = set()
    while traveled < distance:
        key = (current.road_id, current.section_id, current.lane_id)
        if key in visited:
            break
        visited.add(key)
        next_wps = current.next(step)
        if not next_wps:
            return False
        current = next_wps[0]
        traveled += step
        if current.is_junction:
            return True
    return False


def _load_spawn_candidates(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    raw = json.loads(path.read_text())
    if not isinstance(raw, list):
        return []
    return [item for item in raw if isinstance(item, dict)]


def _select_start_waypoint(
    world: "carla.World",
    map_obj: "carla.Map",
    rng: random.Random,
    episode_cfg: Dict[str, Any],
    *,
    start_time: float,
    max_seconds: float | None,
) -> "carla.Waypoint":
    selector = (episode_cfg.get("topology_query") or {}).get("start_selector") or {}
    selector_type = selector.get("type", "multi_lane_straight")
    min_lanes = int(selector.get("min_lanes", 2))
    forward_clear_m = float(selector.get("forward_clear_m", 120.0))
    avoid_junction = bool(selector.get("avoid_junction", True))
    map_name = map_obj.name.split("/")[-1]
    spawn_candidates = _load_spawn_candidates(Path("data/maps") / map_name / "spawn_candidates.json")
    logging.info("Selecting spawn point (map=%s, candidates=%d)", map_name, len(spawn_candidates))
    if spawn_candidates:
        rng.shuffle(spawn_candidates)
        for index, candidate in enumerate(spawn_candidates[:120], start=1):
            if index == 1 or index % 20 == 0:
                logging.info("Checking spawn candidate %d", index)
            check_timeout(start_time, max_seconds, "planner_compiler")
            loc = carla.Location(
                x=float(candidate.get("x", 0.0)),
                y=float(candidate.get("y", 0.0)),
                z=float(candidate.get("z", 0.0)),
            )
            wp = map_obj.get_waypoint(loc, project_to_road=True)
            if wp is None:
                continue
            if avoid_junction and wp.is_junction:
                continue
            if min_lanes > 1 and _count_driving_lanes(wp) < min_lanes:
                continue
            if forward_clear_m and _has_junction_ahead(wp, forward_clear_m):
                continue
            spawn_transform = carla.Transform(loc, carla.Rotation(yaw=float(candidate.get("yaw", 0.0))))
            break
        else:
            spawn_transform = None
    else:
        spawn_transform = None

    if spawn_transform is None:
        spawn_points = map_obj.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available.")
        candidates = list(spawn_points)
        rng.shuffle(candidates)
        spawn_transform = None
        for index, sp in enumerate(candidates[:120], start=1):
            if index == 1 or index % 20 == 0:
                logging.info("Checking map spawn point %d", index)
            check_timeout(start_time, max_seconds, "planner_compiler")
            wp = map_obj.get_waypoint(sp.location, project_to_road=True)
            if wp is None:
                continue
            if avoid_junction and wp.is_junction:
                continue
            if min_lanes > 1 and _count_driving_lanes(wp) < min_lanes:
                continue
            if forward_clear_m and _has_junction_ahead(wp, forward_clear_m):
                continue
            spawn_transform = sp
            break
        if spawn_transform is None:
            spawn_transform = candidates[0]
    waypoint = map_obj.get_waypoint(spawn_transform.location, project_to_road=True)
    if waypoint is None:
        raise RuntimeError("Failed to resolve waypoint from spawn transform.")
    offset_back = float(selector.get("offset_back_m", 0.0))
    if offset_back > 0:
        prev_wps = waypoint.previous(offset_back)
        if prev_wps:
            waypoint = prev_wps[0]
    return waypoint


def _route_forward_distance(episode_cfg: Dict[str, Any], globals_cfg: Dict[str, Any]) -> float:
    route = (episode_cfg.get("topology_query") or {}).get("route_selector") or {}
    forward_m = route.get("forward_m")
    if forward_m is not None:
        return float(forward_m)
    termination = episode_cfg.get("termination") or {}
    if termination.get("min_progress_m"):
        return float(termination.get("min_progress_m"))
    return float(globals_cfg.get("validation", {}).get("min_distance_progress_m", 200.0))


def _detect_lane_change(episode_cfg: Dict[str, Any]) -> bool:
    markers = episode_cfg.get("event_markers") or []
    for marker in markers:
        action = str(marker.get("expected_action", "")).lower()
        if "lane_change" in action or "change_lane" in action:
            return True
    episode_id = str(episode_cfg.get("episode_id", ""))
    return "lane_change" in episode_id


def _lane_change_start_distance(episode_cfg: Dict[str, Any], fallback: float) -> float:
    selector = (episode_cfg.get("topology_query") or {}).get("decision_point_selector") or {}
    offset = selector.get("offset_forward_m")
    if offset is not None:
        return float(offset)
    return fallback


def _sample_waypoints(
    waypoint: "carla.Waypoint",
    distance_m: float,
    step_m: float,
) -> List["carla.Waypoint"]:
    points = [waypoint]
    traveled = 0.0
    current = waypoint
    while traveled < distance_m:
        next_wps = current.next(step_m)
        if not next_wps:
            break
        current = next_wps[0]
        points.append(current)
        traveled += step_m
    return points


def _interpolate_yaw(yaw_a: float, yaw_b: float, alpha: float) -> float:
    diff = (yaw_b - yaw_a + 180.0) % 360.0 - 180.0
    return yaw_a + diff * alpha


def _build_trajectory(
    base_lane: List["carla.Waypoint"],
    target_lane: Optional[List["carla.Waypoint"]],
    *,
    dt: float,
    speed_mps: float,
    lane_change_start_s: float,
    lane_change_duration_s: float,
) -> List[TrajectoryPoint]:
    points: List[TrajectoryPoint] = []
    if not base_lane:
        return points
    step_m = max(0.1, speed_mps * dt)
    total_steps = len(base_lane)
    lane_change_start_idx = int(lane_change_start_s / dt)
    lane_change_steps = max(1, int(lane_change_duration_s / dt))

    def wp_at(seq: List["carla.Waypoint"], idx: int) -> "carla.Waypoint":
        if idx < len(seq):
            return seq[idx]
        return seq[-1]

    for idx in range(total_steps):
        t = idx * dt
        base_wp = wp_at(base_lane, idx)
        if target_lane and idx >= lane_change_start_idx:
            alpha = min(1.0, max(0.0, (idx - lane_change_start_idx) / lane_change_steps))
            target_wp = wp_at(target_lane, idx)
            bx, by = base_wp.transform.location.x, base_wp.transform.location.y
            tx, ty = target_wp.transform.location.x, target_wp.transform.location.y
            x = bx + (tx - bx) * alpha
            y = by + (ty - by) * alpha
            yaw = _interpolate_yaw(base_wp.transform.rotation.yaw, target_wp.transform.rotation.yaw, alpha)
            lane_id = int(target_wp.lane_id if alpha >= 0.5 else base_wp.lane_id)
            s = float(target_wp.s if alpha >= 0.5 else base_wp.s)
        else:
            x = base_wp.transform.location.x
            y = base_wp.transform.location.y
            yaw = base_wp.transform.rotation.yaw
            lane_id = int(base_wp.lane_id)
            s = float(base_wp.s)
        points.append(
            TrajectoryPoint(
                t=t,
                x=x,
                y=y,
                yaw=yaw,
                v=speed_mps,
                a=0.0,
                lane_id=lane_id,
                s=s,
            )
        )

    if points and len(points) > 1:
        for idx in range(len(points) - 1):
            dx = points[idx + 1].x - points[idx].x
            dy = points[idx + 1].y - points[idx].y
            points[idx].yaw = math.degrees(math.atan2(dy, dx))
        points[-1].yaw = points[-2].yaw
    return points


def compile_episode(
    episode_id: str,
    *,
    globals_path: Path,
    episodes_dir: Path,
    out_dir: Path,
    max_seconds: float | None = None,
) -> Plan:
    if carla is None:
        raise RuntimeError("CARLA Python API is not available.")

    start_time = time.monotonic()
    globals_cfg = _load_yaml(globals_path)
    episode_path = episodes_dir / f"{episode_id}.yaml"
    episode_cfg = _load_yaml(episode_path)

    town = _select_town(episode_cfg)
    seed = int(episode_cfg.get("seed", 0))
    rng = random.Random(seed)
    logging.info("Compiling episode %s (town=%s, seed=%d)", episode_id, town, seed)

    carla_cfg = globals_cfg.get("carla", {})
    client, _, _ = connect_client(
        str(carla_cfg.get("host", "127.0.0.1")),
        int(carla_cfg.get("port", 2000)),
        float(carla_cfg.get("timeout_s", 10.0)),
        allow_version_mismatch=bool(carla_cfg.get("allow_version_mismatch", True)),
    )
    check_timeout(start_time, max_seconds, "planner_compiler")
    world = load_world(client, town)
    map_obj = world.get_map()

    start_wp = _select_start_waypoint(
        world,
        map_obj,
        rng,
        episode_cfg,
        start_time=start_time,
        max_seconds=max_seconds,
    )
    check_timeout(start_time, max_seconds, "planner_compiler")
    route_distance = _route_forward_distance(episode_cfg, globals_cfg)
    logging.info("Route distance %.1fm", route_distance)

    dt = float(globals_cfg.get("carla", {}).get("fixed_delta_seconds", 0.05))
    speed_kmh = float((episode_cfg.get("ego") or {}).get("target_speed_kmh", 45))
    speed_mps = max(1.0, speed_kmh / 3.6)
    logging.info("Ego speed %.1f km/h (%.2f m/s), dt=%.2f", speed_kmh, speed_mps, dt)

    base_lane = _sample_waypoints(start_wp, route_distance, speed_mps * dt)
    lane_change = _detect_lane_change(episode_cfg)
    target_lane = None
    if lane_change:
        right_lane = start_wp.get_right_lane()
        left_lane = start_wp.get_left_lane()
        if right_lane and right_lane.lane_type == carla.LaneType.Driving:
            target_lane = _sample_waypoints(right_lane, route_distance, speed_mps * dt)
        elif left_lane and left_lane.lane_type == carla.LaneType.Driving:
            target_lane = _sample_waypoints(left_lane, route_distance, speed_mps * dt)
        logging.info("Lane change target lane %s", "right" if right_lane else "left")

    duration = float((episode_cfg.get("termination") or {}).get("max_time_s", 60.0))
    lane_change_start_dist = _lane_change_start_distance(episode_cfg, route_distance * 0.5)
    lane_change_start_s = lane_change_start_dist / speed_mps
    lane_change_duration_s = float(episode_cfg.get("lane_change_duration_s", 4.0))
    logging.info(
        "Lane change start %.1fm (t=%.2fs), duration %.2fs",
        lane_change_start_dist,
        lane_change_start_s,
        lane_change_duration_s,
    )

    trajectory = _build_trajectory(
        base_lane,
        target_lane,
        dt=dt,
        speed_mps=speed_mps,
        lane_change_start_s=lane_change_start_s,
        lane_change_duration_s=lane_change_duration_s,
    )
    check_timeout(start_time, max_seconds, "planner_compiler")
    if not trajectory:
        raise RuntimeError("Trajectory generation failed (no points).")
    target_steps = max(1, int(duration / dt))
    if len(trajectory) < target_steps:
        last = trajectory[-1]
        for idx in range(len(trajectory), target_steps):
            trajectory.append(
                TrajectoryPoint(
                    t=idx * dt,
                    x=last.x,
                    y=last.y,
                    yaw=last.yaw,
                    v=0.0,
                    a=0.0,
                    lane_id=last.lane_id,
                    s=last.s,
                )
            )
    elif len(trajectory) > target_steps:
        trajectory = trajectory[:target_steps]

    blueprint = str(globals_cfg.get("ego", {}).get("blueprint", "vehicle.tesla.model3"))
    ego_actor = ActorPlan(
        actor_id="ego",
        kind="vehicle",
        role="ego",
        blueprint=blueprint,
        controller="teleport",
        trajectory=trajectory,
    )

    events_plan: List[EventPlan] = []
    if lane_change:
        events_cfg = globals_cfg.get("events", {})
        voice_lead = float(events_cfg.get("voice_lead_time_s", 3.0))
        robot_precue = float(events_cfg.get("robot_precue_lead_s", 0.5))
        t_event = round(lane_change_start_s, 3)
        t_voice = max(0.0, t_event - voice_lead)
        t_robot = max(0.0, t_voice - robot_precue)
        events_plan.append(
            EventPlan(
                t_event=t_event,
                event_type="lane_change",
                expected_action="lane_change",
                audio_id="lane_change_00",
                t_voice_start=t_voice,
                t_robot_precue=t_robot,
            )
        )

    plan = Plan(
        episode_id=episode_id,
        town=town,
        seed=seed,
        dt=dt,
        duration=duration,
        actors=[ego_actor],
        events_plan=events_plan,
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    save_plan(out_dir / "plan.json", plan)
    save_events_plan(out_dir / "events_plan.json", plan.events_plan)
    logging.info("Plan compiled: %s", out_dir)
    return plan


def main() -> int:
    parser = argparse.ArgumentParser(description="Compile episode config into plan.json.")
    parser.add_argument("--episode", required=True, help="Episode ID (e.g., P1_T2_lane_change)")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--episodes-dir", default="configs/episodes", help="Episode YAML directory")
    parser.add_argument("--out", default="outputs", help="Output directory root")
    parser.add_argument("--max-seconds", type=float, default=300.0, help="Maximum runtime before abort")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    out_dir = Path(args.out) / args.episode
    compile_episode(
        args.episode,
        globals_path=Path(args.globals),
        episodes_dir=Path(args.episodes_dir),
        out_dir=out_dir,
        max_seconds=args.max_seconds,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
