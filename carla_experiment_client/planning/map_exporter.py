"""Export CARLA map topology to JSON + GeoJSON for 2D planning."""

from __future__ import annotations

import argparse
import json
import logging
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import yaml

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..carla_client import connect_client, load_world
from ..utils import check_timeout


@dataclass
class MapExportConfig:
    host: str
    port: int
    timeout: float
    map_name: str
    waypoint_distance: float = 2.0


def _lane_uid(road_id: int, section_id: int, lane_id: int) -> str:
    return f"{road_id}:{section_id}:{lane_id}"


def _load_globals(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _extract_spawn_candidates(map_obj: "carla.Map") -> List[Dict[str, Any]]:
    candidates = []
    for sp in map_obj.get_spawn_points():
        wp = map_obj.get_waypoint(sp.location, project_to_road=True)
        candidates.append(
            {
                "x": round(sp.location.x, 3),
                "y": round(sp.location.y, 3),
                "z": round(sp.location.z, 3),
                "yaw": round(sp.rotation.yaw, 3),
                "road_id": int(wp.road_id) if wp else None,
                "section_id": int(wp.section_id) if wp else None,
                "lane_id": int(wp.lane_id) if wp else None,
            }
        )
    return candidates


def _group_lane_centerlines(
    waypoints: Iterable["carla.Waypoint"],
) -> Dict[str, List["carla.Waypoint"]]:
    buckets: Dict[str, List["carla.Waypoint"]] = defaultdict(list)
    for wp in waypoints:
        lane_id = _lane_uid(wp.road_id, wp.section_id, wp.lane_id)
        buckets[lane_id].append(wp)
    for lane_id, items in buckets.items():
        items.sort(key=lambda w: w.s)
    return buckets


def _junction_polygons(waypoints: Iterable["carla.Waypoint"]) -> List[Dict[str, Any]]:
    junction_bounds: Dict[int, List[Tuple[float, float]]] = defaultdict(list)
    for wp in waypoints:
        if not wp.is_junction:
            continue
        try:
            junction = wp.get_junction()
        except RuntimeError:
            junction = None
        if junction is not None:
            bbox = junction.bounding_box
            extent = bbox.extent
            center = bbox.location
            points = [
                (center.x - extent.x, center.y - extent.y),
                (center.x - extent.x, center.y + extent.y),
                (center.x + extent.x, center.y + extent.y),
                (center.x + extent.x, center.y - extent.y),
                (center.x - extent.x, center.y - extent.y),
            ]
            junction_bounds[junction.id].extend(points)
        else:
            junction_bounds[-1].append((wp.transform.location.x, wp.transform.location.y))

    features = []
    for junction_id, points in junction_bounds.items():
        if len(points) < 4:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        polygon = [
            [min_x, min_y],
            [min_x, max_y],
            [max_x, max_y],
            [max_x, min_y],
            [min_x, min_y],
        ]
        features.append(
            {
                "type": "Feature",
                "properties": {"junction_id": int(junction_id)},
                "geometry": {"type": "Polygon", "coordinates": [polygon]},
            }
        )
    return features


# DocRef: technical_details.md#4.5
def export_map(config: MapExportConfig, out_dir: Path, *, max_seconds: float | None = None) -> None:
    if carla is None:
        raise RuntimeError("CARLA Python API is not available.")

    start_time = time.monotonic()
    client, _, _ = connect_client(
        config.host,
        config.port,
        config.timeout,
        allow_version_mismatch=True,
    )
    check_timeout(start_time, max_seconds, "map_exporter")
    world = load_world(client, config.map_name)
    map_obj = world.get_map()

    out_dir.mkdir(parents=True, exist_ok=True)
    logging.info("Generating waypoints (distance=%.1fm)...", config.waypoint_distance)
    waypoints = map_obj.generate_waypoints(config.waypoint_distance)
    logging.info("Waypoints generated: %d", len(waypoints))
    check_timeout(start_time, max_seconds, "map_exporter")
    lane_groups = _group_lane_centerlines(waypoints)
    logging.info("Lane groups: %d", len(lane_groups))

    lanes = []
    centerline_features = []
    for index, (lane_key, items) in enumerate(lane_groups.items(), start=1):
        if not items:
            continue
        first = items[0]
        last = items[-1]
        left_wp = first.get_left_lane()
        right_wp = first.get_right_lane()

        def lane_ref(wp: "carla.Waypoint") -> str | None:
            if wp is None:
                return None
            return _lane_uid(wp.road_id, wp.section_id, wp.lane_id)

        next_wps = last.next(config.waypoint_distance)
        prev_wps = first.previous(config.waypoint_distance)

        centerline = [[round(w.transform.location.x, 3), round(w.transform.location.y, 3)] for w in items]
        lanes.append(
            {
                "id": lane_key,
                "road_id": int(first.road_id),
                "section_id": int(first.section_id),
                "lane_id": int(first.lane_id),
                "lane_type": str(first.lane_type),
                "left_lane": lane_ref(left_wp),
                "right_lane": lane_ref(right_wp),
                "successors": [lane_ref(w) for w in next_wps if w],
                "predecessors": [lane_ref(w) for w in prev_wps if w],
                "centerline": centerline,
            }
        )
        centerline_features.append(
            {
                "type": "Feature",
                "properties": {
                    "lane_id": lane_key,
                    "road_id": int(first.road_id),
                    "section_id": int(first.section_id),
                    "lane_type": str(first.lane_type),
                },
                "geometry": {"type": "LineString", "coordinates": centerline},
            }
        )
        if index % 200 == 0:
            logging.info("Processed %d lane centerlines", index)
            check_timeout(start_time, max_seconds, "map_exporter")

    map_graph = {
        "map": map_obj.name,
        "waypoint_distance": config.waypoint_distance,
        "lanes": lanes,
    }
    (out_dir / "map_graph.json").write_text(
        json.dumps(map_graph, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )

    lane_centerlines = {"type": "FeatureCollection", "features": centerline_features}
    (out_dir / "lane_centerlines.geojson").write_text(
        json.dumps(lane_centerlines, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )

    junction_features = _junction_polygons(waypoints)
    junction_geojson = {"type": "FeatureCollection", "features": junction_features}
    (out_dir / "junction_areas.geojson").write_text(
        json.dumps(junction_geojson, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )

    check_timeout(start_time, max_seconds, "map_exporter")
    spawn_candidates = _extract_spawn_candidates(map_obj)
    (out_dir / "spawn_candidates.json").write_text(
        json.dumps(spawn_candidates, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )

    logging.info("Map export complete: %s", out_dir)


def _build_config(args: argparse.Namespace) -> MapExportConfig:
    globals_cfg = _load_globals(Path(args.globals))
    carla_cfg = globals_cfg.get("carla", {}) if isinstance(globals_cfg, dict) else {}
    return MapExportConfig(
        host=str(args.host or carla_cfg.get("host", "127.0.0.1")),
        port=int(args.port or carla_cfg.get("port", 2000)),
        timeout=float(args.timeout or carla_cfg.get("timeout_s", 10.0)),
        map_name=str(args.map),
        waypoint_distance=float(args.waypoint_distance),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Export CARLA map topology for planning.")
    parser.add_argument("--map", required=True, help="CARLA map name (e.g., Town05)")
    parser.add_argument("--out", type=Path, default=Path("data/maps"), help="Output base directory")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--host", help="CARLA host override")
    parser.add_argument("--port", type=int, help="CARLA port override")
    parser.add_argument("--timeout", type=float, help="CARLA timeout override")
    parser.add_argument("--waypoint-distance", type=float, default=2.0, help="Waypoint sampling distance (m)")
    parser.add_argument("--max-seconds", type=float, default=600.0, help="Maximum runtime before abort")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    config = _build_config(args)
    out_dir = Path(args.out) / config.map_name
    export_map(config, out_dir, max_seconds=args.max_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
