"""MVP 2D editor/visualizer for map + plan previews."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

import matplotlib.pyplot as plt

from ..planning.trajectory_schema import Plan, load_plan


def _load_geojson(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _plot_geojson(ax: Any, geojson: Dict[str, Any], *, color: str, linewidth: float, alpha: float) -> None:
    for feature in geojson.get("features", []):
        geom = feature.get("geometry") or {}
        if geom.get("type") != "LineString":
            continue
        coords = geom.get("coordinates", [])
        if not coords:
            continue
        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]
        ax.plot(xs, ys, color=color, linewidth=linewidth, alpha=alpha)


def _plot_junctions(ax: Any, geojson: Dict[str, Any]) -> None:
    for feature in geojson.get("features", []):
        geom = feature.get("geometry") or {}
        if geom.get("type") != "Polygon":
            continue
        coords = geom.get("coordinates", [])
        if not coords:
            continue
        poly = coords[0]
        xs = [c[0] for c in poly]
        ys = [c[1] for c in poly]
        ax.fill(xs, ys, color="#f2d3a4", alpha=0.3, linewidth=0)


def _collect_event_points(plan: Plan) -> list[dict[str, Any]]:
    ego = next((actor for actor in plan.actors if actor.role == "ego"), None)
    if ego is None or not ego.trajectory or plan.dt <= 0:
        return []
    points = []
    max_index = len(ego.trajectory) - 1
    for idx, event in enumerate(plan.events_plan, start=1):
        frame_idx = int(round(event.t_event / plan.dt))
        frame_idx = max(0, min(max_index, frame_idx))
        point = ego.trajectory[frame_idx]
        label = event.event_type or event.expected_action or f"event_{idx:02d}"
        points.append(
            {
                "x": point.x,
                "y": point.y,
                "t_event": float(event.t_event),
                "label": label,
            }
        )
    return points


def plot_map_preview(map_dir: Path, plan_path: Path | None, out_path: Path) -> None:
    lane_geo = _load_geojson(map_dir / "lane_centerlines.geojson")
    junction_geo = _load_geojson(map_dir / "junction_areas.geojson")

    fig, ax = plt.subplots(figsize=(10, 10))
    _plot_junctions(ax, junction_geo)
    _plot_geojson(ax, lane_geo, color="#3b6ea5", linewidth=0.8, alpha=0.7)

    if plan_path and plan_path.exists():
        plan = load_plan(plan_path)
        for actor in plan.actors:
            xs = [p.x for p in actor.trajectory]
            ys = [p.y for p in actor.trajectory]
            color = "#d1495b" if actor.role == "ego" else "#6b9080"
            ax.plot(xs, ys, color=color, linewidth=2.0, alpha=0.9)
        event_points = _collect_event_points(plan)
        if event_points:
            ex = [p["x"] for p in event_points]
            ey = [p["y"] for p in event_points]
            ax.scatter(
                ex,
                ey,
                s=36,
                color="#f4a261",
                edgecolor="#222222",
                linewidth=0.5,
                zorder=4,
                label="Event",
            )
            if len(event_points) <= 10:
                for point in event_points:
                    label = f"{point['label']}@{point['t_event']:.1f}s"
                    ax.text(
                        point["x"],
                        point["y"],
                        label,
                        fontsize=8,
                        color="#333333",
                        ha="left",
                        va="bottom",
                    )
            ax.legend(loc="upper right")

    ax.set_aspect("equal")
    ax.set_title("2D Map Preview")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a 2D map + plan preview PNG.")
    parser.add_argument("--map-dir", required=True, help="Path to exported map directory")
    parser.add_argument("--plan", help="Path to plan.json (optional)")
    parser.add_argument("--out", required=True, help="Output PNG path")
    args = parser.parse_args()

    plot_map_preview(Path(args.map_dir), Path(args.plan) if args.plan else None, Path(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
