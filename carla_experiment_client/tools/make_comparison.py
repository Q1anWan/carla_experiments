"""Generate comparison plots between telemetry CSV and scene_edit keyframe interpolation.

Usage:
    python -m carla_experiment_client.tools.make_comparison \
        --scene outputs/scene_designs/lane_change_cut_in/scene_edit.json \
        --telemetry-csv runs/final_fix_validation/lane_change_cut_in/telemetry.csv \
        --output outputs/scene_designs/lane_change_cut_in/ \
        [--events runs/final_fix_validation/lane_change_cut_in/events.json] \
        [--actors ego,cut_in_vehicle]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    """Load telemetry CSV into column arrays."""
    import csv
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    cols: Dict[str, List[float]] = {}
    for row in rows:
        for k, v in row.items():
            cols.setdefault(k.strip(), []).append(float(v))
    return {k: np.array(v) for k, v in cols.items()}


def _load_scene(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_events(path: Path) -> list:
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    if isinstance(data, list):
        return data
    return data.get("events", [])


def _interpolate_keyframes(keyframes: list, dt: float, duration: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Linear interpolation of keyframes → (t, x, y, speed) arrays."""
    kfs = sorted(keyframes, key=lambda k: k["t"])
    if not kfs:
        return np.array([]), np.array([]), np.array([]), np.array([])

    n_frames = int(duration / dt) + 1
    t_arr = np.arange(n_frames) * dt
    x_arr = np.zeros(n_frames)
    y_arr = np.zeros(n_frames)

    for i, t in enumerate(t_arr):
        # Find bounding keyframes
        k0 = kfs[0]
        k1 = kfs[-1]
        for j in range(len(kfs) - 1):
            if kfs[j]["t"] <= t <= kfs[j + 1]["t"]:
                k0 = kfs[j]
                k1 = kfs[j + 1]
                break
        else:
            if t < kfs[0]["t"]:
                k0 = k1 = kfs[0]
            elif t > kfs[-1]["t"]:
                k0 = k1 = kfs[-1]

        span = k1["t"] - k0["t"]
        alpha = (t - k0["t"]) / span if span > 1e-6 else 0.0
        x_arr[i] = k0["x"] + (k1["x"] - k0["x"]) * alpha
        y_arr[i] = k0["y"] + (k1["y"] - k0["y"]) * alpha

    # Compute speed from positions
    speed = np.zeros(n_frames)
    for i in range(1, n_frames):
        dx = x_arr[i] - x_arr[i - 1]
        dy = y_arr[i] - y_arr[i - 1]
        speed[i] = math.sqrt(dx * dx + dy * dy) / dt
    speed[0] = speed[1] if n_frames > 1 else 0.0

    return t_arr, x_arr, y_arr, speed


def generate_comparison(
    scene_path: Path,
    csv_path: Path,
    output_dir: Path,
    events_path: Optional[Path] = None,
    actors_filter: Optional[List[str]] = None,
) -> dict:
    """Generate comparison.png and summary.json.

    Returns summary dict with error statistics.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scene = _load_scene(scene_path)
    csv_data = _load_csv(csv_path)
    events = _load_events(events_path) if events_path else []

    dt = scene.get("dt", 0.05)
    duration = scene.get("duration", 60.0)

    # Find ego actor in scene
    ego_actor = None
    for actor in scene.get("actors", []):
        if actor.get("role") == "ego" or actor.get("id") == "ego":
            ego_actor = actor
            break
    if ego_actor is None and scene.get("actors"):
        ego_actor = scene["actors"][0]

    if ego_actor is None:
        print("ERROR: No ego actor in scene")
        return {"error": "no ego actor"}

    kf_t, kf_x, kf_y, kf_speed = _interpolate_keyframes(
        ego_actor["keyframes"], dt, duration
    )

    csv_t = csv_data.get("t_sim", np.array([]))
    csv_x = csv_data.get("world_x", np.array([]))
    csv_y = csv_data.get("world_y", np.array([]))
    csv_speed = csv_data.get("speed", np.array([]))

    # Compute interpolation error (align by time via interpolation)
    if len(csv_t) > 1 and len(kf_t) > 1:
        # Use CSV time as reference, interpolate KF positions at CSV times
        t_common = csv_t
        kf_x_at_csv = np.interp(csv_t, kf_t, kf_x)
        kf_y_at_csv = np.interp(csv_t, kf_t, kf_y)
        kf_speed_at_csv = np.interp(csv_t, kf_t, kf_speed)
        err = np.sqrt((kf_x_at_csv - csv_x)**2 + (kf_y_at_csv - csv_y)**2)
        mean_err = float(np.mean(err))
        max_err = float(np.max(err))
        p95_err = float(np.percentile(err, 95))
    else:
        err = np.array([])
        mean_err = max_err = p95_err = 0.0
        t_common = np.array([])
        kf_x_at_csv = kf_y_at_csv = kf_speed_at_csv = np.array([])

    # Plot
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

    # Left: XY trajectory
    ax1.plot(csv_x, csv_y, "b-", linewidth=1.2, label="CSV telemetry")
    ax1.plot(kf_x, kf_y, "r--o", markersize=2, linewidth=0.8, label="Keyframe interp")
    # Mark keyframe positions
    for kf in ego_actor["keyframes"]:
        ax1.plot(kf["x"], kf["y"], "ro", markersize=6)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("XY Trajectory")
    ax1.legend(fontsize=8)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    # Center: Speed comparison
    ax2.plot(csv_t, csv_speed, "b-", linewidth=1.0, label="CSV speed")
    ax2.plot(kf_t, kf_speed, "r--", linewidth=0.8, label="KF speed")
    for evt in events:
        t_evt = evt.get("t_event", evt.get("t", 0))
        ax2.axvline(t_evt, color="green", alpha=0.5, linestyle="--")
        ax2.text(t_evt, ax2.get_ylim()[1] * 0.9,
                evt.get("event_type", "?"), fontsize=6, rotation=45, color="green")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.set_title("Ego Speed")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Right: Error curve
    if len(t_common) > 0:
        ax3.plot(t_common, err, "k-", linewidth=0.8)
        ax3.axhline(mean_err, color="orange", linestyle="--", label=f"mean={mean_err:.2f}m")
        ax3.axhline(p95_err, color="red", linestyle=":", alpha=0.5, label=f"p95={p95_err:.2f}m")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position Error (m)")
    ax3.set_title(f"Interp Error: mean={mean_err:.2f}m, max={max_err:.2f}m")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    fig.suptitle(f"Comparison: {scene.get('episode_id', 'unknown')}", fontsize=12)
    fig.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / "comparison.png", dpi=150)
    plt.close(fig)

    summary = {
        "episode_id": scene.get("episode_id", "unknown"),
        "mean_error_m": round(mean_err, 3),
        "p95_error_m": round(p95_err, 3),
        "max_error_m": round(max_err, 3),
        "n_frames_compared": len(t_common),
        "n_keyframes": len(ego_actor["keyframes"]),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    print(f"Saved comparison.png + summary.json to {output_dir}")
    print(f"  Mean error: {mean_err:.2f}m | P95: {p95_err:.2f}m | Max: {max_err:.2f}m")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate telemetry vs keyframe comparison.")
    parser.add_argument("--scene", required=True, help="Path to scene_edit.json")
    parser.add_argument("--telemetry-csv", required=True, help="Path to telemetry.csv")
    parser.add_argument("--events", help="Path to events.json (optional)")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--actors", help="Comma-separated actor filter (default: ego only)")
    args = parser.parse_args(argv)

    actors_filter = args.actors.split(",") if args.actors else None

    summary = generate_comparison(
        scene_path=Path(args.scene),
        csv_path=Path(args.telemetry_csv),
        output_dir=Path(args.output),
        events_path=Path(args.events) if args.events else None,
        actors_filter=actors_filter,
    )
    return 0 if "error" not in summary else 1


if __name__ == "__main__":
    sys.exit(main())
