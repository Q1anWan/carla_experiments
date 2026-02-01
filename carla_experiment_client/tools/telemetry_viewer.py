"""Interactive telemetry viewer — plan vs telemetry with synced time cursor.

Usage:
    python -m carla_experiment_client.tools.telemetry_viewer \
        --plan outputs/lane_change_cut_in/plan.json \
        --telemetry outputs/lane_change_cut_in/telemetry.json \
        [--scene outputs/scene_designs/lane_change_cut_in/scene_edit.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


def _load_plan(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_telemetry(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_scene(path: Path) -> Optional[dict]:
    if path and path.exists():
        return json.loads(path.read_text())
    return None


def _extract_ego_plan(plan: dict) -> Optional[dict]:
    for actor in plan.get("actors", []):
        if actor.get("role") == "ego":
            return actor
    return plan.get("actors", [{}])[0] if plan.get("actors") else None


def _extract_actor_plan(plan: dict, actor_id: str) -> Optional[dict]:
    for actor in plan.get("actors", []):
        if actor.get("id") == actor_id:
            return actor
    return None


def launch_viewer(
    plan_path: Path,
    telemetry_path: Path,
    scene_path: Optional[Path] = None,
) -> None:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    plan = _load_plan(plan_path)
    telem = _load_telemetry(telemetry_path)
    scene = _load_scene(scene_path) if scene_path else None

    dt = plan.get("dt", 0.05)
    duration = plan.get("duration", 60.0)
    fps = int(round(1.0 / dt))
    frames = telem.get("frames", [])

    # Consistency check
    p_eid = plan.get("episode_id", "")
    if scene:
        s_eid = scene.get("episode_id", "")
        if p_eid and s_eid and p_eid != s_eid:
            print(f"WARNING: episode_id mismatch: plan={p_eid!r} vs scene={s_eid!r}")

    # Extract ego trajectory from plan
    ego_plan = _extract_ego_plan(plan)
    if ego_plan is None:
        print("ERROR: No ego actor in plan")
        return

    plan_traj = ego_plan.get("trajectory", [])
    plan_t = np.array([p["t"] for p in plan_traj])
    plan_x = np.array([p["x"] for p in plan_traj])
    plan_y = np.array([p["y"] for p in plan_traj])
    plan_v = np.array([p.get("v", 0) for p in plan_traj])
    plan_yaw = np.array([p.get("yaw", 0) for p in plan_traj])

    # Extract ego telemetry
    tel_t = []
    tel_x = []
    tel_y = []
    tel_speed = []
    tel_yaw = []
    for f in frames:
        tel_t.append(f.get("t_sim", 0))
        ego = f.get("ego", {})
        tel_x.append(ego.get("world_x", 0))
        tel_y.append(ego.get("world_y", 0))
        tel_speed.append(ego.get("speed", 0))
        tel_yaw.append(ego.get("yaw", ego.get("world_yaw", 0)))
    tel_t = np.array(tel_t)
    tel_x = np.array(tel_x)
    tel_y = np.array(tel_y)
    tel_speed = np.array(tel_speed)
    tel_yaw = np.array(tel_yaw)

    # Compute finite-diff speed from telemetry positions
    tel_speed_fd = np.zeros_like(tel_t)
    for i in range(1, len(tel_t)):
        dx = tel_x[i] - tel_x[i - 1]
        dy = tel_y[i] - tel_y[i - 1]
        tel_speed_fd[i] = math.hypot(dx, dy) / dt
    if len(tel_speed_fd) > 1:
        tel_speed_fd[0] = tel_speed_fd[1]

    # NPC trajectories from plan
    npc_plans = []
    for actor in plan.get("actors", []):
        if actor.get("role") == "ego":
            continue
        traj = actor.get("trajectory", [])
        if traj:
            npc_plans.append({
                "id": actor.get("id", "?"),
                "x": np.array([p["x"] for p in traj]),
                "y": np.array([p["y"] for p in traj]),
                "t": np.array([p["t"] for p in traj]),
            })

    # NPC presence from telemetry
    npc_presence: Dict[str, List[bool]] = {}
    for f in frames:
        seen_roles = set()
        for a in f.get("actors", []):
            role = a.get("role_name", "")
            if role:
                seen_roles.add(role)
        for npc in npc_plans:
            npc_id = npc["id"]
            npc_presence.setdefault(npc_id, []).append(npc_id in seen_roles)

    # --- Build figure ---
    fig = plt.figure(figsize=(16, 10))
    ax_xy = fig.add_axes([0.05, 0.25, 0.45, 0.70])
    ax_speed = fig.add_axes([0.55, 0.60, 0.40, 0.35])
    ax_yaw = fig.add_axes([0.55, 0.25, 0.40, 0.30])
    ax_slider = fig.add_axes([0.05, 0.08, 0.90, 0.03])
    ax_presence = fig.add_axes([0.05, 0.14, 0.90, 0.06])

    # XY plot
    ax_xy.plot(plan_x, plan_y, "b-", linewidth=0.8, label="Plan", alpha=0.7)
    ax_xy.plot(tel_x, tel_y, "r-", linewidth=0.8, label="Telemetry", alpha=0.7)
    for npc in npc_plans:
        ax_xy.plot(npc["x"], npc["y"], "--", linewidth=0.5, alpha=0.5, label=npc["id"])
    cursor_xy, = ax_xy.plot([], [], "ko", markersize=8)
    ax_xy.set_aspect("equal")
    ax_xy.set_title(f"XY Trajectory — {plan.get('episode_id', '?')}")
    ax_xy.legend(fontsize=7, loc="upper left")
    ax_xy.grid(True, alpha=0.3)

    # Speed plot
    ax_speed.plot(plan_t, plan_v, "b-", linewidth=0.8, label="speed_plan")
    ax_speed.plot(tel_t, tel_speed_fd, "g-", linewidth=0.8, label="speed_fd")
    ax_speed.plot(tel_t, tel_speed, "r--", linewidth=0.5, alpha=0.5, label="speed_actor")
    speed_cursor = ax_speed.axvline(0, color="k", linewidth=0.8)
    ax_speed.set_ylabel("Speed (m/s)")
    ax_speed.set_title("Speed")
    ax_speed.legend(fontsize=7)
    ax_speed.grid(True, alpha=0.3)

    # Yaw plot
    ax_yaw.plot(plan_t, plan_yaw, "b-", linewidth=0.8, label="yaw_plan")
    ax_yaw.plot(tel_t, tel_yaw, "r-", linewidth=0.5, alpha=0.5, label="yaw_telem")
    yaw_cursor = ax_yaw.axvline(0, color="k", linewidth=0.8)
    ax_yaw.set_ylabel("Yaw (deg)")
    ax_yaw.set_xlabel("Time (s)")
    ax_yaw.set_title("Yaw")
    ax_yaw.legend(fontsize=7)
    ax_yaw.grid(True, alpha=0.3)

    # Presence timeline
    ax_presence.set_title("Actor Presence", fontsize=8)
    y_pos = 0
    for npc_id, pres in npc_presence.items():
        for i, present in enumerate(pres):
            if present:
                ax_presence.plot(i * dt, y_pos, "g|", markersize=3)
            else:
                ax_presence.plot(i * dt, y_pos, "r|", markersize=1, alpha=0.2)
        ax_presence.text(-1, y_pos, npc_id, fontsize=6, va="center", ha="right")
        y_pos += 1
    presence_cursor = ax_presence.axvline(0, color="k", linewidth=0.8)
    ax_presence.set_xlim(0, duration)
    ax_presence.set_yticks([])

    # Slider
    slider = Slider(ax_slider, "Time (s)", 0, duration, valinit=0)

    def update(val):
        t = slider.val
        # Update XY cursor
        if len(plan_t) > 0:
            idx = np.argmin(np.abs(plan_t - t))
            cursor_xy.set_data([plan_x[idx]], [plan_y[idx]])
        speed_cursor.set_xdata([t])
        yaw_cursor.set_xdata([t])
        presence_cursor.set_xdata([t])
        fig.canvas.draw_idle()

    slider.on_changed(update)

    fig.suptitle(f"Telemetry Viewer: {plan.get('episode_id', '?')} ({plan.get('town', '?')})", fontsize=12)
    plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Interactive telemetry viewer")
    parser.add_argument("--plan", required=True, help="Path to plan.json")
    parser.add_argument("--telemetry", required=True, help="Path to telemetry.json")
    parser.add_argument("--scene", help="Path to scene_edit.json (optional)")
    args = parser.parse_args(argv)

    launch_viewer(
        plan_path=Path(args.plan),
        telemetry_path=Path(args.telemetry),
        scene_path=Path(args.scene) if args.scene else None,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
