"""Convert validated telemetry runs to scene_edit.json for the interactive editor.

Usage:
    python -m carla_experiment_client.tools.convert_telemetry_to_scene \
        --csv runs/final_fix_validation/lane_change_cut_in/telemetry.csv \
        --json runs/final_fix_validation/lane_change_cut_in/telemetry.json \
        --events runs/final_fix_validation/lane_change_cut_in/events.json \
        --output outputs/scene_designs/lane_change_cut_in/scene_edit.json \
        --town Town05 --duration 60 --dt 0.05
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional


def _sample_ego_keyframes(csv_path: Path, interval_s: float = 10.0) -> list:
    """Extract ego keyframes from telemetry CSV at regular intervals."""
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        return []

    keyframes = []
    last_t = -999.0
    for row in rows:
        t = float(row["t_sim"])
        if t - last_t >= interval_s or t == 0.0:
            kf = {
                "t": round(t, 2),
                "x": round(float(row["world_x"]), 3),
                "y": round(float(row["world_y"]), 3),
                "v": round(float(row["speed"]), 2),
            }
            keyframes.append(kf)
            last_t = t

    # Ensure last frame is included
    last_row = rows[-1]
    t_last = float(last_row["t_sim"])
    if t_last - last_t > 1.0:
        keyframes.append({
            "t": round(t_last, 2),
            "x": round(float(last_row["world_x"]), 3),
            "y": round(float(last_row["world_y"]), 3),
            "v": round(float(last_row["speed"]), 2),
        })

    return keyframes


def _extract_npc_keyframes(json_path: Path, max_npcs: int = 5) -> list:
    """Extract NPC actors from telemetry.json with keyframes."""
    data = json.loads(json_path.read_text())
    frames = data.get("frames", [])
    if not frames:
        return []

    # Collect all unique actors across frames
    actor_tracks: Dict[str, List[dict]] = {}
    for frame in frames:
        t = frame.get("t_sim", 0.0)
        for actor in frame.get("actors", []):
            role = actor.get("role_name", "")
            if role in ("", "hero", "ego"):
                continue
            # Create unique key from role_name
            key = role
            actor_tracks.setdefault(key, []).append({
                "t": t,
                "x": actor["position"]["x"],
                "y": actor["position"]["y"],
                "speed": actor.get("speed", 0.0),
                "type": actor.get("type", "vehicle"),
            })

    # Build actor entries with sampled keyframes
    actors = []
    seen_ids = set()
    for role, track in sorted(actor_tracks.items(), key=lambda x: -len(x[1]))[:max_npcs]:
        # Deduplicate IDs
        actor_id = role
        suffix = 2
        while actor_id in seen_ids:
            actor_id = f"{role}_{suffix}"
            suffix += 1
        seen_ids.add(actor_id)

        # Sample 5-8 keyframes
        n_kf = min(8, max(3, len(track) // 20))
        step = max(1, len(track) // n_kf)
        keyframes = []
        for i in range(0, len(track), step):
            pt = track[i]
            keyframes.append({
                "t": round(pt["t"], 2),
                "x": round(pt["x"], 3),
                "y": round(pt["y"], 3),
                "v": round(pt["speed"], 2),
            })
        if keyframes:
            kind = track[0].get("type", "vehicle")
            actors.append({
                "id": actor_id,
                "kind": kind,
                "role": "npc",
                "blueprint": "vehicle.tesla.model3" if kind == "vehicle" else "walker.pedestrian.0001",
                "controller": "teleport",
                "color": "#6b9080",
                "keyframes": keyframes,
            })

    return actors


def _load_events(events_path: Path) -> list:
    """Load events from events.json."""
    if not events_path.exists():
        return []
    data = json.loads(events_path.read_text())
    events_raw = data if isinstance(data, list) else data.get("events", [])
    events = []
    for evt in events_raw:
        events.append({
            "t_event": evt.get("t_event", evt.get("t", 0)),
            "event_type": evt.get("event_type", evt.get("type", "unknown")),
            "expected_action": evt.get("expected_action", evt.get("action", "")),
            "audio_id": evt.get("audio_id", ""),
            "t_voice_start": evt.get("t_voice_start", max(0, evt.get("t_event", 0) - 3)),
            "t_robot_precue": evt.get("t_robot_precue", max(0, evt.get("t_event", 0) - 3.5)),
        })
    return events


def convert_telemetry(
    csv_path: Path,
    json_path: Path,
    events_path: Path,
    output_path: Path,
    town: str = "Town05",
    duration: float = 60.0,
    dt: float = 0.05,
) -> dict:
    """Convert telemetry files to scene_edit.json format."""
    ego_keyframes = _sample_ego_keyframes(csv_path)
    npc_actors = _extract_npc_keyframes(json_path) if json_path.exists() else []
    events = _load_events(events_path) if events_path.exists() else []

    # Determine duration from CSV if not specified
    if ego_keyframes:
        actual_duration = max(kf["t"] for kf in ego_keyframes)
        if actual_duration > duration:
            duration = actual_duration

    scene = {
        "version": "0.1",
        "episode_id": output_path.stem if output_path.stem != "scene_edit" else output_path.parent.name,
        "town": town,
        "dt": dt,
        "duration": duration,
        "map_dir": f"data/maps/{town}",
        "seed": 0,
        "actors": [
            {
                "id": "ego",
                "kind": "vehicle",
                "role": "ego",
                "blueprint": "vehicle.tesla.model3",
                "controller": "teleport",
                "color": "#d1495b",
                "keyframes": ego_keyframes,
            },
            *npc_actors,
        ],
        "events": events,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(scene, indent=2))
    print(f"Saved scene_edit.json to {output_path}")
    print(f"  Ego: {len(ego_keyframes)} keyframes, NPCs: {len(npc_actors)}, Events: {len(events)}")
    return scene


def main(argv=None):
    parser = argparse.ArgumentParser(description="Convert telemetry to scene_edit.json.")
    parser.add_argument("--csv", required=True, help="Path to telemetry.csv")
    parser.add_argument("--json", required=True, help="Path to telemetry.json")
    parser.add_argument("--events", required=True, help="Path to events.json")
    parser.add_argument("--output", required=True, help="Output scene_edit.json path")
    parser.add_argument("--town", default="Town05", help="CARLA town name")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--dt", type=float, default=0.05)
    args = parser.parse_args(argv)

    convert_telemetry(
        csv_path=Path(args.csv),
        json_path=Path(args.json),
        events_path=Path(args.events),
        output_path=Path(args.output),
        town=args.town,
        duration=args.duration,
        dt=args.dt,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
