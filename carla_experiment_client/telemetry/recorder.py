"""Telemetry recorder for per-tick vehicle and actor state logging.

Records detailed telemetry data including:
- Ego vehicle state (SAE J670 coordinate system)
- Tracked actor states
- Control inputs
- Event triggers

Outputs both JSON and CSV formats by default.
"""

from __future__ import annotations

import csv
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from .sae_j670 import SAEJ670Transformer, VehicleState

try:
    import carla
except ImportError:
    carla = None


@dataclass
class TelemetryConfig:
    """Configuration for telemetry recording."""

    enabled: bool = True
    output_json: bool = True
    output_csv: bool = True
    include_actors: bool = True
    actor_radius_m: float = 50.0  # Only record actors within this radius


@dataclass
class TelemetryFrame:
    """Single frame of telemetry data."""

    frame: int
    t_sim: float  # Simulation time (frame / fps)
    t_world: float  # World elapsed time from snapshot
    dt: float  # Delta time
    ego: Dict[str, Any] = field(default_factory=dict)
    actors: List[Dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "frame": self.frame,
            "t_sim": round(self.t_sim, 3),
            "t_world": round(self.t_world, 3),
            "dt": round(self.dt, 4),
            "ego": self.ego,
            "actors": self.actors,
        }


# DocRef: technical_details.md#4.3
class TelemetryRecorder:
    """Records per-tick telemetry data with SAE J670 coordinate system support.

    Usage:
        recorder = TelemetryRecorder(world, ego_vehicle, fps=20)
        for frame in range(total_frames):
            world.tick()
            recorder.tick(snapshot, frame)
        recorder.save(output_dir)
    """

    def __init__(
        self,
        world: Any,  # carla.World
        ego_vehicle: Any,  # carla.Vehicle
        fps: int = 20,
        config: Optional[TelemetryConfig] = None,
        tracked_actors: Optional[List[Any]] = None,
    ):
        """Initialize telemetry recorder.

        Args:
            world: CARLA world instance
            ego_vehicle: Ego vehicle actor
            fps: Frames per second for time calculations
            config: Telemetry configuration (uses defaults if None)
            tracked_actors: Optional list of actors to track (if None, tracks nearby)
        """
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.fps = fps
        self.config = config or TelemetryConfig()
        self.tracked_actors = tracked_actors or []

        self._transformer = SAEJ670Transformer()
        self._frames: List[TelemetryFrame] = []
        self._ego_states: List[VehicleState] = []
        self._prev_time: Optional[float] = None

    def reset(self) -> None:
        """Reset recorder state for new recording."""
        self._transformer.reset()
        self._frames = []
        self._ego_states = []
        self._prev_time = None

    def tick(
        self,
        snapshot: Any,  # carla.WorldSnapshot
        frame_index: int,
    ) -> TelemetryFrame:
        """Record telemetry for current frame.

        Args:
            snapshot: World snapshot from world.tick()
            frame_index: Current frame number

        Returns:
            TelemetryFrame with recorded data
        """
        if not self.config.enabled:
            return TelemetryFrame(frame=frame_index, t_sim=0, t_world=0, dt=0)

        t_world = snapshot.timestamp.elapsed_seconds
        dt = snapshot.timestamp.delta_seconds
        t_sim = frame_index / self.fps

        # Compute ego state using SAE J1100 transformer
        ego_state = self._transformer.compute_state(self.ego_vehicle, dt)
        self._ego_states.append(ego_state)

        # Record nearby actors if configured
        actor_records = []
        if self.config.include_actors:
            actor_records = self._record_actors(ego_state)

        frame_data = TelemetryFrame(
            frame=frame_index,
            t_sim=t_sim,
            t_world=t_world,
            dt=dt,
            ego=ego_state.to_dict(),
            actors=actor_records,
        )
        self._frames.append(frame_data)
        self._prev_time = t_world

        return frame_data

    def _record_actors(self, ego_state: VehicleState) -> List[Dict[str, Any]]:
        """Record state of nearby actors."""
        records = []
        ego_loc = carla.Location(
            x=ego_state.world_x,
            y=ego_state.world_y,
            z=ego_state.world_z,
        )

        # If tracked_actors is specified, use that list
        if self.tracked_actors:
            actors_to_record = self.tracked_actors
        else:
            # Otherwise, find nearby vehicles and walkers
            actors_to_record = []
            for actor in self.world.get_actors():
                if actor.id == self.ego_vehicle.id:
                    continue
                if not hasattr(actor, "get_location"):
                    continue
                actor_loc = actor.get_location()
                dist = actor_loc.distance(ego_loc)
                if dist <= self.config.actor_radius_m:
                    actors_to_record.append(actor)

        for actor in actors_to_record:
            if actor.id == self.ego_vehicle.id:
                continue
            try:
                record = self._record_actor(actor, ego_loc)
                if record:
                    records.append(record)
            except RuntimeError:
                # Actor may have been destroyed
                continue

        return records

    def _record_actor(
        self,
        actor: Any,
        ego_loc: Any,  # carla.Location
    ) -> Optional[Dict[str, Any]]:
        """Record state of a single actor."""
        try:
            transform = actor.get_transform()
            location = transform.location
            rotation = transform.rotation

            # Get actor type
            actor_type = "unknown"
            type_id = actor.type_id
            if "vehicle" in type_id:
                actor_type = "vehicle"
            elif "walker" in type_id:
                actor_type = "walker"
            elif "traffic" in type_id:
                actor_type = "traffic_light"

            # Compute distance to ego
            dist = location.distance(ego_loc)

            record = {
                "id": actor.id,
                "type": actor_type,
                "type_id": type_id,
                "role_name": actor.attributes.get("role_name", ""),
                "position": {
                    "x": round(location.x, 3),
                    "y": round(location.y, 3),
                    "z": round(location.z, 3),
                },
                "rotation": {
                    "roll": round(rotation.roll, 2),
                    "pitch": round(rotation.pitch, 2),
                    "yaw": round(rotation.yaw, 2),
                },
                "distance_to_ego": round(dist, 2),
            }

            # Add velocity for vehicles and walkers
            if hasattr(actor, "get_velocity"):
                vel = actor.get_velocity()
                speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
                record["velocity"] = {
                    "x": round(vel.x, 3),
                    "y": round(vel.y, 3),
                    "z": round(vel.z, 3),
                }
                record["speed"] = round(speed, 3)

            return record

        except RuntimeError:
            return None

    def save(self, output_dir: Path) -> Dict[str, Path]:
        """Save telemetry data to files.

        Args:
            output_dir: Directory to save files

        Returns:
            Dict mapping format name to file path
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        saved = {}

        if self.config.output_json:
            json_path = output_dir / "telemetry.json"
            self._save_json(json_path)
            saved["json"] = json_path
            logging.info("Saved telemetry JSON: %s", json_path)

        if self.config.output_csv:
            csv_path = output_dir / "telemetry.csv"
            self._save_csv(csv_path)
            saved["csv"] = csv_path
            logging.info("Saved telemetry CSV: %s", csv_path)

        return saved

    def _save_json(self, path: Path) -> None:
        """Save telemetry as JSON."""
        data = {
            "metadata": {
                "fps": self.fps,
                "total_frames": len(self._frames),
                "coordinate_system": "SAE_J670",
                "coordinate_description": {
                    "X": "Forward (positive = front of vehicle)",
                    "Y": "Left (positive = left side of vehicle)",
                    "Z": "Up (positive = top of vehicle)",
                },
                "units": {
                    "position": "meters",
                    "velocity": "m/s",
                    "acceleration": "m/s^2",
                    "angular_velocity": "deg/s",
                    "angles": "degrees",
                },
            },
            "frames": [f.to_dict() for f in self._frames],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def _save_csv(self, path: Path) -> None:
        """Save ego vehicle telemetry as CSV."""
        if not self._ego_states:
            return

        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)

            # Write header
            header = ["frame", "t_sim", "t_world", "dt"] + VehicleState.csv_header()
            writer.writerow(header)

            # Write data rows
            for i, (frame_data, ego_state) in enumerate(
                zip(self._frames, self._ego_states)
            ):
                row = [
                    frame_data.frame,
                    round(frame_data.t_sim, 3),
                    round(frame_data.t_world, 3),
                    round(frame_data.dt, 4),
                ] + [round(v, 6) if isinstance(v, float) else v for v in ego_state.to_csv_row()]
                writer.writerow(row)

    def get_summary(self) -> Dict[str, Any]:
        """Get summary statistics of recorded telemetry."""
        if not self._ego_states:
            return {}

        speeds = [s.speed for s in self._ego_states]
        ax_vals = [s.ax for s in self._ego_states]
        ay_vals = [s.ay for s in self._ego_states]

        return {
            "total_frames": len(self._frames),
            "duration_s": len(self._frames) / self.fps if self.fps > 0 else 0,
            "speed": {
                "min": round(min(speeds), 2),
                "max": round(max(speeds), 2),
                "avg": round(sum(speeds) / len(speeds), 2),
            },
            "acceleration_longitudinal": {
                "min": round(min(ax_vals), 2),
                "max": round(max(ax_vals), 2),
            },
            "acceleration_lateral": {
                "min": round(min(ay_vals), 2),
                "max": round(max(ay_vals), 2),
            },
        }
