"""Trajectory replay controllers (teleport + control placeholders)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..planning.trajectory_schema import TrajectoryPoint


@dataclass
class TeleportFollower:
    actor: "carla.Actor"
    trajectory: List[TrajectoryPoint]
    map_obj: "carla.Map"
    lane_type: "carla.LaneType"

    def apply(self, frame_index: int) -> None:
        if not self.trajectory:
            return
        idx = frame_index if frame_index < len(self.trajectory) else len(self.trajectory) - 1
        point = self.trajectory[idx]
        loc = carla.Location(x=point.x, y=point.y, z=0.0)
        wp = self.map_obj.get_waypoint(loc, project_to_road=True, lane_type=self.lane_type)
        z = wp.transform.location.z if wp else 0.0
        transform = carla.Transform(
            carla.Location(x=point.x, y=point.y, z=z + 0.2),
            carla.Rotation(yaw=point.yaw),
        )
        self.actor.set_transform(transform)


def build_follower(
    *,
    actor: "carla.Actor",
    trajectory: List[TrajectoryPoint],
    map_obj: "carla.Map",
    lane_type: "carla.LaneType",
    controller: str,
) -> TeleportFollower:
    # MVP: all controllers use teleport replay for deterministic playback.
    return TeleportFollower(
        actor=actor,
        trajectory=trajectory,
        map_obj=map_obj,
        lane_type=lane_type,
    )
