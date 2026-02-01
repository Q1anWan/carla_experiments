"""Trajectory replay controllers (teleport + control placeholders)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

try:
    import carla
except ImportError:  # pragma: no cover
    carla = None

from ..planning.trajectory_schema import TrajectoryPoint


# DocRef: technical_details.md#5
@dataclass
class TeleportFollower:
    actor: "carla.Actor"
    trajectory: List[TrajectoryPoint]
    map_obj: "carla.Map"
    lane_type: "carla.LaneType"
    _last_z: float = 0.0
    _initialised: bool = False

    def apply(self, frame_index: int) -> None:
        if not self.trajectory:
            return

        # Disable physics on first call to prevent gravity / momentum artefacts
        if not self._initialised:
            self.actor.set_simulate_physics(False)
            self._initialised = True

        idx = frame_index if frame_index < len(self.trajectory) else len(self.trajectory) - 1
        point = self.trajectory[idx]

        # --- Z disambiguation: prefer continuity over nearest-waypoint ---
        loc = carla.Location(x=point.x, y=point.y, z=self._last_z)
        wp = self.map_obj.get_waypoint(
            loc, project_to_road=True, lane_type=self.lane_type,
        )
        z_candidate = wp.transform.location.z if wp else self._last_z
        # Reject large z jumps (bridge / tunnel ambiguity): keep last_z
        if abs(z_candidate - self._last_z) > 4.0 and frame_index > 0:
            z = self._last_z
        else:
            z = z_candidate
        self._last_z = z

        transform = carla.Transform(
            carla.Location(x=point.x, y=point.y, z=z + 0.05),
            carla.Rotation(yaw=point.yaw),
        )
        self.actor.set_transform(transform)

        # Clear residual velocity / angular velocity
        if hasattr(self.actor, "set_target_velocity"):
            self.actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        if hasattr(self.actor, "set_target_angular_velocity"):
            self.actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))


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
