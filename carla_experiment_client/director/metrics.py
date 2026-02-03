"""Per-tick pairwise actor metrics for trigger evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

try:
    import carla
except ImportError:
    carla = None


@dataclass
class ActorMetrics:
    """Metrics between ego and a target actor at a single tick."""

    gap_m: float  # longitudinal bumper-to-bumper gap (positive = target ahead)
    ttc_s: float  # time-to-collision; inf if diverging
    lateral_offset_m: float  # signed lateral offset (positive = target to right)
    ego_speed_mps: float
    target_speed_mps: float
    relative_speed_mps: float  # closing speed (positive = approaching)
    elapsed_s: float  # scenario elapsed time


def _get_speed(vehicle: Any) -> float:
    """Get scalar speed from CARLA vehicle velocity vector."""
    vel = vehicle.get_velocity()
    return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def _forward_speed(vehicle: Any) -> float:
    """Get speed projected onto vehicle forward direction."""
    vel = vehicle.get_velocity()
    fwd = vehicle.get_transform().get_forward_vector()
    return vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z


_half_length_cache: dict[int, float] = {}


def _vehicle_half_length(vehicle: Any) -> float:
    """Get half of vehicle bounding box extent along X (cached)."""
    vid = vehicle.id
    if vid in _half_length_cache:
        return _half_length_cache[vid]
    try:
        bb = vehicle.bounding_box
        val = bb.extent.x
    except (AttributeError, RuntimeError):
        val = 2.0  # default ~4m car
    _half_length_cache[vid] = val
    return val


def compute_metrics(
    ego: Any,
    target: Any,
    map_obj: Any,
    frame_index: int,
    fps: int,
) -> ActorMetrics:
    """Compute pairwise metrics between ego and target vehicle.

    Uses Frenet-like projection along ego's forward axis for gap/lateral
    decomposition, and CARLA velocity vectors for TTC computation.

    Args:
        ego: Ego vehicle (carla.Vehicle)
        target: Target actor (carla.Vehicle or carla.Walker)
        map_obj: CARLA map (carla.Map)
        frame_index: Current frame number
        fps: Frames per second
    """
    ego_transform = ego.get_transform()
    target_transform = target.get_transform()

    ego_loc = ego_transform.location
    target_loc = target_transform.location

    # Vector from ego to target
    dx = target_loc.x - ego_loc.x
    dy = target_loc.y - ego_loc.y

    # Ego forward and right vectors
    fwd = ego_transform.get_forward_vector()
    # Right vector: rotate forward 90 degrees CW in XY plane
    right_x = -fwd.y
    right_y = fwd.x

    # Project ego→target onto ego's forward/right axes
    longitudinal = dx * fwd.x + dy * fwd.y
    lateral = dx * right_x + dy * right_y

    # Bumper-to-bumper gap (subtract half-lengths)
    ego_half = _vehicle_half_length(ego)
    target_half = _vehicle_half_length(target)
    gap = abs(longitudinal) - ego_half - target_half
    gap = max(0.0, gap)

    # Speeds
    ego_speed = _get_speed(ego)
    target_speed = _get_speed(target)

    # Forward speeds for TTC
    ego_fwd_speed = _forward_speed(ego)
    target_fwd_vel = target.get_velocity()
    if target_fwd_vel is not None:
        target_fwd_speed = (
            target_fwd_vel.x * fwd.x + target_fwd_vel.y * fwd.y + target_fwd_vel.z * fwd.z
        )
    else:
        target_fwd_speed = 0.0

    # Closing speed (positive means approaching)
    if longitudinal >= 0:
        # Target ahead: closing = ego_speed - target_speed (in ego forward dir)
        closing_speed = ego_fwd_speed - target_fwd_speed
    else:
        # Target behind: closing = target_speed - ego_speed
        closing_speed = target_fwd_speed - ego_fwd_speed

    # TTC
    if closing_speed > 0.1 and gap > 0:
        ttc = gap / closing_speed
    else:
        ttc = float("inf")

    return ActorMetrics(
        gap_m=round(gap, 3),
        ttc_s=round(ttc, 3) if ttc != float("inf") else float("inf"),
        lateral_offset_m=round(lateral, 3),
        ego_speed_mps=round(ego_speed, 3),
        target_speed_mps=round(target_speed, 3),
        relative_speed_mps=round(closing_speed, 3),
        elapsed_s=round(frame_index / max(1, fps), 3),
    )
