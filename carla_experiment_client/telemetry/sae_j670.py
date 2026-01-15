"""SAE J670 coordinate system transformer.

SAE J670 (Vehicle Dynamics Terminology) defines a vehicle-fixed coordinate system:
- X-axis: Points forward (front of vehicle is positive)
- Y-axis: Points left (left side of vehicle is positive)
- Z-axis: Points up (top of vehicle is positive)
- Origin: Vehicle center of mass (approximated at vehicle center)

CARLA uses a different convention:
- X-axis: Points forward
- Y-axis: Points right
- Z-axis: Points up

The main difference is the Y-axis direction:
- CARLA: Y points RIGHT
- SAE J670: Y points LEFT

Therefore: SAE_Y = -CARLA_Y
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

try:
    import carla
except ImportError:
    carla = None  # Allow module to be imported without CARLA


@dataclass
class VehicleState:
    """Vehicle state in SAE J670 coordinate system."""

    # World position (CARLA world coordinates)
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0

    # Vehicle-relative velocity (SAE J670: front-left-up)
    vx: float = 0.0  # Longitudinal velocity (m/s, positive = forward)
    vy: float = 0.0  # Lateral velocity (m/s, positive = left)
    vz: float = 0.0  # Vertical velocity (m/s, positive = up)

    # Vehicle-relative acceleration (SAE J670: front-left-up)
    ax: float = 0.0  # Longitudinal acceleration (m/s^2, positive = forward)
    ay: float = 0.0  # Lateral acceleration (m/s^2, positive = left)
    az: float = 0.0  # Vertical acceleration (m/s^2, positive = up)

    # Angular velocity (deg/s)
    roll_rate: float = 0.0  # Roll rate (deg/s, positive = right side up)
    pitch_rate: float = 0.0  # Pitch rate (deg/s, positive = nose up)
    yaw_rate: float = 0.0  # Yaw rate (deg/s, positive = counter-clockwise from above)

    # Euler angles (degrees)
    roll: float = 0.0  # Roll angle (deg)
    pitch: float = 0.0  # Pitch angle (deg)
    yaw: float = 0.0  # Yaw angle (deg)

    # Speed magnitude
    speed: float = 0.0  # Total speed (m/s)

    # Control inputs
    throttle: float = 0.0  # Throttle input [0, 1]
    brake: float = 0.0  # Brake input [0, 1]
    steer: float = 0.0  # Steering input [-1, 1]

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "position": {
                "x": round(self.world_x, 3),
                "y": round(self.world_y, 3),
                "z": round(self.world_z, 3),
            },
            "velocity": {
                "vx": round(self.vx, 4),
                "vy": round(self.vy, 4),
                "vz": round(self.vz, 4),
            },
            "acceleration": {
                "ax": round(self.ax, 4),
                "ay": round(self.ay, 4),
                "az": round(self.az, 4),
            },
            "angular_velocity": {
                "roll_rate": round(self.roll_rate, 4),
                "pitch_rate": round(self.pitch_rate, 4),
                "yaw_rate": round(self.yaw_rate, 4),
            },
            "orientation": {
                "roll": round(self.roll, 3),
                "pitch": round(self.pitch, 3),
                "yaw": round(self.yaw, 3),
            },
            "speed": round(self.speed, 3),
            "control": {
                "throttle": round(self.throttle, 3),
                "brake": round(self.brake, 3),
                "steer": round(self.steer, 3),
            },
        }

    def to_csv_row(self) -> list:
        """Convert to flat list for CSV output."""
        return [
            self.world_x,
            self.world_y,
            self.world_z,
            self.vx,
            self.vy,
            self.vz,
            self.ax,
            self.ay,
            self.az,
            self.roll_rate,
            self.pitch_rate,
            self.yaw_rate,
            self.roll,
            self.pitch,
            self.yaw,
            self.speed,
            self.throttle,
            self.brake,
            self.steer,
        ]

    @staticmethod
    def csv_header() -> list:
        """Return CSV column headers."""
        return [
            "world_x",
            "world_y",
            "world_z",
            "vx",
            "vy",
            "vz",
            "ax",
            "ay",
            "az",
            "roll_rate",
            "pitch_rate",
            "yaw_rate",
            "roll",
            "pitch",
            "yaw",
            "speed",
            "throttle",
            "brake",
            "steer",
        ]


class SAEJ670Transformer:
    """Transform CARLA vehicle data to SAE J670 coordinate system.

    This class maintains previous frame state to compute derivatives
    (acceleration, angular velocity) from position and velocity data.
    """

    def __init__(self):
        self._prev_velocity: Optional[Any] = None  # carla.Vector3D
        self._prev_rotation: Optional[Any] = None  # carla.Rotation
        self._prev_transform: Optional[Any] = None  # carla.Transform

    def reset(self) -> None:
        """Reset state for new recording."""
        self._prev_velocity = None
        self._prev_rotation = None
        self._prev_transform = None

    def compute_state(
        self,
        vehicle: Any,  # carla.Vehicle
        dt: float,
    ) -> VehicleState:
        """Compute vehicle state in SAE J670 coordinate system.

        Args:
            vehicle: CARLA vehicle actor
            dt: Time delta since last frame (seconds)

        Returns:
            VehicleState with all computed values
        """
        if carla is None:
            raise RuntimeError("CARLA module not available")

        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        control = vehicle.get_control()
        location = transform.location
        rotation = transform.rotation

        # Get forward and right vectors for coordinate transformation
        fwd = transform.get_forward_vector()
        right = transform.get_right_vector()

        # Compute velocity in vehicle frame (SAE J670)
        # CARLA velocity is in world frame, project onto vehicle axes
        vx = self._dot(velocity, fwd)  # Longitudinal (forward)
        vy = -self._dot(velocity, right)  # Lateral (SAE: left positive)
        vz = velocity.z  # Vertical (same direction)

        # Compute speed magnitude
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        # Compute acceleration from velocity change
        ax, ay, az = 0.0, 0.0, 0.0
        if self._prev_velocity is not None and dt > 1e-6:
            dv_x = velocity.x - self._prev_velocity.x
            dv_y = velocity.y - self._prev_velocity.y
            dv_z = velocity.z - self._prev_velocity.z

            # Create delta velocity vector for projection
            # Project world-frame acceleration onto vehicle axes
            # ax: longitudinal acceleration (forward direction)
            ax = (fwd.x * dv_x + fwd.y * dv_y + fwd.z * dv_z) / dt
            # ay: lateral acceleration (SAE: left positive, so negate right projection)
            ay = -(right.x * dv_x + right.y * dv_y + right.z * dv_z) / dt
            # az: vertical acceleration
            az = dv_z / dt

        # Compute angular velocity from rotation change
        roll_rate, pitch_rate, yaw_rate = 0.0, 0.0, 0.0
        if self._prev_rotation is not None and dt > 1e-6:
            # Handle angle wrap-around for yaw
            d_roll = rotation.roll - self._prev_rotation.roll
            d_pitch = rotation.pitch - self._prev_rotation.pitch
            d_yaw = self._normalize_angle(rotation.yaw - self._prev_rotation.yaw)

            roll_rate = d_roll / dt
            pitch_rate = d_pitch / dt
            yaw_rate = d_yaw / dt

        # Store current state for next iteration
        self._prev_velocity = velocity
        self._prev_rotation = rotation
        self._prev_transform = transform

        return VehicleState(
            world_x=location.x,
            world_y=location.y,
            world_z=location.z,
            vx=vx,
            vy=vy,
            vz=vz,
            ax=ax,
            ay=ay,
            az=az,
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            roll=rotation.roll,
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            speed=speed,
            throttle=control.throttle,
            brake=control.brake,
            steer=control.steer,
        )

    @staticmethod
    def _dot(v1: Any, v2: Any) -> float:
        """Compute dot product of two CARLA vectors."""
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] range."""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle
