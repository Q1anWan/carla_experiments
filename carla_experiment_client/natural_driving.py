"""Natural driving behavior configuration for human-like autonomous driving.

This module provides configuration presets and utilities to make CARLA's Traffic
Manager behave more like a human driver, reducing jerky motions and improving
predictability.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, Any, Optional

import carla


@dataclass
class NaturalDrivingConfig:
    """Configuration for natural (human-like) driving behavior.

    These parameters are tuned based on human factors research to create
    comfortable, predictable driving behavior.
    """

    # Speed behavior
    speed_delta_percent: float = 10.0  # Slightly slower than limit (more human-like)
    speed_variance_percent: float = 5.0  # Natural speed variation

    # Following behavior
    desired_headway_s: float = 1.8  # Time headway (human: 1.5-2.5s)
    min_follow_distance_m: float = 5.0  # Minimum following distance

    # Comfort thresholds (based on ISO 2631-1)
    max_comfort_accel_mps2: float = 2.0  # Comfortable acceleration
    max_comfort_decel_mps2: float = 3.0  # Comfortable deceleration
    max_comfort_jerk_mps3: float = 1.5  # Comfortable jerk

    # Lane change behavior
    lane_change_distance_m: float = 30.0  # Min distance for safe lane change
    auto_lane_change: bool = True  # Allow automatic lane changes

    # Traffic light behavior
    ignore_lights_percent: float = 0.0  # Compliance with traffic lights
    yellow_light_reaction: str = "cautious"  # "cautious" or "aggressive"

    # Intersection behavior
    ignore_vehicles_percent: float = 0.0  # Safety margin at intersections

    # Smoothness parameters
    throttle_smoothing: float = 0.8  # Throttle input smoothing (0-1)
    brake_smoothing: float = 0.7  # Brake input smoothing (0-1)
    steering_smoothing: float = 0.9  # Steering input smoothing (0-1)


# Preset configurations for different driving styles
DRIVING_PRESETS: Dict[str, NaturalDrivingConfig] = {
    "relaxed": NaturalDrivingConfig(
        speed_delta_percent=15.0,
        desired_headway_s=2.2,
        max_comfort_accel_mps2=1.5,
        max_comfort_decel_mps2=2.5,
        max_comfort_jerk_mps3=1.0,
        lane_change_distance_m=40.0,
    ),
    "normal": NaturalDrivingConfig(
        speed_delta_percent=10.0,
        desired_headway_s=1.8,
        max_comfort_accel_mps2=2.0,
        max_comfort_decel_mps2=3.0,
        max_comfort_jerk_mps3=1.5,
        lane_change_distance_m=30.0,
    ),
    "assertive": NaturalDrivingConfig(
        speed_delta_percent=5.0,
        desired_headway_s=1.5,
        max_comfort_accel_mps2=2.5,
        max_comfort_decel_mps2=3.5,
        max_comfort_jerk_mps3=2.0,
        lane_change_distance_m=25.0,
    ),
}


def apply_natural_driving_config(
    tm: carla.TrafficManager,
    vehicle: carla.Vehicle,
    config: Optional[NaturalDrivingConfig] = None,
    preset: str = "normal",
) -> None:
    """Apply natural driving configuration to a vehicle via Traffic Manager.

    Args:
        tm: CARLA Traffic Manager instance
        vehicle: Vehicle to configure
        config: Optional explicit configuration (overrides preset)
        preset: Preset name ("relaxed", "normal", "assertive")
    """
    if config is None:
        config = DRIVING_PRESETS.get(preset, DRIVING_PRESETS["normal"])

    try:
        # Speed configuration
        tm.vehicle_percentage_speed_difference(vehicle, config.speed_delta_percent)
    except RuntimeError as e:
        logging.warning("Failed to set speed delta: %s", e)

    try:
        # Following distance (maps to headway behavior)
        tm.distance_to_leading_vehicle(vehicle, config.min_follow_distance_m)
    except (RuntimeError, AttributeError) as e:
        logging.warning("Failed to set follow distance: %s", e)

    try:
        # Lane change behavior
        tm.auto_lane_change(vehicle, config.auto_lane_change)
    except (RuntimeError, AttributeError) as e:
        logging.warning("Failed to set auto lane change: %s", e)

    try:
        # Traffic light compliance
        tm.ignore_lights_percentage(vehicle, config.ignore_lights_percent)
    except (RuntimeError, AttributeError) as e:
        logging.warning("Failed to set ignore lights: %s", e)

    try:
        # Intersection safety
        tm.ignore_vehicles_percentage(vehicle, config.ignore_vehicles_percent)
    except (RuntimeError, AttributeError) as e:
        logging.warning("Failed to set ignore vehicles: %s", e)

    logging.info(
        "Applied natural driving config (preset=%s) to vehicle %d: "
        "speed_delta=%.1f%%, headway=%.1fs, follow_dist=%.1fm",
        preset,
        vehicle.id,
        config.speed_delta_percent,
        config.desired_headway_s,
        config.min_follow_distance_m,
    )


def configure_global_tm_behavior(
    tm: carla.TrafficManager,
    config: Optional[NaturalDrivingConfig] = None,
    preset: str = "normal",
) -> None:
    """Configure global Traffic Manager behavior for more natural driving.

    Args:
        tm: CARLA Traffic Manager instance
        config: Optional explicit configuration
        preset: Preset name
    """
    if config is None:
        config = DRIVING_PRESETS.get(preset, DRIVING_PRESETS["normal"])

    try:
        # Global distance to leading vehicle
        tm.set_global_distance_to_leading_vehicle(config.min_follow_distance_m)
    except (RuntimeError, AttributeError) as e:
        logging.warning("Failed to set global follow distance: %s", e)

    try:
        # Hybrid physics mode for smoother motion (if available)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(70.0)  # Apply within 70m of hero vehicle
    except (RuntimeError, AttributeError) as e:
        logging.debug("Hybrid physics mode not available: %s", e)

    try:
        # Respawn dormant vehicles (for traffic flow)
        tm.set_respawn_dormant_vehicles(True)
    except (RuntimeError, AttributeError) as e:
        logging.debug("Respawn dormant vehicles not available: %s", e)

    logging.info(
        "Configured global TM behavior: preset=%s, follow_dist=%.1fm",
        preset,
        config.min_follow_distance_m,
    )


@dataclass
class SmoothController:
    """Applies smoothing to vehicle control inputs for natural motion.

    This controller wraps raw control inputs and applies exponential smoothing
    to create more human-like, gradual control changes.
    """

    throttle_alpha: float = 0.3  # Lower = smoother
    brake_alpha: float = 0.4
    steer_alpha: float = 0.2

    _prev_throttle: float = 0.0
    _prev_brake: float = 0.0
    _prev_steer: float = 0.0

    def smooth(self, control: carla.VehicleControl) -> carla.VehicleControl:
        """Apply smoothing to control input.

        Args:
            control: Raw vehicle control

        Returns:
            Smoothed vehicle control
        """
        # Exponential moving average smoothing
        smooth_throttle = (
            self.throttle_alpha * control.throttle
            + (1 - self.throttle_alpha) * self._prev_throttle
        )
        smooth_brake = (
            self.brake_alpha * control.brake
            + (1 - self.brake_alpha) * self._prev_brake
        )
        smooth_steer = (
            self.steer_alpha * control.steer
            + (1 - self.steer_alpha) * self._prev_steer
        )

        # Update state
        self._prev_throttle = smooth_throttle
        self._prev_brake = smooth_brake
        self._prev_steer = smooth_steer

        # Create smoothed control
        smoothed = carla.VehicleControl(
            throttle=smooth_throttle,
            brake=smooth_brake,
            steer=smooth_steer,
            hand_brake=control.hand_brake,
            reverse=control.reverse,
            manual_gear_shift=control.manual_gear_shift,
            gear=control.gear,
        )
        return smoothed

    def reset(self) -> None:
        """Reset controller state."""
        self._prev_throttle = 0.0
        self._prev_brake = 0.0
        self._prev_steer = 0.0


def create_comfort_brake_profile(
    current_speed: float,
    target_speed: float,
    distance: float,
    max_decel: float = 3.0,
) -> float:
    """Calculate comfortable brake value for smooth deceleration.

    Uses constant deceleration profile to achieve target speed over distance.

    Args:
        current_speed: Current speed in m/s
        target_speed: Target speed in m/s
        distance: Distance to target in meters
        max_decel: Maximum comfortable deceleration in m/s²

    Returns:
        Brake value (0.0 to 1.0)
    """
    if current_speed <= target_speed:
        return 0.0

    if distance <= 0:
        return 1.0

    # Required deceleration: v² = v0² + 2*a*d => a = (v² - v0²) / (2*d)
    speed_diff_sq = current_speed**2 - target_speed**2
    required_decel = speed_diff_sq / (2 * distance)

    # Map to brake value (assuming linear relationship)
    # Typical vehicle: 1.0 brake ≈ 8-10 m/s² deceleration
    brake_value = required_decel / 8.0

    # Clamp to comfortable range
    comfort_brake_max = max_decel / 8.0
    brake_value = min(brake_value, comfort_brake_max)
    brake_value = max(0.0, min(1.0, brake_value))

    return brake_value
