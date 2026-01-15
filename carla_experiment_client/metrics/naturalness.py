"""Natural driving behavior metrics for human-like autonomous driving evaluation.

This module implements metrics to evaluate how "human-like" the autonomous driving
behavior is, focusing on comfort, predictability, and social driving norms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any

import carla


@dataclass
class NaturalnessMetrics:
    """Computed naturalness metrics for a driving episode."""

    # Safety metrics
    collision_count: int = 0
    min_ttc: float = float("inf")
    hard_brake_count: int = 0
    min_headway_s: float = float("inf")

    # Compliance metrics
    speed_limit_violation_ratio: float = 0.0
    lane_boundary_cross_count: int = 0
    red_light_violation_count: int = 0

    # Comfort metrics (key for natural driving)
    jerk_rms_longitudinal: float = 0.0
    jerk_rms_lateral: float = 0.0
    jerk_max_longitudinal: float = 0.0
    jerk_max_lateral: float = 0.0
    accel_rms_longitudinal: float = 0.0
    accel_rms_lateral: float = 0.0
    steering_rate_rms: float = 0.0

    # Predictability metrics
    speed_oscillation_score: float = 0.0
    lane_center_oscillation: float = 0.0
    action_smoothness: float = 0.0

    # Efficiency metrics
    average_speed: float = 0.0
    progress_ratio: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary."""
        return {
            "safety": {
                "collision_count": self.collision_count,
                "min_ttc": self.min_ttc if self.min_ttc != float("inf") else None,
                "hard_brake_count": self.hard_brake_count,
                "min_headway_s": self.min_headway_s if self.min_headway_s != float("inf") else None,
            },
            "compliance": {
                "speed_limit_violation_ratio": self.speed_limit_violation_ratio,
                "lane_boundary_cross_count": self.lane_boundary_cross_count,
                "red_light_violation_count": self.red_light_violation_count,
            },
            "comfort": {
                "jerk_rms_longitudinal": self.jerk_rms_longitudinal,
                "jerk_rms_lateral": self.jerk_rms_lateral,
                "jerk_max_longitudinal": self.jerk_max_longitudinal,
                "jerk_max_lateral": self.jerk_max_lateral,
                "accel_rms_longitudinal": self.accel_rms_longitudinal,
                "accel_rms_lateral": self.accel_rms_lateral,
                "steering_rate_rms": self.steering_rate_rms,
            },
            "predictability": {
                "speed_oscillation_score": self.speed_oscillation_score,
                "lane_center_oscillation": self.lane_center_oscillation,
                "action_smoothness": self.action_smoothness,
            },
            "efficiency": {
                "average_speed": self.average_speed,
                "progress_ratio": self.progress_ratio,
            },
        }

    def compute_naturalness_score(
        self,
        weights: Optional[Dict[str, float]] = None,
    ) -> float:
        """Compute overall naturalness score (0-100, higher is more natural).

        Args:
            weights: Optional weight dictionary for different metric categories
        """
        if weights is None:
            weights = {
                "comfort": 0.35,
                "predictability": 0.25,
                "safety": 0.25,
                "efficiency": 0.15,
            }

        # Comfort score (lower jerk/accel = higher score)
        jerk_penalty = min(1.0, (self.jerk_rms_longitudinal + self.jerk_rms_lateral) / 4.0)
        accel_penalty = min(1.0, (self.accel_rms_longitudinal + self.accel_rms_lateral) / 4.0)
        steer_penalty = min(1.0, self.steering_rate_rms / 0.5)
        comfort_score = 100 * (1.0 - (jerk_penalty + accel_penalty + steer_penalty) / 3.0)

        # Predictability score
        oscillation_penalty = min(1.0, self.speed_oscillation_score / 2.0)
        lane_penalty = min(1.0, self.lane_center_oscillation / 0.5)
        smoothness_bonus = min(1.0, self.action_smoothness)
        predict_score = 100 * (1.0 - (oscillation_penalty + lane_penalty) / 2.0 + smoothness_bonus * 0.2)
        predict_score = min(100, max(0, predict_score))

        # Safety score
        collision_penalty = min(1.0, self.collision_count * 0.5)
        brake_penalty = min(1.0, self.hard_brake_count * 0.1)
        ttc_penalty = 0.0 if self.min_ttc > 3.0 else min(1.0, (3.0 - self.min_ttc) / 3.0)
        safety_score = 100 * (1.0 - (collision_penalty + brake_penalty + ttc_penalty) / 3.0)

        # Efficiency score
        speed_score = min(1.0, self.average_speed / 15.0)  # Normalized to ~15 m/s
        efficiency_score = 100 * (speed_score * 0.5 + self.progress_ratio * 0.5)

        total = (
            weights["comfort"] * comfort_score
            + weights["predictability"] * predict_score
            + weights["safety"] * safety_score
            + weights["efficiency"] * efficiency_score
        )
        return round(total, 2)


@dataclass
class MetricsCollector:
    """Collects per-tick data for computing naturalness metrics."""

    fps: int = 20
    speed_limit_mps: float = 13.89  # 50 km/h default

    # Internal state
    _speeds: List[float] = field(default_factory=list)
    _accels_long: List[float] = field(default_factory=list)
    _accels_lat: List[float] = field(default_factory=list)
    _jerks_long: List[float] = field(default_factory=list)
    _jerks_lat: List[float] = field(default_factory=list)
    _steers: List[float] = field(default_factory=list)
    _steer_rates: List[float] = field(default_factory=list)
    _lane_offsets: List[float] = field(default_factory=list)
    _ttcs: List[float] = field(default_factory=list)
    _headways: List[float] = field(default_factory=list)

    _prev_speed: float = 0.0
    _prev_accel_long: float = 0.0
    _prev_accel_lat: float = 0.0
    _prev_steer: float = 0.0
    _prev_velocity: Optional[carla.Vector3D] = None

    _collision_count: int = 0
    _hard_brake_count: int = 0
    _lane_cross_count: int = 0
    _red_light_violations: int = 0
    _speed_violation_ticks: int = 0
    _total_ticks: int = 0
    _start_location: Optional[carla.Location] = None
    _end_location: Optional[carla.Location] = None

    def tick(
        self,
        ego_vehicle: carla.Vehicle,
        world: carla.World,
        map_obj: carla.Map,
    ) -> None:
        """Record metrics for current tick."""
        self._total_ticks += 1
        dt = 1.0 / max(1, self.fps)

        # Get vehicle state
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()
        control = ego_vehicle.get_control()
        location = transform.location

        if self._start_location is None:
            self._start_location = location
        self._end_location = location

        # Speed
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self._speeds.append(speed)

        # Longitudinal acceleration (along vehicle forward direction)
        fwd = transform.get_forward_vector()
        accel_long = 0.0
        if self._prev_velocity is not None:
            dv = carla.Vector3D(
                velocity.x - self._prev_velocity.x,
                velocity.y - self._prev_velocity.y,
                velocity.z - self._prev_velocity.z,
            )
            accel_long = (dv.x * fwd.x + dv.y * fwd.y + dv.z * fwd.z) / dt
        self._accels_long.append(accel_long)

        # Lateral acceleration
        right = self._get_right_vector(transform)
        accel_lat = 0.0
        if self._prev_velocity is not None:
            dv = carla.Vector3D(
                velocity.x - self._prev_velocity.x,
                velocity.y - self._prev_velocity.y,
                velocity.z - self._prev_velocity.z,
            )
            accel_lat = (dv.x * right.x + dv.y * right.y + dv.z * right.z) / dt
        self._accels_lat.append(accel_lat)

        # Jerk (derivative of acceleration)
        jerk_long = (accel_long - self._prev_accel_long) / dt
        jerk_lat = (accel_lat - self._prev_accel_lat) / dt
        self._jerks_long.append(jerk_long)
        self._jerks_lat.append(jerk_lat)

        # Steering rate
        steer = control.steer
        steer_rate = (steer - self._prev_steer) / dt
        self._steers.append(steer)
        self._steer_rates.append(steer_rate)

        # Lane center offset
        waypoint = map_obj.get_waypoint(location, project_to_road=True)
        if waypoint:
            lane_center = waypoint.transform.location
            offset = math.sqrt(
                (location.x - lane_center.x) ** 2 + (location.y - lane_center.y) ** 2
            )
            self._lane_offsets.append(offset)

        # TTC and headway to lead vehicle
        lead_vehicle, lead_dist = self._find_lead_vehicle(ego_vehicle, world, fwd)
        if lead_vehicle and lead_dist > 0:
            lead_vel = lead_vehicle.get_velocity()
            lead_speed = math.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)
            rel_speed = speed - lead_speed
            if rel_speed > 0.1:
                ttc = lead_dist / rel_speed
                self._ttcs.append(ttc)
            if speed > 0.1:
                headway = lead_dist / speed
                self._headways.append(headway)

        # Hard brake detection
        if accel_long < -4.0:  # > 4 m/s² deceleration
            self._hard_brake_count += 1

        # Speed limit violation
        if speed > self.speed_limit_mps:
            self._speed_violation_ticks += 1

        # Update previous state
        self._prev_speed = speed
        self._prev_accel_long = accel_long
        self._prev_accel_lat = accel_lat
        self._prev_steer = steer
        self._prev_velocity = velocity

    def record_collision(self) -> None:
        """Record a collision event."""
        self._collision_count += 1

    def record_lane_cross(self) -> None:
        """Record a lane boundary crossing."""
        self._lane_cross_count += 1

    def record_red_light_violation(self) -> None:
        """Record a red light violation."""
        self._red_light_violations += 1

    def finalize(self) -> NaturalnessMetrics:
        """Compute final metrics from collected data."""
        metrics = NaturalnessMetrics()

        # Safety
        metrics.collision_count = self._collision_count
        metrics.hard_brake_count = self._hard_brake_count
        if self._ttcs:
            metrics.min_ttc = min(self._ttcs)
        if self._headways:
            metrics.min_headway_s = min(self._headways)

        # Compliance
        if self._total_ticks > 0:
            metrics.speed_limit_violation_ratio = self._speed_violation_ticks / self._total_ticks
        metrics.lane_boundary_cross_count = self._lane_cross_count
        metrics.red_light_violation_count = self._red_light_violations

        # Comfort - Jerk RMS
        if self._jerks_long:
            metrics.jerk_rms_longitudinal = self._rms(self._jerks_long)
            metrics.jerk_max_longitudinal = max(abs(j) for j in self._jerks_long)
        if self._jerks_lat:
            metrics.jerk_rms_lateral = self._rms(self._jerks_lat)
            metrics.jerk_max_lateral = max(abs(j) for j in self._jerks_lat)

        # Comfort - Acceleration RMS
        if self._accels_long:
            metrics.accel_rms_longitudinal = self._rms(self._accels_long)
        if self._accels_lat:
            metrics.accel_rms_lateral = self._rms(self._accels_lat)

        # Comfort - Steering rate RMS
        if self._steer_rates:
            metrics.steering_rate_rms = self._rms(self._steer_rates)

        # Predictability - Speed oscillation
        if len(self._speeds) > 2:
            speed_diffs = [
                self._speeds[i + 1] - self._speeds[i] for i in range(len(self._speeds) - 1)
            ]
            metrics.speed_oscillation_score = self._rms(speed_diffs)

        # Predictability - Lane center oscillation
        if self._lane_offsets:
            metrics.lane_center_oscillation = self._rms(self._lane_offsets)

        # Predictability - Action smoothness (inverse of steering rate variance)
        if self._steer_rates:
            steer_var = self._variance(self._steer_rates)
            metrics.action_smoothness = 1.0 / (1.0 + steer_var)

        # Efficiency
        if self._speeds:
            metrics.average_speed = sum(self._speeds) / len(self._speeds)

        if self._start_location and self._end_location:
            total_dist = self._start_location.distance(self._end_location)
            path_length = sum(self._speeds) / max(1, self.fps)
            if path_length > 0:
                metrics.progress_ratio = min(1.0, total_dist / path_length)

        return metrics

    @staticmethod
    def _rms(values: List[float]) -> float:
        """Compute root mean square."""
        if not values:
            return 0.0
        return math.sqrt(sum(v * v for v in values) / len(values))

    @staticmethod
    def _variance(values: List[float]) -> float:
        """Compute variance."""
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        return sum((v - mean) ** 2 for v in values) / len(values)

    @staticmethod
    def _get_right_vector(transform: carla.Transform) -> carla.Vector3D:
        """Get right vector from transform."""
        try:
            return transform.get_right_vector()
        except AttributeError:
            yaw = math.radians(transform.rotation.yaw)
            return carla.Vector3D(x=-math.sin(yaw), y=math.cos(yaw), z=0.0)

    def _find_lead_vehicle(
        self,
        ego: carla.Vehicle,
        world: carla.World,
        fwd: carla.Vector3D,
        max_dist: float = 50.0,
    ) -> tuple[Optional[carla.Actor], float]:
        """Find the lead vehicle in front of ego."""
        ego_loc = ego.get_location()
        vehicles = world.get_actors().filter("vehicle.*")

        closest = None
        min_dist = max_dist

        for v in vehicles:
            if v.id == ego.id:
                continue
            v_loc = v.get_location()
            to_v = carla.Vector3D(
                v_loc.x - ego_loc.x,
                v_loc.y - ego_loc.y,
                0.0,
            )
            dist = math.sqrt(to_v.x**2 + to_v.y**2)
            if dist < 1.0 or dist >= min_dist:
                continue

            # Normalize
            to_v.x /= dist
            to_v.y /= dist

            # Check if in front
            dot = fwd.x * to_v.x + fwd.y * to_v.y
            if dot < 0.7:  # Within ~45 degrees of forward
                continue

            min_dist = dist
            closest = v

        return closest, min_dist
