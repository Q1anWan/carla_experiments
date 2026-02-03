"""Short-time override controller for physics-based lane changes.

Disables autopilot for a 1-3 second window and drives the vehicle along a
quintic-polynomial lateral profile using PID speed + lateral tracking,
with jerk and acceleration limiting for smooth, natural motion.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from math import copysign
from typing import Any, List, Optional

from ..control.pid import PIDController
from ..utils import clamp
from .base import DriverBackend

try:
    import carla
except ImportError:
    carla = None


@dataclass
class PathRef:
    """Reference point along the planned lane-change path."""

    x: float
    y: float
    yaw: float  # degrees
    s: float  # progress along path (0..1 normalised)


class ShortOverrideBackend(DriverBackend):
    """Physics-closed-loop short-time takeover controller.

    On activate():
      - Disables autopilot
      - Queries CARLA waypoints for source and target lane
      - Plans a quintic lateral profile over the override window

    On each run_step():
      - Samples the reference path at current progress
      - PID speed control → throttle/brake (with jerk limiting)
      - PID lateral control → steer (with steer-rate limiting)

    When the window expires, is_done becomes True and the Director
    hands the vehicle back to TM autopilot.
    """

    def __init__(
        self,
        world: Any,
        map_obj: Any,
        fps: int,
        *,
        window_s: float = 2.0,
        profile: str = "quintic",
        target_speed_mps: Optional[float] = None,
        direction: str = "left",
        # Kinematic constraints
        a_long_max: float = 3.0,
        a_long_min: float = -4.0,
        a_lat_max: float = 4.0,
        jerk_max: float = 10.0,
        steer_rate_max: float = 0.3,
    ) -> None:
        self._world = world
        self._map_obj = map_obj
        self._fps = fps
        self._window_s = window_s
        self._profile = profile
        self._target_speed_mps = target_speed_mps
        self._direction = direction

        self._a_long_max = a_long_max
        self._a_long_min = a_long_min
        self._a_lat_max = a_lat_max
        self._jerk_max = jerk_max
        self._steer_rate_max = steer_rate_max

        self._pid_speed = PIDController(
            kp=1.0, ki=0.1, kd=0.05,
            output_min=a_long_min, output_max=a_long_max,
        )
        self._pid_lateral = PIDController(
            kp=0.8, ki=0.0, kd=0.2,
            output_min=-1.0, output_max=1.0,
        )

        self._path: List[PathRef] = []
        self._start_frame: Optional[int] = None
        self._done = False
        self._prev_accel: Optional[float] = None
        self._prev_steer: Optional[float] = None
        self._initial_speed: Optional[float] = None

    def set_direction(self, direction: str) -> None:
        """Set lane change direction before activate()."""
        self._direction = direction

    def activate(self, vehicle: Any) -> None:
        """Disable autopilot and plan the lane-change path."""
        vehicle.set_autopilot(False)
        self._start_frame = None
        self._done = False
        self._prev_accel = None
        self._prev_steer = None
        self._pid_speed.reset()
        self._pid_lateral.reset()

        self._initial_speed = self._get_speed(vehicle)

        cur_wp = self._map_obj.get_waypoint(
            vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        target_wp = self._get_target_lane_wp(cur_wp)
        if target_wp is None:
            logging.warning(
                "No adjacent lane found for %s direction, override will pass-through",
                self._direction,
            )
            self._done = True
            return

        self._path = self._plan_lane_change(cur_wp, target_wp)
        logging.info(
            "Short override activated: direction=%s, window=%.1fs, path=%d pts",
            self._direction,
            self._window_s,
            len(self._path),
        )

    def deactivate(self, vehicle: Any) -> None:
        """Clean up state."""
        self._path = []
        self._done = True

    def run_step(
        self, vehicle: Any, frame_index: int, dt: float
    ) -> Optional[Any]:
        if self._done or not self._path:
            self._done = True
            return None

        if self._start_frame is None:
            self._start_frame = frame_index

        elapsed = (frame_index - self._start_frame) / max(1, self._fps)
        if elapsed >= self._window_s:
            self._done = True
            return None

        progress = elapsed / self._window_s
        ref = self._sample_path(progress)

        # --- Speed PID ---
        cur_speed = self._get_speed(vehicle)
        target_speed = self._target_speed_mps or self._initial_speed or cur_speed
        speed_err = target_speed - cur_speed
        accel_cmd = self._pid_speed.step(speed_err, dt)
        accel_cmd = clamp(accel_cmd, self._a_long_min, self._a_long_max)

        # Jerk limiting
        if self._prev_accel is not None and dt > 0:
            jerk = (accel_cmd - self._prev_accel) / dt
            if abs(jerk) > self._jerk_max:
                accel_cmd = self._prev_accel + copysign(self._jerk_max * dt, jerk)
        self._prev_accel = accel_cmd

        if accel_cmd >= 0:
            throttle = accel_cmd / max(0.1, self._a_long_max)
            brake = 0.0
        else:
            throttle = 0.0
            brake = -accel_cmd / max(0.1, abs(self._a_long_min))

        # --- Lateral PID (cross-track error → steer) ---
        lateral_err = self._compute_cross_track(vehicle, ref)
        # Also add heading error as feedforward
        heading_err = self._compute_heading_error(vehicle, ref)
        steer_raw = self._pid_lateral.step(lateral_err, dt) + 0.3 * heading_err

        # Steer rate limiting
        if self._prev_steer is not None:
            delta = steer_raw - self._prev_steer
            if abs(delta) > self._steer_rate_max:
                steer_raw = self._prev_steer + copysign(self._steer_rate_max, delta)
        self._prev_steer = steer_raw

        return carla.VehicleControl(
            throttle=clamp(throttle, 0.0, 1.0),
            brake=clamp(brake, 0.0, 1.0),
            steer=clamp(steer_raw, -1.0, 1.0),
        )

    @property
    def is_done(self) -> bool:
        return self._done

    # ------------------------------------------------------------------
    # Path planning
    # ------------------------------------------------------------------

    def _get_target_lane_wp(self, cur_wp: Any) -> Optional[Any]:
        """Get waypoint in the target adjacent lane."""
        if self._direction == "left":
            wp = cur_wp.get_left_lane()
            if wp and wp.lane_type == carla.LaneType.Driving:
                return wp
        elif self._direction == "right":
            wp = cur_wp.get_right_lane()
            if wp and wp.lane_type == carla.LaneType.Driving:
                return wp
        return None

    def _plan_lane_change(
        self, src_wp: Any, dst_wp: Any
    ) -> List[PathRef]:
        """Plan a smooth lane-change path using quintic lateral profile.

        Samples waypoints ahead in both source and destination lanes,
        then blends laterally using a quintic polynomial that ensures
        zero velocity and acceleration at both endpoints.
        """
        # Estimate longitudinal distance during the window
        speed_est = self._initial_speed or 10.0
        longitudinal_m = speed_est * self._window_s
        sample_step = 2.0  # meters
        n_samples = max(5, int(longitudinal_m / sample_step))

        # Sample waypoints ahead in both lanes
        src_wps = self._sample_waypoints_ahead(src_wp, longitudinal_m, n_samples)
        dst_wps = self._sample_waypoints_ahead(dst_wp, longitudinal_m, n_samples)

        if len(src_wps) < 2 or len(dst_wps) < 2:
            # Fallback: minimal path
            return [
                PathRef(
                    x=dst_wp.transform.location.x,
                    y=dst_wp.transform.location.y,
                    yaw=dst_wp.transform.rotation.yaw,
                    s=1.0,
                )
            ]

        # Ensure same number of points
        n = min(len(src_wps), len(dst_wps))
        src_wps = src_wps[:n]
        dst_wps = dst_wps[:n]

        path = []
        for i in range(n):
            t = i / max(1, n - 1)  # normalised progress 0..1

            # Quintic blend: d(t) = 10t^3 - 15t^4 + 6t^5
            if self._profile == "quintic":
                blend = 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5
            else:
                # Cubic Hermite: d(t) = 3t^2 - 2t^3
                blend = 3 * t ** 2 - 2 * t ** 3

            sx = src_wps[i].transform.location.x
            sy = src_wps[i].transform.location.y
            dx = dst_wps[i].transform.location.x
            dy = dst_wps[i].transform.location.y

            x = sx + blend * (dx - sx)
            y = sy + blend * (dy - sy)

            # Yaw from blended heading
            s_yaw = math.radians(src_wps[i].transform.rotation.yaw)
            d_yaw = math.radians(dst_wps[i].transform.rotation.yaw)
            # Angle interpolation via atan2(sin, cos)
            yaw = math.degrees(
                math.atan2(
                    math.sin(s_yaw) + blend * (math.sin(d_yaw) - math.sin(s_yaw)),
                    math.cos(s_yaw) + blend * (math.cos(d_yaw) - math.cos(s_yaw)),
                )
            )

            path.append(PathRef(x=x, y=y, yaw=yaw, s=t))

        return path

    def _sample_waypoints_ahead(
        self, wp: Any, total_distance: float, n_points: int
    ) -> list:
        """Sample waypoints ahead along the lane."""
        step = total_distance / max(1, n_points - 1)
        wps = [wp]
        current = wp
        for _ in range(n_points - 1):
            next_wps = current.next(step)
            if not next_wps:
                break
            current = next_wps[0]
            wps.append(current)
        return wps

    def _sample_path(self, progress: float) -> PathRef:
        """Sample the path at normalised progress [0, 1]."""
        if not self._path:
            return PathRef(x=0, y=0, yaw=0, s=0)

        progress = max(0.0, min(1.0, progress))

        # Find bounding path points
        n = len(self._path)
        if n == 1:
            return self._path[0]

        idx_float = progress * (n - 1)
        idx = int(idx_float)
        frac = idx_float - idx

        if idx >= n - 1:
            return self._path[-1]

        p0 = self._path[idx]
        p1 = self._path[idx + 1]
        return PathRef(
            x=p0.x + frac * (p1.x - p0.x),
            y=p0.y + frac * (p1.y - p0.y),
            yaw=p0.yaw + frac * (p1.yaw - p0.yaw),  # simple for small angles
            s=progress,
        )

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _get_speed(vehicle: Any) -> float:
        vel = vehicle.get_velocity()
        return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    @staticmethod
    def _compute_cross_track(vehicle: Any, ref: PathRef) -> float:
        """Signed cross-track error: positive = vehicle is to the right of ref."""
        loc = vehicle.get_transform().location
        dx = loc.x - ref.x
        dy = loc.y - ref.y
        # Ref heading → right vector
        yaw_rad = math.radians(ref.yaw)
        right_x = -math.sin(yaw_rad)
        right_y = math.cos(yaw_rad)
        return dx * right_x + dy * right_y

    @staticmethod
    def _compute_heading_error(vehicle: Any, ref: PathRef) -> float:
        """Heading error in [-1, 1] range, normalised from degrees."""
        veh_yaw = vehicle.get_transform().rotation.yaw
        err = ref.yaw - veh_yaw
        # Wrap to [-180, 180]
        err = (err + 180) % 360 - 180
        # Normalise to [-1, 1] (90 deg maps to ~1)
        return clamp(err / 90.0, -1.0, 1.0)
