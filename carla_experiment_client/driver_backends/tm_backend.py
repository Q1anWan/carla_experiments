"""Traffic Manager driver backend — default V3 control mode."""

from __future__ import annotations

import logging
from typing import Any, Optional

from .base import DriverBackend

try:
    import carla
except ImportError:
    carla = None


class TMBackend(DriverBackend):
    """Default backend: vehicle runs under CARLA Traffic Manager autopilot.

    run_step() always returns None (TM drives).
    Provides helper methods to adjust TM parameters at runtime.
    """

    def __init__(self, tm: Any) -> None:
        self._tm = tm

    def run_step(
        self, vehicle: Any, frame_index: int, dt: float
    ) -> Optional[Any]:
        return None  # TM drives

    def activate(self, vehicle: Any) -> None:
        """Enable autopilot on the vehicle."""
        vehicle.set_autopilot(True, self._tm.get_port())

    def deactivate(self, vehicle: Any) -> None:
        """Disable autopilot on the vehicle."""
        vehicle.set_autopilot(False)

    def configure(self, vehicle: Any, action: Any) -> None:
        """Apply TMAction parameters to a vehicle.

        Args:
            vehicle: carla.Vehicle
            action: TMAction dataclass from event_dsl
        """
        if action is None:
            return

        if action.speed_diff_pct is not None:
            try:
                self._tm.vehicle_percentage_speed_difference(
                    vehicle, float(action.speed_diff_pct)
                )
            except RuntimeError:
                logging.warning("TM speed diff failed for %s", vehicle.id)

        if action.distance_to_leading_vehicle_m is not None:
            try:
                self._tm.distance_to_leading_vehicle(
                    vehicle, float(action.distance_to_leading_vehicle_m)
                )
            except (AttributeError, RuntimeError):
                logging.warning("TM follow distance failed for %s", vehicle.id)

        if action.ignore_lights_pct is not None:
            try:
                self._tm.ignore_lights_percentage(
                    vehicle, float(action.ignore_lights_pct)
                )
            except (AttributeError, RuntimeError):
                logging.warning("TM ignore lights failed for %s", vehicle.id)

        if action.ignore_vehicles_pct is not None:
            try:
                self._tm.ignore_vehicles_percentage(
                    vehicle, float(action.ignore_vehicles_pct)
                )
            except (AttributeError, RuntimeError):
                logging.warning("TM ignore vehicles failed for %s", vehicle.id)

        if action.auto_lane_change is not None:
            try:
                self._tm.auto_lane_change(vehicle, bool(action.auto_lane_change))
            except (AttributeError, RuntimeError):
                logging.warning("TM auto lane change failed for %s", vehicle.id)

    def force_lane_change(self, vehicle: Any, direction: str) -> None:
        """Force an immediate lane change.

        Args:
            vehicle: carla.Vehicle
            direction: "left" or "right"
        """
        # CARLA TM: force_lane_change(actor, direction_bool)
        # True = left, False = right
        go_left = direction.lower() == "left"
        try:
            self._tm.force_lane_change(vehicle, go_left)
            logging.info(
                "Forced lane change %s for vehicle %s",
                direction,
                vehicle.id,
            )
        except (AttributeError, RuntimeError) as e:
            logging.warning("force_lane_change failed: %s", e)
