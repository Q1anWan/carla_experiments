"""Abstract base class for V3 driver backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Optional


class DriverBackend(ABC):
    """Base class for vehicle control backends.

    A backend produces VehicleControl commands for a single actor.
    Returning None from run_step() means the vehicle stays under
    its current control mode (typically Traffic Manager autopilot).
    """

    @abstractmethod
    def run_step(
        self, vehicle: Any, frame_index: int, dt: float
    ) -> Optional[Any]:
        """Compute control for this tick.

        Args:
            vehicle: carla.Vehicle actor
            frame_index: Current simulation frame
            dt: Time step in seconds

        Returns:
            carla.VehicleControl to apply, or None to keep current mode.
        """
        ...

    def activate(self, vehicle: Any) -> None:
        """Called when this backend takes control of the vehicle."""

    def deactivate(self, vehicle: Any) -> None:
        """Called when this backend releases control."""

    @property
    def is_done(self) -> bool:
        """Whether this backend has finished its task."""
        return True
