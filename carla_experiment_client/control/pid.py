"""PID controller with anti-windup and output clamping."""

from __future__ import annotations


class PIDController:
    """Discrete PID controller for vehicle control loops.

    Features:
    - Proportional, integral, derivative terms
    - Anti-windup via integral clamping
    - Output clamping
    - Derivative filtering (first-order)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_min: float | None = None,
        integral_max: float | None = None,
        derivative_filter: float = 0.1,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min if integral_min is not None else output_min * 2
        self.integral_max = integral_max if integral_max is not None else output_max * 2
        self.derivative_filter = derivative_filter

        self._integral = 0.0
        self._prev_error: float | None = None
        self._prev_derivative = 0.0

    def step(self, error: float, dt: float) -> float:
        """Compute PID output for current error.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds

        Returns:
            Clamped control output
        """
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))
        i = self.ki * self._integral

        # Derivative with first-order filter
        if self._prev_error is not None:
            raw_derivative = (error - self._prev_error) / dt
            alpha = self.derivative_filter
            self._prev_derivative = alpha * raw_derivative + (1 - alpha) * self._prev_derivative
        d = self.kd * self._prev_derivative
        self._prev_error = error

        output = p + i + d
        return max(self.output_min, min(self.output_max, output))

    def reset(self) -> None:
        """Reset controller state."""
        self._integral = 0.0
        self._prev_error = None
        self._prev_derivative = 0.0
