"""Telemetry recording and SAE J670 coordinate system support.

This module provides per-tick telemetry recording with SAE J670 coordinate
system transformations for vehicle state data including:
- Position (world and vehicle-relative)
- Velocity (longitudinal, lateral, vertical)
- Acceleration (longitudinal, lateral, vertical)
- Angular velocity (roll rate, pitch rate, yaw rate)
- Vehicle control inputs (throttle, brake, steer)
"""

from .recorder import TelemetryRecorder
from .sae_j670 import SAEJ670Transformer

__all__ = ["TelemetryRecorder", "SAEJ670Transformer"]
