"""Quality gate checks for V3 scenarios.

Validates event intensity (TTC, gap at trigger) and motion continuity
(speed, acceleration, yaw smoothness) against configurable thresholds.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from .event_dsl import EventSpec
from .metrics import ActorMetrics


@dataclass
class GateResult:
    """Single gate check result."""

    event_id: str
    metric: str
    passed: bool
    value: float
    threshold: float
    severity: str  # "error" | "warning"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "event_id": self.event_id,
            "metric": self.metric,
            "passed": self.passed,
            "value": round(self.value, 3) if self.value != float("inf") else None,
            "threshold": round(self.threshold, 3),
            "severity": self.severity,
        }


def check_event_intensity(
    metrics: ActorMetrics,
    spec: EventSpec,
) -> List[GateResult]:
    """Check that event trigger metrics meet quality thresholds.

    Args:
        metrics: Metrics at the moment the event was triggered
        spec: Event specification with gate thresholds

    Returns:
        List of gate check results
    """
    results = []

    # Gate: minimum TTC at trigger (collision risk not too severe)
    if spec.gate_min_ttc_s is not None:
        ttc = metrics.ttc_s
        passed = ttc >= spec.gate_min_ttc_s or ttc == float("inf")
        results.append(
            GateResult(
                event_id=spec.id,
                metric="ttc_at_trigger_s",
                passed=passed,
                value=ttc,
                threshold=spec.gate_min_ttc_s,
                severity="error" if not passed else "warning",
            )
        )

    # Gate: maximum gap at trigger (event close enough to be meaningful)
    if spec.gate_max_gap_m is not None:
        passed = metrics.gap_m <= spec.gate_max_gap_m
        results.append(
            GateResult(
                event_id=spec.id,
                metric="gap_at_trigger_m",
                passed=passed,
                value=metrics.gap_m,
                threshold=spec.gate_max_gap_m,
                severity="warning" if not passed else "warning",
            )
        )

    return results


def check_continuity(
    telemetry_path: Path,
    fps: int,
    *,
    max_speed_mps: float = 25.0,
    max_accel_mps2: float = 8.0,
    max_yaw_jump_deg: float = 30.0,
) -> List[GateResult]:
    """Check telemetry for motion continuity violations.

    Args:
        telemetry_path: Path to telemetry.json
        fps: Frames per second
        max_speed_mps: Maximum allowed speed
        max_accel_mps2: Maximum allowed acceleration magnitude
        max_yaw_jump_deg: Maximum yaw change between frames

    Returns:
        List of gate check results
    """
    results = []

    if not telemetry_path.exists():
        results.append(
            GateResult(
                event_id="continuity",
                metric="telemetry_exists",
                passed=False,
                value=0.0,
                threshold=1.0,
                severity="error",
            )
        )
        return results

    try:
        data = json.loads(telemetry_path.read_text())
    except (json.JSONDecodeError, IOError):
        results.append(
            GateResult(
                event_id="continuity",
                metric="telemetry_readable",
                passed=False,
                value=0.0,
                threshold=1.0,
                severity="error",
            )
        )
        return results

    frames = data.get("frames", [])
    if not frames:
        return results

    dt = 1.0 / max(1, fps)
    max_speed_seen = 0.0
    max_accel_seen = 0.0
    max_yaw_jump_seen = 0.0
    prev_speed = None
    prev_yaw = None

    for frame in frames:
        ego = frame.get("ego", {})
        speed = ego.get("speed_fd", ego.get("speed", 0.0))
        yaw = ego.get("yaw", 0.0)

        max_speed_seen = max(max_speed_seen, abs(speed))

        if prev_speed is not None:
            accel = abs(speed - prev_speed) / dt
            max_accel_seen = max(max_accel_seen, accel)

        if prev_yaw is not None:
            dyaw = abs(yaw - prev_yaw)
            if dyaw > 180:
                dyaw = 360 - dyaw
            max_yaw_jump_seen = max(max_yaw_jump_seen, dyaw)

        prev_speed = speed
        prev_yaw = yaw

    results.append(
        GateResult(
            event_id="continuity",
            metric="max_speed_mps",
            passed=max_speed_seen <= max_speed_mps,
            value=max_speed_seen,
            threshold=max_speed_mps,
            severity="error" if max_speed_seen > max_speed_mps else "warning",
        )
    )
    results.append(
        GateResult(
            event_id="continuity",
            metric="max_accel_mps2",
            passed=max_accel_seen <= max_accel_mps2,
            value=max_accel_seen,
            threshold=max_accel_mps2,
            severity="error" if max_accel_seen > max_accel_mps2 else "warning",
        )
    )
    results.append(
        GateResult(
            event_id="continuity",
            metric="max_yaw_jump_deg",
            passed=max_yaw_jump_seen <= max_yaw_jump_deg,
            value=max_yaw_jump_seen,
            threshold=max_yaw_jump_deg,
            severity="error" if max_yaw_jump_seen > max_yaw_jump_deg else "warning",
        )
    )

    return results


def run_all_gates(
    event_states: list,
    telemetry_path: Path,
    fps: int,
) -> Dict[str, Any]:
    """Run all quality gate checks and return structured report.

    Args:
        event_states: List of EventState from director
        telemetry_path: Path to telemetry.json
        fps: Frames per second

    Returns:
        Dict suitable for writing to gate_report.json
    """
    all_results: List[GateResult] = []

    # Event intensity gates
    for es in event_states:
        if es.metrics_at_trigger is not None:
            all_results.extend(
                check_event_intensity(es.metrics_at_trigger, es.spec)
            )

    # Continuity gates
    all_results.extend(check_continuity(telemetry_path, fps))

    passed = all(r.passed for r in all_results)
    errors = [r for r in all_results if not r.passed and r.severity == "error"]
    warnings = [r for r in all_results if not r.passed and r.severity == "warning"]

    report = {
        "passed": passed,
        "total_checks": len(all_results),
        "errors": len(errors),
        "warnings": len(warnings),
        "results": [r.to_dict() for r in all_results],
    }

    if not passed:
        logging.warning(
            "Quality gates FAILED: %d errors, %d warnings",
            len(errors),
            len(warnings),
        )
    else:
        logging.info(
            "Quality gates passed (%d checks, %d warnings)",
            len(all_results),
            len(warnings),
        )

    return report


def save_gate_report(report: Dict[str, Any], output_dir: Path) -> Path:
    """Save gate report to JSON file."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "gate_report.json"
    path.write_text(json.dumps(report, indent=2))
    logging.info("Saved gate report: %s", path)
    return path
