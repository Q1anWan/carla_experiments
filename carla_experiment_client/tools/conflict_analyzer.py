"""Automatic conflict analysis for plan.json trajectories.

G4: Analyzes multi-actor trajectories to detect:
- Minimum gap (spatial distance between actors)
- Minimum TTC (time-to-collision)
- Cut-in events (lane change into ego's lane)
- Ego response metrics (brake timing, max deceleration)

Usage:
    python -m carla_experiment_client.tools.conflict_analyzer \
        --plan outputs/lane_change_cut_in/plan.json \
        --output outputs/lane_change_cut_in/
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..planning.trajectory_schema import Plan, load_plan, TrajectoryPoint


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class CutInEvent:
    """Detected lane change cut-in event."""
    actor_id: str
    start_t: float  # Time when lane change begins
    end_t: float  # Time when lane change completes
    start_frame: int
    end_frame: int
    lane_before: Optional[str] = None
    lane_after: Optional[str] = None
    min_gap_during: float = float("inf")  # Min gap during cut-in

    def to_dict(self) -> Dict[str, Any]:
        return {
            "actor_id": self.actor_id,
            "start_t": round(self.start_t, 3),
            "end_t": round(self.end_t, 3),
            "start_frame": self.start_frame,
            "end_frame": self.end_frame,
            "lane_before": self.lane_before,
            "lane_after": self.lane_after,
            "min_gap_during": round(self.min_gap_during, 2),
        }


@dataclass
class EgoResponse:
    """Ego vehicle response to conflict."""
    brake_start_t: Optional[float] = None
    brake_start_frame: Optional[int] = None
    max_decel: float = 0.0
    max_decel_t: Optional[float] = None
    max_decel_frame: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "brake_start_t": round(self.brake_start_t, 3) if self.brake_start_t else None,
            "brake_start_frame": self.brake_start_frame,
            "max_decel": round(self.max_decel, 3),
            "max_decel_t": round(self.max_decel_t, 3) if self.max_decel_t else None,
            "max_decel_frame": self.max_decel_frame,
        }


@dataclass
class ConflictReport:
    """Full conflict analysis report."""
    episode_id: str
    # Gap metrics
    min_gap_m: float = float("inf")
    min_gap_t: float = 0.0
    min_gap_frame: int = 0
    min_gap_actors: Tuple[str, str] = ("", "")
    # TTC metrics
    min_ttc_s: float = float("inf")
    min_ttc_t: float = 0.0
    min_ttc_frame: int = 0
    min_ttc_actors: Tuple[str, str] = ("", "")
    # Cut-in events
    cut_in_events: List[CutInEvent] = field(default_factory=list)
    # Ego response
    ego_response: EgoResponse = field(default_factory=EgoResponse)
    # Per-frame conflict score
    conflict_score: List[float] = field(default_factory=list)
    # Summary
    has_conflict: bool = False
    conflict_severity: str = "none"  # none, low, medium, high

    def to_dict(self) -> Dict[str, Any]:
        return {
            "episode_id": self.episode_id,
            "min_gap": {
                "distance_m": round(self.min_gap_m, 2) if self.min_gap_m < float("inf") else None,
                "t": round(self.min_gap_t, 3),
                "frame": self.min_gap_frame,
                "actors": list(self.min_gap_actors),
            },
            "min_ttc": {
                "ttc_s": round(self.min_ttc_s, 2) if self.min_ttc_s < float("inf") else None,
                "t": round(self.min_ttc_t, 3),
                "frame": self.min_ttc_frame,
                "actors": list(self.min_ttc_actors),
            },
            "cut_in_events": [e.to_dict() for e in self.cut_in_events],
            "ego_response": self.ego_response.to_dict(),
            "conflict_score": [round(s, 3) for s in self.conflict_score],
            "has_conflict": self.has_conflict,
            "conflict_severity": self.conflict_severity,
        }


# ---------------------------------------------------------------------------
# Analysis functions
# ---------------------------------------------------------------------------


def _compute_gap(p1: TrajectoryPoint, p2: TrajectoryPoint) -> float:
    """Compute Euclidean distance between two trajectory points."""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)


def _compute_ttc(
    p1: TrajectoryPoint, v1: float,
    p2: TrajectoryPoint, v2: float,
) -> float:
    """Compute time-to-collision between two actors.

    Uses simplified longitudinal TTC assuming actors are on same lane.
    Returns infinity if no collision expected.
    """
    # Relative position and velocity
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    gap = math.hypot(dx, dy)

    if gap < 0.1:
        return 0.0  # Already colliding

    # Project velocities along the gap direction
    if gap > 0:
        nx, ny = dx / gap, dy / gap
    else:
        return float("inf")

    # Compute heading-aligned velocities
    yaw1_rad = math.radians(p1.yaw)
    yaw2_rad = math.radians(p2.yaw)

    vx1 = v1 * math.cos(yaw1_rad)
    vy1 = v1 * math.sin(yaw1_rad)
    vx2 = v2 * math.cos(yaw2_rad)
    vy2 = v2 * math.sin(yaw2_rad)

    # Relative velocity along gap direction
    rel_v = (vx1 - vx2) * nx + (vy1 - vy2) * ny

    if rel_v <= 0:
        # Not approaching
        return float("inf")

    # TTC = gap / closing_speed
    # Subtract vehicle length approximation (4m)
    effective_gap = max(0, gap - 4.0)
    ttc = effective_gap / rel_v

    return ttc


def _detect_lane_change(
    trajectory: List[TrajectoryPoint],
    min_lateral_shift: float = 2.0,
    window_frames: int = 40,  # 2s at 20fps
) -> List[Tuple[int, int, Optional[str], Optional[str]]]:
    """Detect lane change events in a trajectory.

    Returns list of (start_frame, end_frame, lane_before, lane_after).
    """
    if len(trajectory) < window_frames:
        return []

    events = []
    i = 0
    while i < len(trajectory) - window_frames:
        # Look for lateral shift over window
        start_pt = trajectory[i]

        # Find the end of potential lane change
        best_shift = 0.0
        best_j = i
        for j in range(i + 1, min(i + window_frames, len(trajectory))):
            end_pt = trajectory[j]
            # Compute lateral offset (perpendicular to heading)
            yaw_rad = math.radians(start_pt.yaw)
            dx = end_pt.x - start_pt.x
            dy = end_pt.y - start_pt.y
            lateral = abs(-dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad))
            if lateral > best_shift:
                best_shift = lateral
                best_j = j

        if best_shift >= min_lateral_shift:
            lane_before = trajectory[i].lane_id
            lane_after = trajectory[best_j].lane_id
            if isinstance(lane_before, int):
                lane_before = str(lane_before)
            if isinstance(lane_after, int):
                lane_after = str(lane_after)
            events.append((i, best_j, lane_before, lane_after))
            i = best_j + 1
        else:
            i += 1

    return events


def _is_in_front(ego: TrajectoryPoint, other: TrajectoryPoint, threshold_m: float = 50.0) -> bool:
    """Check if 'other' is in front of ego within threshold distance."""
    dx = other.x - ego.x
    dy = other.y - ego.y
    dist = math.hypot(dx, dy)
    if dist > threshold_m:
        return False

    # Check if in front (within ±90° of heading)
    yaw_rad = math.radians(ego.yaw)
    fwd_x = math.cos(yaw_rad)
    fwd_y = math.sin(yaw_rad)
    dot = dx * fwd_x + dy * fwd_y

    return dot > 0


def analyze_conflicts(plan: Plan) -> ConflictReport:
    """Analyze plan for conflicts between actors.

    Returns ConflictReport with:
    - min_gap_m, min_gap_t, min_gap_actors
    - min_ttc_s, min_ttc_t, min_ttc_actors
    - cut_in_events
    - ego_response
    - conflict_score per frame
    """
    report = ConflictReport(episode_id=plan.episode_id)

    # Find ego actor
    ego_actor = None
    other_actors = []
    for actor in plan.actors:
        if actor.role == "ego":
            ego_actor = actor
        else:
            other_actors.append(actor)

    if ego_actor is None or not ego_actor.trajectory:
        return report

    n_frames = len(ego_actor.trajectory)
    dt = plan.dt

    # Initialize per-frame conflict score
    report.conflict_score = [0.0] * n_frames

    # Analyze each frame
    for frame_idx in range(n_frames):
        ego_pt = ego_actor.trajectory[frame_idx]
        ego_v = ego_pt.v
        t = frame_idx * dt

        for other_actor in other_actors:
            if frame_idx >= len(other_actor.trajectory):
                continue

            other_pt = other_actor.trajectory[frame_idx]
            other_v = other_pt.v

            # Compute gap
            gap = _compute_gap(ego_pt, other_pt)
            if gap < report.min_gap_m:
                report.min_gap_m = gap
                report.min_gap_t = t
                report.min_gap_frame = frame_idx
                report.min_gap_actors = (ego_actor.actor_id, other_actor.actor_id)

            # Compute TTC (only for actors in front)
            if _is_in_front(ego_pt, other_pt):
                ttc = _compute_ttc(ego_pt, ego_v, other_pt, other_v)
                if ttc < report.min_ttc_s:
                    report.min_ttc_s = ttc
                    report.min_ttc_t = t
                    report.min_ttc_frame = frame_idx
                    report.min_ttc_actors = (ego_actor.actor_id, other_actor.actor_id)

            # Update conflict score (higher = more dangerous)
            # Score based on gap and TTC
            gap_score = max(0, (20.0 - gap) / 20.0)  # 1.0 at 0m, 0.0 at 20m
            ttc = _compute_ttc(ego_pt, ego_v, other_pt, other_v)
            ttc_score = max(0, (5.0 - ttc) / 5.0) if ttc < float("inf") else 0.0
            frame_score = max(gap_score, ttc_score)
            report.conflict_score[frame_idx] = max(report.conflict_score[frame_idx], frame_score)

    # Detect cut-in events from other actors
    for other_actor in other_actors:
        lane_changes = _detect_lane_change(other_actor.trajectory)
        for start_f, end_f, lane_before, lane_after in lane_changes:
            # Check if cut-in is into ego's lane
            if start_f < len(ego_actor.trajectory):
                ego_lane = ego_actor.trajectory[start_f].lane_id
                if isinstance(ego_lane, int):
                    ego_lane = str(ego_lane)

                # If ending lane matches ego lane, it's a cut-in
                is_cut_in = (lane_after == ego_lane) if (lane_after and ego_lane) else False

                # Or if gap decreases significantly during lane change
                if start_f < len(ego_actor.trajectory) and end_f < len(ego_actor.trajectory):
                    gap_start = _compute_gap(ego_actor.trajectory[start_f], other_actor.trajectory[start_f])
                    gap_end = _compute_gap(ego_actor.trajectory[end_f], other_actor.trajectory[end_f])
                    if gap_end < gap_start * 0.5 and gap_end < 15.0:
                        is_cut_in = True

                if is_cut_in:
                    # Calculate min gap during cut-in
                    min_gap_during = float("inf")
                    for f in range(start_f, min(end_f + 1, len(ego_actor.trajectory), len(other_actor.trajectory))):
                        g = _compute_gap(ego_actor.trajectory[f], other_actor.trajectory[f])
                        min_gap_during = min(min_gap_during, g)

                    event = CutInEvent(
                        actor_id=other_actor.actor_id,
                        start_t=start_f * dt,
                        end_t=end_f * dt,
                        start_frame=start_f,
                        end_frame=end_f,
                        lane_before=lane_before,
                        lane_after=lane_after,
                        min_gap_during=min_gap_during,
                    )
                    report.cut_in_events.append(event)

    # Analyze ego response (braking)
    ego_response = EgoResponse()
    for frame_idx in range(1, n_frames):
        ego_pt = ego_actor.trajectory[frame_idx]
        ego_prev = ego_actor.trajectory[frame_idx - 1]
        accel = ego_pt.a if ego_pt.a != 0 else (ego_pt.v - ego_prev.v) / dt

        # Detect brake start (first significant deceleration)
        if accel < -0.5 and ego_response.brake_start_t is None:
            ego_response.brake_start_t = frame_idx * dt
            ego_response.brake_start_frame = frame_idx

        # Track max deceleration
        if accel < ego_response.max_decel:
            ego_response.max_decel = accel
            ego_response.max_decel_t = frame_idx * dt
            ego_response.max_decel_frame = frame_idx

    report.ego_response = ego_response

    # Determine if there's a conflict and its severity
    if report.min_gap_m < 5.0 or report.min_ttc_s < 2.0:
        report.has_conflict = True
        if report.min_gap_m < 2.0 or report.min_ttc_s < 1.0:
            report.conflict_severity = "high"
        elif report.min_gap_m < 4.0 or report.min_ttc_s < 1.5:
            report.conflict_severity = "medium"
        else:
            report.conflict_severity = "low"
    elif report.cut_in_events:
        report.has_conflict = True
        report.conflict_severity = "low"

    return report


def save_conflict_report(report: ConflictReport, output_dir: Path) -> Path:
    """Save conflict report to JSON file."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "conflict_report.json"
    path.write_text(json.dumps(report.to_dict(), indent=2))
    return path


# ---------------------------------------------------------------------------
# G5: Event narrative templates and auto-populate
# ---------------------------------------------------------------------------


# Standard event types for conflict scenarios
CONFLICT_EVENT_TYPES = [
    "cut_in_start",      # NPC begins lane change toward ego lane
    "cut_in_complete",   # NPC fully in ego lane
    "risk_peak",         # Minimum TTC or minimum gap moment
    "ego_brake_start",   # Ego begins braking
    "ego_max_decel",     # Ego reaches maximum deceleration
    "risk_resolved",     # Gap returns to safe threshold
]


@dataclass
class SuggestedEvent:
    """An auto-suggested event from conflict analysis."""
    t_event: float
    event_type: str
    expected_action: str
    source: str  # e.g., "conflict_report.min_ttc", "conflict_report.cut_in"
    confidence: float = 1.0  # 0.0 to 1.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t_event": round(self.t_event, 3),
            "event_type": self.event_type,
            "expected_action": self.expected_action,
            "source": self.source,
            "confidence": round(self.confidence, 2),
        }


def auto_populate_events(
    report: ConflictReport,
    dt: float = 0.05,
) -> List[SuggestedEvent]:
    """Generate suggested events from conflict report.

    Returns a list of SuggestedEvent that can be used to populate
    the events timeline in the editor.
    """
    events: List[SuggestedEvent] = []

    # Cut-in events
    for ci in report.cut_in_events:
        events.append(SuggestedEvent(
            t_event=ci.start_t,
            event_type="cut_in_start",
            expected_action="monitor",
            source=f"cut_in.{ci.actor_id}",
            confidence=0.9,
        ))
        events.append(SuggestedEvent(
            t_event=ci.end_t,
            event_type="cut_in_complete",
            expected_action="brake_or_yield",
            source=f"cut_in.{ci.actor_id}",
            confidence=0.9,
        ))

    # Risk peak (min TTC or min gap)
    if report.min_ttc_s < float("inf") and report.min_ttc_s < 3.0:
        events.append(SuggestedEvent(
            t_event=report.min_ttc_t,
            event_type="risk_peak",
            expected_action="brake_hard",
            source="min_ttc",
            confidence=0.95,
        ))
    elif report.min_gap_m < 10.0:
        events.append(SuggestedEvent(
            t_event=report.min_gap_t,
            event_type="risk_peak",
            expected_action="brake_or_yield",
            source="min_gap",
            confidence=0.85,
        ))

    # Ego brake start
    if report.ego_response.brake_start_t is not None:
        events.append(SuggestedEvent(
            t_event=report.ego_response.brake_start_t,
            event_type="ego_brake_start",
            expected_action="braking",
            source="ego_response.brake_start",
            confidence=0.9,
        ))

    # Ego max deceleration
    if report.ego_response.max_decel < -1.0 and report.ego_response.max_decel_t is not None:
        events.append(SuggestedEvent(
            t_event=report.ego_response.max_decel_t,
            event_type="ego_max_decel",
            expected_action="braking_hard",
            source="ego_response.max_decel",
            confidence=0.85,
        ))

    # Risk resolved: find when gap returns to safe level after risk peak
    if report.conflict_score and report.has_conflict:
        peak_idx = report.conflict_score.index(max(report.conflict_score))
        # Find when score drops below 0.2 after peak
        for i in range(peak_idx, len(report.conflict_score)):
            if report.conflict_score[i] < 0.2:
                events.append(SuggestedEvent(
                    t_event=i * dt,
                    event_type="risk_resolved",
                    expected_action="resume_normal",
                    source="conflict_score",
                    confidence=0.7,
                ))
                break

    # Sort by time
    events.sort(key=lambda e: e.t_event)

    return events


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Analyze plan.json for conflicts.")
    parser.add_argument("--plan", required=True, help="Path to plan.json")
    parser.add_argument("--output", required=True, help="Output directory")
    args = parser.parse_args(argv)

    plan_path = Path(args.plan)
    output_dir = Path(args.output)

    if not plan_path.exists():
        print(f"Error: plan.json not found: {plan_path}")
        return 1

    plan = load_plan(plan_path)
    report = analyze_conflicts(plan)

    out_path = save_conflict_report(report, output_dir)
    print(f"Saved conflict_report.json to {out_path}")
    print(f"  Has conflict: {report.has_conflict} ({report.conflict_severity})")
    if report.min_gap_m < float("inf"):
        print(f"  Min gap: {report.min_gap_m:.2f}m at t={report.min_gap_t:.2f}s")
    if report.min_ttc_s < float("inf"):
        print(f"  Min TTC: {report.min_ttc_s:.2f}s at t={report.min_ttc_t:.2f}s")
    if report.cut_in_events:
        print(f"  Cut-in events: {len(report.cut_in_events)}")
        for ci in report.cut_in_events:
            print(f"    - {ci.actor_id}: t={ci.start_t:.2f}s-{ci.end_t:.2f}s, min_gap={ci.min_gap_during:.2f}m")

    # G5: Auto-populate suggested events
    suggested = auto_populate_events(report, plan.dt)
    if suggested:
        suggested_path = output_dir / "suggested_events.json"
        suggested_path.write_text(json.dumps([e.to_dict() for e in suggested], indent=2))
        print(f"Saved suggested_events.json ({len(suggested)} events):")
        for evt in suggested:
            print(f"    - t={evt.t_event:.2f}s: {evt.event_type} ({evt.source})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
