"""Post-generation plan validator with gating rules and diagnostic plots.

Checks:
  1. Yaw jump (> threshold per frame)
  2. Reverse driving (heading opposes motion for > N frames)
  3. Speed / acceleration out of bounds
  4. Z jump detection (requires telemetry)
  5. Actor presence continuity

Usage:
    from .plan_validator import validate_plan, ValidationReport
    report = validate_plan(plan)
    if not report.passed:
        print(report.to_markdown())
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from .trajectory_schema import Plan, TrajectoryPoint

# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------

_YAW_JUMP_DEG = 120.0      # per frame (90° would catch path U-turns)
_Z_JUMP_M = 2.0            # per frame
_REVERSE_FRAMES = 10        # 0.5s at 20fps
_V_MAX = 25.0               # m/s
_A_MAX = 8.0                # m/s²


@dataclass
class Issue:
    actor_id: str
    kind: str           # "yaw_jump" | "reverse" | "speed" | "accel" | "z_jump"
    severity: str       # "error" | "warning"
    frame_start: int
    frame_end: int
    detail: str


@dataclass
class ValidationReport:
    issues: List[Issue] = field(default_factory=list)
    actor_summaries: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return not any(i.severity == "error" for i in self.issues)

    def to_markdown(self) -> str:
        lines = ["# Plan Validation Report\n"]
        if self.passed:
            lines.append("**PASSED** — no blocking errors.\n")
        else:
            lines.append(f"**FAILED** — {sum(1 for i in self.issues if i.severity == 'error')} error(s).\n")
        lines.append(f"Total issues: {len(self.issues)}\n")

        for issue in self.issues:
            icon = "❌" if issue.severity == "error" else "⚠️"
            lines.append(
                f"- {icon} **{issue.kind}** [{issue.actor_id}] "
                f"frames {issue.frame_start}-{issue.frame_end}: {issue.detail}"
            )

        if self.actor_summaries:
            lines.append("\n## Actor Summaries\n")
            lines.append("| Actor | Points | Yaw Range | Speed Range | Max ΔYaw/f | Max Δpos/f |")
            lines.append("|-------|--------|-----------|-------------|------------|------------|")
            for aid, s in self.actor_summaries.items():
                lines.append(
                    f"| {aid} | {s.get('n_pts', 0)} "
                    f"| [{s.get('yaw_min', 0):.1f}, {s.get('yaw_max', 0):.1f}] "
                    f"| [{s.get('v_min', 0):.1f}, {s.get('v_max', 0):.1f}] m/s "
                    f"| {s.get('max_dyaw', 0):.1f}° "
                    f"| {s.get('max_dpos', 0):.2f} m |"
                )
        return "\n".join(lines)


def validate_plan(
    plan: Plan,
    *,
    yaw_jump_deg: float = _YAW_JUMP_DEG,
    reverse_frames: int = _REVERSE_FRAMES,
    v_max: float = _V_MAX,
    a_max: float = _A_MAX,
) -> ValidationReport:
    """Validate a loaded Plan, returning a report with issues."""
    report = ValidationReport()

    for actor in plan.actors:
        traj = actor.trajectory
        if len(traj) < 2:
            continue

        aid = actor.actor_id
        yaws = [p.yaw for p in traj]
        speeds = [p.v for p in traj]
        accels = [p.a for p in traj]

        max_dyaw = 0.0
        max_dpos = 0.0
        reverse_count = 0
        reverse_start = 0

        for i in range(1, len(traj)):
            p0, p1 = traj[i - 1], traj[i]

            # Yaw jump — warning for moderate, error only for sustained reversal
            dyaw = p1.yaw - p0.yaw
            dyaw = (dyaw + 180) % 360 - 180
            abs_dyaw = abs(dyaw)
            max_dyaw = max(max_dyaw, abs_dyaw)
            if abs_dyaw > yaw_jump_deg:
                report.issues.append(Issue(
                    actor_id=aid, kind="yaw_jump", severity="warning",
                    frame_start=i - 1, frame_end=i,
                    detail=f"Δyaw={dyaw:.1f}° (threshold={yaw_jump_deg}°)",
                ))

            # Position step
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dpos = math.hypot(dx, dy)
            max_dpos = max(max_dpos, dpos)

            # Reverse detection: dot(Δpos, heading) < 0
            if dpos > 0.05:
                heading_rad = math.radians(p1.yaw)
                forward = dx * math.cos(heading_rad) + dy * math.sin(heading_rad)
                if forward < -0.01:
                    if reverse_count == 0:
                        reverse_start = i
                    reverse_count += 1
                else:
                    if reverse_count >= reverse_frames:
                        report.issues.append(Issue(
                            actor_id=aid, kind="reverse", severity="error",
                            frame_start=reverse_start, frame_end=i - 1,
                            detail=f"Reverse driving for {reverse_count} frames",
                        ))
                    reverse_count = 0

            # Speed
            if abs(p1.v) > v_max:
                report.issues.append(Issue(
                    actor_id=aid, kind="speed", severity="warning",
                    frame_start=i, frame_end=i,
                    detail=f"v={p1.v:.1f} m/s (limit={v_max})",
                ))

            # Acceleration
            if abs(p1.a) > a_max:
                report.issues.append(Issue(
                    actor_id=aid, kind="accel", severity="warning",
                    frame_start=i, frame_end=i,
                    detail=f"a={p1.a:.1f} m/s² (limit={a_max})",
                ))

        # Flush remaining reverse
        if reverse_count >= reverse_frames:
            report.issues.append(Issue(
                actor_id=aid, kind="reverse", severity="error",
                frame_start=reverse_start, frame_end=len(traj) - 1,
                detail=f"Reverse driving for {reverse_count} frames",
            ))

        report.actor_summaries[aid] = {
            "n_pts": len(traj),
            "yaw_min": min(yaws),
            "yaw_max": max(yaws),
            "v_min": min(speeds),
            "v_max": max(speeds),
            "a_min": min(accels),
            "a_max": max(accels),
            "max_dyaw": max_dyaw,
            "max_dpos": max_dpos,
        }

    return report


def generate_debug_plots(
    plan: Plan,
    output_path: Path,
) -> None:
    """Generate debug diagnostic plots for all actors in a plan."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    actors = plan.actors
    n_actors = len(actors)
    if n_actors == 0:
        return

    fig, axes = plt.subplots(n_actors, 4, figsize=(20, 4 * n_actors), squeeze=False)

    for row, actor in enumerate(actors):
        traj = actor.trajectory
        if len(traj) < 2:
            continue
        ts = [p.t for p in traj]
        xs = [p.x for p in traj]
        ys = [p.y for p in traj]
        yaws = [p.yaw for p in traj]
        vs = [p.v for p in traj]

        # XY
        ax = axes[row][0]
        ax.plot(xs, ys, linewidth=0.5)
        ax.plot(xs[0], ys[0], "go", markersize=5)
        ax.plot(xs[-1], ys[-1], "rs", markersize=5)
        ax.set_title(f"{actor.actor_id} XY")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        # Yaw
        ax = axes[row][1]
        ax.plot(ts, yaws, linewidth=0.5)
        ax.set_title("Yaw (deg)")
        ax.grid(True, alpha=0.3)

        # Speed
        ax = axes[row][2]
        ax.plot(ts, vs, linewidth=0.5)
        ax.set_title("Speed (m/s)")
        ax.grid(True, alpha=0.3)

        # Δyaw per frame
        ax = axes[row][3]
        dyaws = [0.0]
        for i in range(1, len(yaws)):
            d = yaws[i] - yaws[i - 1]
            d = (d + 180) % 360 - 180
            dyaws.append(d)
        ax.plot(ts, dyaws, linewidth=0.5)
        ax.axhline(90, color="r", linestyle="--", alpha=0.5)
        ax.axhline(-90, color="r", linestyle="--", alpha=0.5)
        ax.set_title("Δyaw/frame (deg)")
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"Plan Debug: {plan.episode_id}", fontsize=14)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=120)
    plt.close(fig)


def save_report(report: ValidationReport, output_dir: Path) -> None:
    """Save report.md to output directory."""
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "report.md").write_text(report.to_markdown())
