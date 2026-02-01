"""Generate comparison plots: plan trajectory vs telemetry.

DOC:comparison-align-interp

Usage:
    python -m carla_experiment_client.tools.make_comparison \
        --scene outputs/scene_designs/lane_change_cut_in/scene_edit.json \
        --telemetry-csv outputs/lane_change_cut_in/render/telemetry.csv \
        --output outputs/lane_change_cut_in/render/
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def _compute_file_sha256(file_path: Path) -> str:
    """Compute SHA256 hash of a file's contents."""
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    """Load telemetry CSV into column arrays."""
    import csv
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    cols: Dict[str, List[float]] = {}
    for row in rows:
        for k, v in row.items():
            cols.setdefault(k.strip(), []).append(float(v))
    return {k: np.array(v) for k, v in cols.items()}


def _load_scene(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_events(path: Path) -> list:
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    if isinstance(data, list):
        return data
    return data.get("events", [])


def _load_plan(path: Path) -> Optional[dict]:
    if path.exists():
        return json.loads(path.read_text())
    return None


# ---------------------------------------------------------------------------
# Alignment & interpolation (single source of truth)
# ---------------------------------------------------------------------------

# DOC:comparison-align-interp
def align_trajectories(
    ref_t: np.ndarray,
    src_t: np.ndarray,
    src_x: np.ndarray,
    src_y: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Interpolate src trajectory at ref_t times. Returns (x_interp, y_interp)."""
    if len(src_t) < 2 or len(ref_t) < 1:
        return np.zeros_like(ref_t), np.zeros_like(ref_t)
    # Ensure src_t is sorted
    order = np.argsort(src_t)
    src_t = src_t[order]
    src_x = src_x[order]
    src_y = src_y[order]
    return np.interp(ref_t, src_t, src_x), np.interp(ref_t, src_t, src_y)


def compute_position_error(
    x_a: np.ndarray, y_a: np.ndarray,
    x_b: np.ndarray, y_b: np.ndarray,
) -> np.ndarray:
    """Euclidean distance between two trajectories (same length)."""
    return np.sqrt((x_a - x_b) ** 2 + (y_a - y_b) ** 2)


def error_stats(err: np.ndarray) -> dict:
    if len(err) == 0:
        return {"mean": 0.0, "median": 0.0, "p95": 0.0, "max": 0.0, "peak_idx": 0}
    peak_idx = int(np.argmax(err))
    return {
        "mean": round(float(np.mean(err)), 3),
        "median": round(float(np.median(err)), 3),
        "p95": round(float(np.percentile(err, 95)), 3),
        "max": round(float(np.max(err)), 3),
        "peak_idx": peak_idx,
    }


# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------


def _sanity_check(csv_data: dict, plan: Optional[dict], scene: dict):
    """Raise on critical data inconsistencies."""
    warnings = []

    # Check episode_id consistency
    if plan:
        p_eid = plan.get("episode_id", "")
        s_eid = scene.get("episode_id", "")
        if p_eid and s_eid and p_eid != s_eid:
            raise RuntimeError(
                f"episode_id mismatch: scene={s_eid!r} vs plan={p_eid!r}"
            )
        p_town = plan.get("town", "")
        s_town = scene.get("town", "")
        if p_town and s_town and p_town != s_town:
            raise RuntimeError(f"town mismatch: scene={s_town!r} vs plan={p_town!r}")

    # Check if speed_actor is always 0 (teleport mode)
    speed_col = csv_data.get("speed_actor", csv_data.get("speed", np.array([])))
    if len(speed_col) > 10 and np.max(np.abs(speed_col)) < 0.01:
        warnings.append(
            "speed_actor is ~0 throughout (expected under TeleportFollower). "
            "Using speed_fd for comparison."
        )

    # Check CSV t_sim is sorted
    csv_t = csv_data.get("t_sim", np.array([]))
    if len(csv_t) > 1 and np.any(np.diff(csv_t) < -1e-6):
        raise RuntimeError("CSV t_sim is not sorted! Data may be corrupted.")

    return warnings


def _validate_hash_chain(
    scene_path: Path,
    plan: Optional[dict],
    telemetry_json_path: Optional[Path] = None,
) -> dict:
    """Validate SHA256 hash chain: scene -> plan -> telemetry.

    Returns dict with validation results.
    """
    result = {
        "hash_chain_valid": True,
        "source_scene_sha256": None,
        "source_plan_sha256": None,
        "scene_hash_match": None,
        "plan_hash_match": None,
        "warnings": [],
    }

    # Compute current scene hash
    if scene_path.exists():
        result["source_scene_sha256"] = _compute_file_sha256(scene_path)

    # Check plan's recorded scene hash
    if plan:
        plan_scene_sha = plan.get("source_scene_sha256")
        if plan_scene_sha:
            if result["source_scene_sha256"]:
                result["scene_hash_match"] = (plan_scene_sha == result["source_scene_sha256"])
                if not result["scene_hash_match"]:
                    result["hash_chain_valid"] = False
                    result["warnings"].append(
                        f"scene_edit.json has been modified since plan was generated "
                        f"(plan expects {plan_scene_sha[:12]}..., got {result['source_scene_sha256'][:12]}...)"
                    )
        else:
            result["warnings"].append("plan.json missing source_scene_sha256 (old format)")

    # Check telemetry's recorded plan hash (if telemetry.json exists)
    if telemetry_json_path and telemetry_json_path.exists():
        try:
            telem_data = json.loads(telemetry_json_path.read_text())
            telem_meta = telem_data.get("metadata", {})
            telem_plan_sha = telem_meta.get("source_plan_sha256")
            if telem_plan_sha:
                result["source_plan_sha256"] = telem_plan_sha
                # We'd need plan.json path to verify, but at least record it
        except (json.JSONDecodeError, IOError):
            pass

    return result


def _extract_keyframe_alignment(plan: Optional[dict]) -> Tuple[List[dict], dict]:
    """Extract keyframe_alignment from plan's ego actor.

    Returns (alignment_list, retime_stats).
    """
    alignment = []
    retime_stats = {
        "retime_drift_max_s": 0.0,
        "retime_drift_mean_s": 0.0,
        "spatial_error_max_m": 0.0,
        "spatial_error_mean_m": 0.0,
    }

    if not plan:
        return alignment, retime_stats

    for actor in plan.get("actors", []):
        if actor.get("role") == "ego" or actor.get("id") == "ego":
            alignment = actor.get("keyframe_alignment", [])
            break

    if alignment:
        drifts = [abs(a.get("t_plan", 0) - a.get("t_ui", 0)) for a in alignment]
        spatial_errs = [a.get("spatial_error_m", 0) for a in alignment]
        retime_stats = {
            "retime_drift_max_s": round(max(drifts), 3) if drifts else 0.0,
            "retime_drift_mean_s": round(sum(drifts) / len(drifts), 3) if drifts else 0.0,
            "spatial_error_max_m": round(max(spatial_errs), 3) if spatial_errs else 0.0,
            "spatial_error_mean_m": round(sum(spatial_errs) / len(spatial_errs), 3) if spatial_errs else 0.0,
        }

    return alignment, retime_stats


# ---------------------------------------------------------------------------
# Main comparison generation
# ---------------------------------------------------------------------------


def generate_comparison(
    scene_path: Path,
    csv_path: Path,
    output_dir: Path,
    events_path: Optional[Path] = None,
    actors_filter: Optional[List[str]] = None,
) -> dict:
    """Generate comparison.png and compare_report.json."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scene = _load_scene(scene_path)
    csv_data = _load_csv(csv_path)
    events = _load_events(events_path) if events_path else []

    # Try loading plan.json from render dir or sibling
    plan_path = csv_path.parent / "plan.json"
    if not plan_path.exists():
        # Try episode subdir
        for p in csv_path.parent.glob("*/plan.json"):
            plan_path = p
            break
    plan = _load_plan(plan_path)

    dt = scene.get("dt", 0.05)
    duration = scene.get("duration", 60.0)

    # Sanity checks
    sanity_warnings = _sanity_check(csv_data, plan, scene)

    # Hash chain validation (G1)
    telemetry_json_path = csv_path.parent / "telemetry.json"
    hash_validation = _validate_hash_chain(scene_path, plan, telemetry_json_path)
    sanity_warnings.extend(hash_validation.get("warnings", []))

    # Extract keyframe alignment from plan (G2)
    kf_alignment, retime_stats = _extract_keyframe_alignment(plan)

    # --- Extract telemetry ---
    csv_t = csv_data.get("t_sim", np.array([]))
    csv_x = csv_data.get("world_x", np.array([]))
    csv_y = csv_data.get("world_y", np.array([]))
    # Prefer speed_fd over speed_actor
    csv_speed_fd = csv_data.get("speed_fd", np.array([]))
    csv_speed_actor = csv_data.get("speed_actor", csv_data.get("speed", np.array([])))
    if len(csv_speed_fd) == 0:
        # Compute from position if not in CSV
        csv_speed_fd = np.zeros_like(csv_t)
        for i in range(1, len(csv_t)):
            dx = csv_x[i] - csv_x[i - 1]
            dy = csv_y[i] - csv_y[i - 1]
            csv_speed_fd[i] = math.hypot(dx, dy) / dt
        if len(csv_speed_fd) > 1:
            csv_speed_fd[0] = csv_speed_fd[1]

    # --- Extract plan trajectory ---
    plan_t = plan_x = plan_y = plan_v = None
    if plan:
        for actor in plan.get("actors", []):
            if actor.get("role") == "ego" or actor.get("id") == "ego":
                traj = actor.get("trajectory", [])
                if traj:
                    plan_t = np.array([p["t"] for p in traj])
                    plan_x = np.array([p["x"] for p in traj])
                    plan_y = np.array([p["y"] for p in traj])
                    plan_v = np.array([p.get("v", 0) for p in traj])
                break

    # --- Compute errors using shared align function ---
    # Primary: plan trajectory vs telemetry
    plan_stats = {"mean": 0.0, "median": 0.0, "p95": 0.0, "max": 0.0, "peak_idx": 0}
    plan_err = np.array([])
    if plan_t is not None and len(csv_t) > 1:
        px_at_csv, py_at_csv = align_trajectories(csv_t, plan_t, plan_x, plan_y)
        plan_err = compute_position_error(px_at_csv, py_at_csv, csv_x, csv_y)
        plan_stats = error_stats(plan_err)

    # Secondary: keyframe error using retimed alignment (G2 fix)
    # Uses t_plan from keyframe_alignment instead of t_ui to avoid time drift error
    ego_actor = None
    for actor in scene.get("actors", []):
        if actor.get("role") == "ego" or actor.get("id") == "ego":
            ego_actor = actor
            break
    if ego_actor is None and scene.get("actors"):
        ego_actor = scene["actors"][0]

    kf_retime_stats = {"mean": 0.0, "median": 0.0, "p95": 0.0, "max": 0.0, "peak_idx": 0}
    kf_legacy_stats = {"mean": 0.0, "median": 0.0, "p95": 0.0, "max": 0.0, "peak_idx": 0}

    if ego_actor and len(csv_t) > 1:
        kfs = sorted(ego_actor.get("keyframes", []), key=lambda k: k["t"])
        if kfs:
            # Legacy: use original keyframe times (t_ui) - kept for comparison
            kf_t = np.array([k["t"] for k in kfs])
            kf_x = np.array([k["x"] for k in kfs])
            kf_y = np.array([k["y"] for k in kfs])
            kfx_at_csv, kfy_at_csv = align_trajectories(csv_t, kf_t, kf_x, kf_y)
            kf_err_legacy = compute_position_error(kfx_at_csv, kfy_at_csv, csv_x, csv_y)
            kf_legacy_stats = error_stats(kf_err_legacy)

            # G2 fix: use retimed t_plan from keyframe_alignment
            if kf_alignment and len(kf_alignment) == len(kfs):
                # Use t_plan and (x_plan, y_plan) from alignment for proper comparison
                kf_t_plan = np.array([a.get("t_plan", a.get("t_ui", 0)) for a in kf_alignment])
                kf_x_plan = np.array([a.get("x_plan", kfs[i]["x"]) for i, a in enumerate(kf_alignment)])
                kf_y_plan = np.array([a.get("y_plan", kfs[i]["y"]) for i, a in enumerate(kf_alignment)])
                kfx_retime, kfy_retime = align_trajectories(csv_t, kf_t_plan, kf_x_plan, kf_y_plan)
                kf_err_retime = compute_position_error(kfx_retime, kfy_retime, csv_x, csv_y)
                kf_retime_stats = error_stats(kf_err_retime)
            else:
                # Fallback to legacy if no alignment data
                kf_retime_stats = kf_legacy_stats

    # --- Plot (4 panels) ---
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # Panel 1: XY trajectory
    ax1 = axes[0, 0]
    ax1.plot(csv_x, csv_y, "b-", linewidth=1.2, label="Telemetry")
    if plan_x is not None:
        ax1.plot(plan_x, plan_y, "g-", linewidth=1.0, alpha=0.7, label="Plan trajectory")
    if ego_actor:
        for kf in ego_actor.get("keyframes", []):
            ax1.plot(kf["x"], kf["y"], "ro", markersize=5)
        ax1.plot([], [], "ro", markersize=5, label="Keyframes")
    # Mark peak error location
    if len(plan_err) > 0 and plan_stats["peak_idx"] < len(csv_x):
        pidx = plan_stats["peak_idx"]
        ax1.plot(csv_x[pidx], csv_y[pidx], "mx", markersize=10, markeredgewidth=2,
                 label=f"Peak err: {plan_stats['max']:.2f}m")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("XY Trajectory")
    ax1.legend(fontsize=7)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    # Panel 2: Speed (fd + plan + actor)
    ax2 = axes[0, 1]
    ax2.plot(csv_t, csv_speed_fd, "b-", linewidth=1.0, label="speed_fd (primary)")
    if plan_v is not None and plan_t is not None:
        ax2.plot(plan_t, plan_v, "g--", linewidth=0.8, label="speed_plan")
    if len(csv_speed_actor) > 0 and np.max(np.abs(csv_speed_actor)) > 0.01:
        ax2.plot(csv_t, csv_speed_actor, "r:", linewidth=0.5, alpha=0.5, label="speed_actor")
    for evt in events:
        t_evt = evt.get("t_event", evt.get("t", 0))
        ax2.axvline(t_evt, color="green", alpha=0.4, linestyle="--")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.set_title("Ego Speed")
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Plan → Telemetry error (primary)
    ax3 = axes[1, 0]
    if len(plan_err) > 0:
        ax3.plot(csv_t, plan_err, "g-", linewidth=0.8)
        ax3.axhline(plan_stats["mean"], color="orange", linestyle="--",
                     label=f"mean={plan_stats['mean']:.3f}m")
        ax3.axhline(plan_stats["p95"], color="red", linestyle=":", alpha=0.5,
                     label=f"p95={plan_stats['p95']:.3f}m")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position Error (m)")
    ax3.set_title(f"Plan→Telem: mean={plan_stats['mean']:.3f}m, max={plan_stats['max']:.3f}m")
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.3)

    # Panel 4: Speed error (plan speed vs fd speed)
    ax4 = axes[1, 1]
    if plan_v is not None and plan_t is not None and len(csv_t) > 1:
        pv_at_csv = np.interp(csv_t, plan_t, plan_v)
        speed_err = np.abs(pv_at_csv - csv_speed_fd)
        ax4.plot(csv_t, speed_err, "m-", linewidth=0.8)
        se_stats = error_stats(speed_err)
        ax4.axhline(se_stats["mean"], color="orange", linestyle="--",
                     label=f"mean={se_stats['mean']:.2f} m/s")
        ax4.set_title(f"Speed Error: mean={se_stats['mean']:.2f}, p95={se_stats['p95']:.2f} m/s")
    else:
        ax4.set_title("Speed Error (no plan data)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("|speed_plan - speed_fd| (m/s)")
    ax4.legend(fontsize=7)
    ax4.grid(True, alpha=0.3)

    eid = scene.get("episode_id", "unknown")
    fig.suptitle(f"Comparison Report: {eid}", fontsize=13)
    fig.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / "comparison.png", dpi=150)
    plt.close(fig)

    # --- Build structured report ---
    report = {
        "episode_id": eid,
        "plan_trajectory_error": plan_stats,
        # G2: renamed from kf_interp_error, uses t_plan alignment
        "kf_retime_error": kf_retime_stats,
        # Legacy metric kept for comparison (uses original t_ui)
        "kf_legacy_error": kf_legacy_stats,
        # G2: retime drift statistics
        "retime_stats": retime_stats,
        "n_frames": int(len(csv_t)),
        "n_keyframes": len(ego_actor.get("keyframes", [])) if ego_actor else 0,
        "speed_source": "speed_fd (finite-difference)",
        "sanity_warnings": sanity_warnings,
        # G1: hash chain validation
        "hash_chain_valid": hash_validation.get("hash_chain_valid", True),
        "source_scene_sha256": hash_validation.get("source_scene_sha256"),
    }
    if len(plan_err) > 0 and plan_stats["peak_idx"] < len(csv_t):
        pidx = plan_stats["peak_idx"]
        report["peak_error_location"] = {
            "frame": int(pidx),
            "t": round(float(csv_t[pidx]), 3),
            "x": round(float(csv_x[pidx]), 2),
            "y": round(float(csv_y[pidx]), 2),
        }
    (output_dir / "compare_report.json").write_text(json.dumps(report, indent=2))
    (output_dir / "summary.json").write_text(json.dumps(report, indent=2))

    print(f"Saved comparison.png + compare_report.json to {output_dir}")
    print(f"  Plan→Telem: mean={plan_stats['mean']:.3f}m | p95={plan_stats['p95']:.3f}m | max={plan_stats['max']:.3f}m")
    print(f"  KF retime:  mean={kf_retime_stats['mean']:.2f}m  | p95={kf_retime_stats['p95']:.2f}m  | max={kf_retime_stats['max']:.2f}m")
    if retime_stats["retime_drift_max_s"] > 0:
        print(f"  Retime drift: max={retime_stats['retime_drift_max_s']:.1f}s | mean={retime_stats['retime_drift_mean_s']:.1f}s")
    if not hash_validation.get("hash_chain_valid", True):
        print("  WARNING: Hash chain validation FAILED - data may be inconsistent")
    if sanity_warnings:
        for w in sanity_warnings:
            print(f"  WARNING: {w}")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate telemetry comparison report.")
    parser.add_argument("--scene", required=True, help="Path to scene_edit.json")
    parser.add_argument("--telemetry-csv", required=True, help="Path to telemetry.csv")
    parser.add_argument("--events", help="Path to events.json (optional)")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--actors", help="Comma-separated actor filter")
    args = parser.parse_args(argv)

    report = generate_comparison(
        scene_path=Path(args.scene),
        csv_path=Path(args.telemetry_csv),
        output_dir=Path(args.output),
        events_path=Path(args.events) if args.events else None,
        actors_filter=args.actors.split(",") if args.actors else None,
    )
    return 0 if "error" not in report else 1


if __name__ == "__main__":
    sys.exit(main())
