"""Road-following path planner and trajectory generator.

Replaces simple linear interpolation with lane-centerline-following paths
and kinematically feasible speed profiles.

DocRef: technical_details.md#2.5
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .trajectory_schema import TrajectoryPoint

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class PathPoint:
    """A sampled point along a road-following path."""

    x: float
    y: float
    yaw: float  # degrees
    lane_id: Optional[str] = None
    s: float = 0.0  # cumulative arc-length (m)
    curvature: float = 0.0  # 1/R (1/m)


@dataclass
class _LaneData:
    """Internal representation of a single lane from map_graph.json."""

    lane_id: str
    centerline: List[Tuple[float, float]]  # [(x, y), ...]
    cum_s: List[float]  # cumulative arc-length at each centerline point
    yaws: List[float]  # heading (degrees) at each point
    successors: List[str]
    predecessors: List[str]
    left_lane: Optional[str]
    right_lane: Optional[str]


# ---------------------------------------------------------------------------
# Kinematic parameters (defaults)
# ---------------------------------------------------------------------------

_A_LAT_MAX = 4.0  # m/s²  lateral acceleration limit (curvature speed cap)
_A_LONG_MAX = 3.0  # m/s²  comfortable longitudinal acceleration
_A_LONG_MIN = -4.0  # m/s²  comfortable deceleration (negative)
_V_MAX_DEFAULT = 16.7  # m/s  ≈ 60 km/h
_LANE_CHANGE_S = 20.0  # m   longitudinal distance for lane change blend
_PATH_SAMPLE_STEP = 1.0  # m   path resampling interval


# ---------------------------------------------------------------------------
# LaneGraph — offline lane topology from map_graph.json
# ---------------------------------------------------------------------------


class LaneGraph:
    """Spatial + topological index over lane centerlines from *map_graph.json*."""

    def __init__(self, map_graph_path: Path) -> None:
        raw = json.loads(map_graph_path.read_text())
        self._lanes: Dict[str, _LaneData] = {}
        self._cell_size = 5.0
        # spatial hash: cell → list of (x, y, lane_id, point_index)
        self._cells: Dict[Tuple[int, int], List[Tuple[float, float, str, int]]] = {}

        for lane_raw in raw.get("lanes", []):
            lid = lane_raw["id"]
            pts = [(p[0], p[1]) for p in lane_raw["centerline"]]
            if len(pts) < 2:
                continue

            # Compute cumulative arc-length and yaw for each point
            cum_s = [0.0]
            yaws: List[float] = []
            for i in range(1, len(pts)):
                dx = pts[i][0] - pts[i - 1][0]
                dy = pts[i][1] - pts[i - 1][1]
                cum_s.append(cum_s[-1] + math.hypot(dx, dy))
            # yaw from consecutive points
            for i in range(len(pts) - 1):
                dx = pts[i + 1][0] - pts[i][0]
                dy = pts[i + 1][1] - pts[i][1]
                yaws.append(math.degrees(math.atan2(dy, dx)))
            yaws.append(yaws[-1] if yaws else 0.0)

            ld = _LaneData(
                lane_id=lid,
                centerline=pts,
                cum_s=cum_s,
                yaws=yaws,
                successors=lane_raw.get("successors", []),
                predecessors=lane_raw.get("predecessors", []),
                left_lane=lane_raw.get("left_lane"),
                right_lane=lane_raw.get("right_lane"),
            )
            self._lanes[lid] = ld

            # Index points into spatial hash
            for idx, (px, py) in enumerate(pts):
                cell = self._cell(px, py)
                self._cells.setdefault(cell, []).append((px, py, lid, idx))

    # -- helpers --

    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor(x / self._cell_size)), int(math.floor(y / self._cell_size)))

    # -- public API --

    def project(self, x: float, y: float) -> Tuple[Optional[str], float, float]:
        """Project *(x, y)* onto nearest lane centerline.

        Returns ``(lane_id, s_along_lane, lateral_offset)``.
        """
        best_d2: Optional[float] = None
        best_lane: Optional[str] = None
        best_idx = 0
        cx, cy = self._cell(x, y)
        for radius in range(7):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for px, py, lid, pidx in self._cells.get((cx + dx, cy + dy), ()):
                        d2 = (x - px) ** 2 + (y - py) ** 2
                        if best_d2 is None or d2 < best_d2:
                            best_d2 = d2
                            best_lane = lid
                            best_idx = pidx
            if best_d2 is not None and radius >= 1:
                break

        if best_lane is None:
            return None, 0.0, float("inf")

        ld = self._lanes[best_lane]
        # Refine: project onto segment between best_idx-1..best_idx+1
        s_proj, lat = self._project_onto_segment(x, y, ld, best_idx)
        return best_lane, s_proj, lat

    def _project_onto_segment(
        self, x: float, y: float, ld: _LaneData, near_idx: int
    ) -> Tuple[float, float]:
        """Project (x,y) onto the nearest segment around *near_idx*, return (s, lateral_offset)."""
        best_s = ld.cum_s[near_idx]
        best_lat = math.hypot(x - ld.centerline[near_idx][0], y - ld.centerline[near_idx][1])

        for i0 in range(max(0, near_idx - 1), min(len(ld.centerline) - 1, near_idx + 1)):
            ax, ay = ld.centerline[i0]
            bx, by = ld.centerline[i0 + 1]
            abx, aby = bx - ax, by - ay
            ab_len2 = abx * abx + aby * aby
            if ab_len2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / ab_len2))
            px = ax + t * abx
            py = ay + t * aby
            lat = math.hypot(x - px, y - py)
            if lat < best_lat:
                best_lat = lat
                best_s = ld.cum_s[i0] + t * math.sqrt(ab_len2)

        return best_s, best_lat

    def sample_along(
        self, lane_id: str, s_start: float, s_end: float, step: float = _PATH_SAMPLE_STEP
    ) -> List[PathPoint]:
        """Sample dense points along *lane_id* centerline from *s_start* to *s_end*."""
        ld = self._lanes.get(lane_id)
        if ld is None or not ld.centerline:
            return []

        forward = s_end >= s_start
        total_len = ld.cum_s[-1]
        if total_len < 1e-3:
            return []

        # Clamp
        s_start = max(0.0, min(s_start, total_len))
        s_end = max(0.0, min(s_end, total_len))

        points: List[PathPoint] = []
        if forward:
            s = s_start
            while s <= s_end + 1e-6:
                pt = self._interp_at_s(ld, s)
                if pt is not None:
                    points.append(pt)
                s += step
        else:
            s = s_start
            while s >= s_end - 1e-6:
                pt = self._interp_at_s(ld, s)
                if pt is not None:
                    points.append(pt)
                s -= step

        return points

    def _interp_at_s(self, ld: _LaneData, s: float) -> Optional[PathPoint]:
        """Interpolate position/yaw on lane at arc-length *s*."""
        if s <= 0:
            return PathPoint(
                x=ld.centerline[0][0], y=ld.centerline[0][1],
                yaw=ld.yaws[0], lane_id=ld.lane_id, s=0.0,
            )
        if s >= ld.cum_s[-1]:
            return PathPoint(
                x=ld.centerline[-1][0], y=ld.centerline[-1][1],
                yaw=ld.yaws[-1], lane_id=ld.lane_id, s=ld.cum_s[-1],
            )
        # Binary search for segment
        lo, hi = 0, len(ld.cum_s) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if ld.cum_s[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg_len = ld.cum_s[hi] - ld.cum_s[lo]
        t = (s - ld.cum_s[lo]) / seg_len if seg_len > 1e-9 else 0.0
        x0, y0 = ld.centerline[lo]
        x1, y1 = ld.centerline[hi]
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        # Interpolate yaw
        yaw0 = ld.yaws[lo]
        yaw1 = ld.yaws[hi]
        # Handle angle wrapping
        dyaw = yaw1 - yaw0
        if dyaw > 180:
            dyaw -= 360
        elif dyaw < -180:
            dyaw += 360
        yaw = yaw0 + dyaw * t
        return PathPoint(x=x, y=y, yaw=yaw, lane_id=ld.lane_id, s=s)

    def find_route(
        self, start_lane: str, end_lane: str, *, max_depth: int = 20
    ) -> Optional[List[str]]:
        """BFS to find a lane sequence from *start_lane* to *end_lane* via successors."""
        if start_lane == end_lane:
            return [start_lane]
        if start_lane not in self._lanes or end_lane not in self._lanes:
            return None

        visited = {start_lane}
        queue: deque[Tuple[str, List[str]]] = deque()
        queue.append((start_lane, [start_lane]))
        while queue:
            cur, path = queue.popleft()
            if len(path) > max_depth:
                continue
            ld = self._lanes.get(cur)
            if ld is None:
                continue
            for nxt in ld.successors:
                if nxt == end_lane:
                    return path + [nxt]
                if nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, path + [nxt]))
        return None

    def get_adjacent(self, lane_id: str, direction: str) -> Optional[str]:
        """Return the left or right adjacent lane ID, or None."""
        ld = self._lanes.get(lane_id)
        if ld is None:
            return None
        if direction == "left":
            return ld.left_lane
        elif direction == "right":
            return ld.right_lane
        return None

    def lane_length(self, lane_id: str) -> float:
        ld = self._lanes.get(lane_id)
        return ld.cum_s[-1] if ld else 0.0

    def has_lane(self, lane_id: str) -> bool:
        return lane_id in self._lanes


# ---------------------------------------------------------------------------
# Path building — keyframes → road-following path
# ---------------------------------------------------------------------------


def _are_adjacent(graph: LaneGraph, lane_a: str, lane_b: str) -> bool:
    """Check if *lane_a* and *lane_b* are laterally adjacent."""
    return (
        graph.get_adjacent(lane_a, "left") == lane_b
        or graph.get_adjacent(lane_a, "right") == lane_b
    )


def _hermite_blend(t: float) -> float:
    """Smooth-step Hermite basis: H(t) = 3t² − 2t³."""
    return 3.0 * t * t - 2.0 * t * t * t


def _blend_lane_change(
    src_pts: List[PathPoint],
    dst_pts: List[PathPoint],
) -> List[PathPoint]:
    """Blend between *src_pts* and *dst_pts* using Hermite smooth-step.

    Both lists should have the same length (resampled to match).
    """
    n = min(len(src_pts), len(dst_pts))
    if n == 0:
        return []
    result: List[PathPoint] = []
    for i in range(n):
        alpha = _hermite_blend(i / max(1, n - 1))
        sp = src_pts[i]
        dp = dst_pts[i]
        x = sp.x + (dp.x - sp.x) * alpha
        y = sp.y + (dp.y - sp.y) * alpha
        # Blend yaw with angle wrapping
        dyaw = dp.yaw - sp.yaw
        if dyaw > 180:
            dyaw -= 360
        elif dyaw < -180:
            dyaw += 360
        yaw = sp.yaw + dyaw * alpha
        result.append(PathPoint(
            x=x, y=y, yaw=yaw,
            lane_id=dp.lane_id if alpha > 0.5 else sp.lane_id,
            s=0.0,  # recomputed later
        ))
    return result


def build_road_path(
    keyframes: Sequence,  # List[Keyframe] — avoid import cycle
    graph: LaneGraph,
    *,
    sample_step: float = _PATH_SAMPLE_STEP,
    lane_change_s: float = _LANE_CHANGE_S,
) -> List[PathPoint]:
    """Build a dense road-following path from sparse *keyframes*.

    Each keyframe must have attributes ``t``, ``x``, ``y`` (and optionally ``v``).
    """
    if len(keyframes) < 2:
        return []

    # 1. Project each keyframe onto nearest lane
    projections: List[Tuple[Optional[str], float]] = []
    for kf in keyframes:
        lid, s, _lat = graph.project(kf.x, kf.y)
        projections.append((lid, s))

    # 2. Build path segment-by-segment
    all_points: List[PathPoint] = []

    for i in range(len(keyframes) - 1):
        lid_a, s_a = projections[i]
        lid_b, s_b = projections[i + 1]

        if lid_a is None or lid_b is None:
            # Fallback: linear segment
            all_points.extend(_linear_segment(keyframes[i], keyframes[i + 1], sample_step))
            continue

        if lid_a == lid_b:
            # Same lane: follow centerline
            seg = graph.sample_along(lid_a, s_a, s_b, step=sample_step)
            if seg:
                # Skip first point if we already have points (avoid duplicates)
                if all_points:
                    seg = seg[1:] if len(seg) > 1 else seg
                all_points.extend(seg)
        elif _are_adjacent(graph, lid_a, lid_b):
            # Adjacent lane: smooth lane change
            # Sample both lanes over the transition distance
            mid_s_a = s_a
            end_s_a = min(s_a + lane_change_s, graph.lane_length(lid_a))
            src_pts = graph.sample_along(lid_a, mid_s_a, end_s_a, step=sample_step)

            # Sample target lane over same distance
            dst_pts = graph.sample_along(lid_b, s_b - lane_change_s, s_b, step=sample_step)

            # Match lengths
            n = min(len(src_pts), len(dst_pts))
            if n > 0:
                blended = _blend_lane_change(src_pts[:n], dst_pts[:n])
                if all_points:
                    blended = blended[1:] if len(blended) > 1 else blended
                all_points.extend(blended)
        else:
            # Different road segments: try BFS route
            route = graph.find_route(lid_a, lid_b)
            if route and len(route) >= 2:
                # Follow each lane in route
                for j, rid in enumerate(route):
                    if j == 0:
                        seg = graph.sample_along(rid, s_a, graph.lane_length(rid), step=sample_step)
                    elif j == len(route) - 1:
                        seg = graph.sample_along(rid, 0.0, s_b, step=sample_step)
                    else:
                        seg = graph.sample_along(rid, 0.0, graph.lane_length(rid), step=sample_step)
                    if seg:
                        if all_points:
                            seg = seg[1:] if len(seg) > 1 else seg
                        all_points.extend(seg)
            else:
                # No route found: linear fallback
                all_points.extend(_linear_segment(keyframes[i], keyframes[i + 1], sample_step))

    # 3. Recompute cumulative arc-length
    if all_points:
        all_points[0].s = 0.0
        for i in range(1, len(all_points)):
            dx = all_points[i].x - all_points[i - 1].x
            dy = all_points[i].y - all_points[i - 1].y
            all_points[i].s = all_points[i - 1].s + math.hypot(dx, dy)

    # 4. Compute curvature at each point (three-point Menger curvature)
    _compute_curvature(all_points)

    return all_points


def _linear_segment(kf_a, kf_b, step: float) -> List[PathPoint]:
    """Fallback: linear interpolation between two keyframes."""
    dx = kf_b.x - kf_a.x
    dy = kf_b.y - kf_a.y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return [PathPoint(x=kf_a.x, y=kf_a.y, yaw=0.0)]
    n = max(2, int(dist / step) + 1)
    yaw = math.degrees(math.atan2(dy, dx))
    pts: List[PathPoint] = []
    for i in range(n):
        alpha = i / (n - 1)
        pts.append(PathPoint(
            x=kf_a.x + dx * alpha,
            y=kf_a.y + dy * alpha,
            yaw=yaw,
        ))
    return pts


def _compute_curvature(points: List[PathPoint]) -> None:
    """Compute Menger curvature for each point (three-point formula)."""
    n = len(points)
    if n < 3:
        return
    for i in range(1, n - 1):
        x0, y0 = points[i - 1].x, points[i - 1].y
        x1, y1 = points[i].x, points[i].y
        x2, y2 = points[i + 1].x, points[i + 1].y
        # Twice the signed area of triangle
        area2 = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
        d01 = math.hypot(x1 - x0, y1 - y0)
        d12 = math.hypot(x2 - x1, y2 - y1)
        d02 = math.hypot(x2 - x0, y2 - y0)
        denom = d01 * d12 * d02
        points[i].curvature = (2.0 * area2 / denom) if denom > 1e-9 else 0.0
    # Copy boundary values
    points[0].curvature = points[1].curvature
    points[-1].curvature = points[-2].curvature


# ---------------------------------------------------------------------------
# Speed profile generation
# ---------------------------------------------------------------------------


def generate_speed_profile(
    path: List[PathPoint],
    keyframes: Sequence,
    *,
    a_lat_max: float = _A_LAT_MAX,
    a_long_max: float = _A_LONG_MAX,
    a_long_min: float = _A_LONG_MIN,
    v_max: float = _V_MAX_DEFAULT,
) -> List[float]:
    """Generate a kinematically feasible speed profile over *path*.

    Uses forward/backward pass to enforce acceleration/deceleration limits.
    """
    n = len(path)
    if n == 0:
        return []

    speeds = [0.0] * n

    # Step 1: curvature-based speed limit + default max
    for i, pt in enumerate(path):
        if pt.curvature > 1e-6:
            v_curv = math.sqrt(a_lat_max / pt.curvature)
            speeds[i] = min(v_max, v_curv)
        else:
            speeds[i] = v_max

    # Step 2: apply user speed hints at closest path points
    total_s = path[-1].s if path else 0.0
    for kf in keyframes:
        if getattr(kf, "v", None) is not None:
            # Find closest path point
            best_idx = 0
            best_d2 = float("inf")
            for j, pt in enumerate(path):
                d2 = (kf.x - pt.x) ** 2 + (kf.y - pt.y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = j
            speeds[best_idx] = float(kf.v)

    # Step 3: forward pass — limit acceleration
    for i in range(1, n):
        ds = path[i].s - path[i - 1].s
        if ds < 1e-6:
            continue
        v_prev = max(0.1, speeds[i - 1])
        # v² = v0² + 2*a*ds
        v_max_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * a_long_max * ds))
        speeds[i] = min(speeds[i], v_max_accel)

    # Step 4: backward pass — limit deceleration
    for i in range(n - 2, -1, -1):
        ds = path[i + 1].s - path[i].s
        if ds < 1e-6:
            continue
        v_next = max(0.1, speeds[i + 1])
        # v² = v_next² + 2*|a_min|*ds  (decel going backward)
        v_max_decel = math.sqrt(max(0.0, v_next * v_next + 2.0 * abs(a_long_min) * ds))
        speeds[i] = min(speeds[i], v_max_decel)

    # Ensure no zero speeds in the middle (minimum crawl speed)
    for i in range(n):
        if speeds[i] < 0.5:
            speeds[i] = 0.5

    return speeds


# ---------------------------------------------------------------------------
# Time parameterization — path + speeds → TrajectoryPoint[]
# ---------------------------------------------------------------------------


def parameterize_by_time(
    path: List[PathPoint],
    speeds: List[float],
    dt: float,
    duration: float,
) -> List[TrajectoryPoint]:
    """Convert spatial *path* + *speeds* into time-indexed trajectory at fixed *dt*."""
    if not path or not speeds:
        return []

    total_frames = max(1, int(duration / dt))
    trajectory: List[TrajectoryPoint] = []

    # Build cumulative time from path + speeds
    # t[i] = t[i-1] + ds / v_avg
    path_times = [0.0]
    for i in range(1, len(path)):
        ds = path[i].s - path[i - 1].s
        v_avg = max(0.1, (speeds[i] + speeds[i - 1]) * 0.5)
        path_times.append(path_times[-1] + ds / v_avg)

    total_path_time = path_times[-1] if path_times else 0.0

    # Sample at fixed dt
    path_idx = 0
    for frame in range(total_frames):
        t = frame * dt

        if t >= total_path_time:
            # Path ended — hold last position
            last = path[-1]
            trajectory.append(TrajectoryPoint(
                t=t, x=last.x, y=last.y, yaw=last.yaw,
                v=0.0, a=0.0, lane_id=last.lane_id, s=last.s,
            ))
            continue

        # Advance path_idx to find the segment containing time t
        while path_idx < len(path_times) - 2 and path_times[path_idx + 1] < t:
            path_idx += 1

        # Interpolate within segment
        t0 = path_times[path_idx]
        t1 = path_times[min(path_idx + 1, len(path_times) - 1)]
        seg_dt = t1 - t0
        alpha = (t - t0) / seg_dt if seg_dt > 1e-9 else 0.0
        alpha = max(0.0, min(1.0, alpha))

        p0 = path[path_idx]
        p1 = path[min(path_idx + 1, len(path) - 1)]
        x = p0.x + (p1.x - p0.x) * alpha
        y = p0.y + (p1.y - p0.y) * alpha

        # Yaw with angle wrapping
        dyaw = p1.yaw - p0.yaw
        if dyaw > 180:
            dyaw -= 360
        elif dyaw < -180:
            dyaw += 360
        yaw = p0.yaw + dyaw * alpha

        v = speeds[path_idx] + (speeds[min(path_idx + 1, len(speeds) - 1)] - speeds[path_idx]) * alpha
        s = p0.s + (p1.s - p0.s) * alpha
        lane_id = p1.lane_id if alpha > 0.5 else p0.lane_id

        trajectory.append(TrajectoryPoint(
            t=t, x=x, y=y, yaw=yaw, v=v, a=0.0,
            lane_id=lane_id, s=s,
        ))

    # Compute acceleration from consecutive speeds
    for i in range(1, len(trajectory)):
        trajectory[i].a = (trajectory[i].v - trajectory[i - 1].v) / dt

    # --- Yaw post-processing: override all yaw with motion tangent ---
    # This avoids lane-centerline yaw artifacts (wrong direction, jumps at junctions)
    if len(trajectory) >= 2:
        # Forward pass: compute yaw from motion direction for all points
        # Only update yaw when there's meaningful movement (>= 0.02m between frames)
        last_valid_yaw = trajectory[0].yaw
        for i in range(len(trajectory)):
            if i < len(trajectory) - 1:
                dx = trajectory[i + 1].x - trajectory[i].x
                dy = trajectory[i + 1].y - trajectory[i].y
            else:
                dx = trajectory[i].x - trajectory[i - 1].x
                dy = trajectory[i].y - trajectory[i - 1].y
            dist = math.hypot(dx, dy)
            if dist > 0.02:
                last_valid_yaw = math.degrees(math.atan2(dy, dx))
            trajectory[i].yaw = last_valid_yaw
        # Unwrap pass: ensure continuity (no > 180° jumps)
        for i in range(1, len(trajectory)):
            dyaw = trajectory[i].yaw - trajectory[i - 1].yaw
            if dyaw > 180:
                trajectory[i].yaw -= 360
            elif dyaw < -180:
                trajectory[i].yaw += 360
        # Smoothing pass: exponential moving average on yaw to reduce jerkiness
        # alpha = 0.15 → ~7 frame window at 20fps = 0.35s smoothing
        _SMOOTH_ALPHA = 0.15
        smoothed = trajectory[0].yaw
        for i in range(1, len(trajectory)):
            dyaw = trajectory[i].yaw - smoothed
            # Ensure delta is already unwrapped (should be after above pass)
            smoothed = smoothed + _SMOOTH_ALPHA * dyaw
            trajectory[i].yaw = smoothed

    return trajectory


# ---------------------------------------------------------------------------
# Keyframe alignment: map original keyframe times to plan trajectory times
# ---------------------------------------------------------------------------


def compute_keyframe_alignment(
    trajectory: List[TrajectoryPoint],
    keyframes: Sequence,  # List[Keyframe] from editor
) -> List[dict]:
    """Compute alignment between original keyframe positions and plan trajectory.

    For each keyframe, finds the closest point in the trajectory and records
    the time mapping (t_ui -> t_plan) and spatial error.

    Returns list of dicts suitable for KeyframeAlignment.from_dict().
    """
    if not trajectory or not keyframes:
        return []

    alignment = []
    for idx, kf in enumerate(keyframes):
        # Find trajectory point closest to keyframe position
        best_i = 0
        best_dist = float("inf")
        for i, pt in enumerate(trajectory):
            dist = math.hypot(pt.x - kf.x, pt.y - kf.y)
            if dist < best_dist:
                best_dist = dist
                best_i = i

        pt = trajectory[best_i]
        alignment.append({
            "kf_idx": idx,
            "t_ui": kf.t,
            "t_plan": pt.t,
            "x_kf": kf.x,
            "y_kf": kf.y,
            "x_plan": pt.x,
            "y_plan": pt.y,
            "spatial_error_m": best_dist,
        })

    return alignment


def compute_file_sha256(file_path: Path) -> str:
    """Compute SHA256 hash of a file's contents."""
    import hashlib
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# G3: Static tail handling — trim_to_motion / hold_at_end / extend_cruise
# ---------------------------------------------------------------------------


def apply_episode_end_mode(
    trajectory: List[TrajectoryPoint],
    end_mode: str,
    dt: float,
    *,
    min_motion_speed: float = 0.1,
) -> List[TrajectoryPoint]:
    """Apply episode end mode to trajectory.

    Args:
        trajectory: Input trajectory
        end_mode: One of "trim_to_motion", "hold_at_end", "extend_cruise"
        dt: Time step
        min_motion_speed: Speed threshold for detecting motion (m/s)

    Returns:
        Processed trajectory

    Modes:
        - trim_to_motion: Truncate after last frame with speed > min_motion_speed
        - hold_at_end: Keep trajectory as-is (actor sits at final position)
        - extend_cruise: If path ends early, continue at last speed (not implemented)
    """
    if not trajectory or len(trajectory) < 2:
        return trajectory

    if end_mode == "hold_at_end":
        # Default behavior: keep everything, actor sits at final position
        return trajectory

    elif end_mode == "trim_to_motion":
        # Find the last frame where the actor is still moving
        last_moving_idx = len(trajectory) - 1
        for i in range(len(trajectory) - 1, -1, -1):
            if trajectory[i].v > min_motion_speed:
                last_moving_idx = i
                break

        # Keep a few frames after stopping to show the stop, but not the long tail
        # Add 10 frames (0.5s at 20fps) of buffer after last motion
        buffer_frames = min(10, len(trajectory) - last_moving_idx - 1)
        trim_idx = min(last_moving_idx + buffer_frames + 1, len(trajectory))

        if trim_idx < len(trajectory):
            return trajectory[:trim_idx]
        return trajectory

    elif end_mode == "extend_cruise":
        # If trajectory ends with the actor stopped but duration not reached,
        # extend by continuing at the last non-zero speed
        # This is more complex and requires lane-following; leave as hold_at_end for now
        # TODO: Implement extend_cruise with lane following
        return trajectory

    else:
        # Unknown mode: return as-is
        return trajectory


def get_trajectory_motion_stats(trajectory: List[TrajectoryPoint]) -> dict:
    """Compute motion statistics for a trajectory.

    Returns dict with:
        - last_motion_t: Time of last frame with speed > 0.1 m/s
        - last_motion_idx: Frame index of last motion
        - static_tail_duration: Duration of static tail (actor not moving)
        - total_duration: Total trajectory duration
        - static_tail_ratio: Ratio of static tail to total duration
    """
    if not trajectory:
        return {
            "last_motion_t": 0.0,
            "last_motion_idx": 0,
            "static_tail_duration": 0.0,
            "total_duration": 0.0,
            "static_tail_ratio": 0.0,
        }

    last_motion_idx = len(trajectory) - 1
    for i in range(len(trajectory) - 1, -1, -1):
        if trajectory[i].v > 0.1:
            last_motion_idx = i
            break

    last_motion_t = trajectory[last_motion_idx].t
    total_duration = trajectory[-1].t
    static_tail_duration = total_duration - last_motion_t

    return {
        "last_motion_t": round(last_motion_t, 3),
        "last_motion_idx": last_motion_idx,
        "static_tail_duration": round(static_tail_duration, 3),
        "total_duration": round(total_duration, 3),
        "static_tail_ratio": round(static_tail_duration / total_duration, 3) if total_duration > 0 else 0.0,
    }
