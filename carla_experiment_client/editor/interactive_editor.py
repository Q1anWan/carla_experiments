"""Interactive 2D scene editor MVP for time-parameterized trajectories."""

from __future__ import annotations

import argparse
import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from ..planning.trajectory_schema import ActorPlan, EventPlan, Plan, TrajectoryPoint, save_events_plan, save_plan

plt = None
Button = None
RadioButtons = None
Slider = None
TextBox = None


@dataclass
class Keyframe:
    t: float
    x: float
    y: float
    v: Optional[float] = None


@dataclass
class ActorState:
    actor_id: str
    kind: str
    role: str
    blueprint: str
    controller: str
    keyframes: List[Keyframe] = field(default_factory=list)
    color: str = "#d1495b"
    line: Any = None
    scatter: Any = None
    ghost: Any = None
    conflict_scatter: Any = None
    infeasible_scatter: Any = None
    ttc_scatter: Any = None
    kinematic_scatter: Any = None


@dataclass
class EventMarker:
    t_event: float
    event_type: str
    expected_action: str
    audio_id: Optional[str] = None
    t_voice_start: Optional[float] = None
    t_robot_precue: Optional[float] = None


@dataclass
class SceneEdit:
    version: str
    episode_id: str
    town: str
    dt: float
    duration: float
    map_dir: str
    seed: int = 0
    actors: List[ActorState] = field(default_factory=list)
    events: List[EventMarker] = field(default_factory=list)


# DocRef: technical_details.md#2.4
class MapIndex:
    def __init__(self, lane_points: List[Tuple[float, float, int]], *, cell_size: float = 5.0) -> None:
        self._points = lane_points
        self._cell_size = max(1.0, cell_size)
        self._cells: Dict[Tuple[int, int], List[Tuple[float, float, int]]] = {}
        for x, y, lane_id in lane_points:
            cell = self._cell(x, y)
            self._cells.setdefault(cell, []).append((x, y, lane_id))

    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor(x / self._cell_size)), int(math.floor(y / self._cell_size)))

    def nearest_lane(self, x: float, y: float, *, max_radius: int = 6) -> Tuple[float, Optional[int]]:
        best_d2 = None
        best_lane = None
        cx, cy = self._cell(x, y)
        for radius in range(max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    points = self._cells.get((cx + dx, cy + dy))
                    if not points:
                        continue
                    for px, py, lane_id in points:
                        d2 = (x - px) ** 2 + (y - py) ** 2
                        if best_d2 is None or d2 < best_d2:
                            best_d2 = d2
                            best_lane = lane_id
            if best_d2 is not None and radius >= 1:
                break
        if best_d2 is None:
            for px, py, lane_id in self._points:
                d2 = (x - px) ** 2 + (y - py) ** 2
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best_lane = lane_id
        dist = math.sqrt(best_d2) if best_d2 is not None else float("inf")
        return dist, best_lane

    def snap_to_centerline(self, x: float, y: float, *, max_radius: int = 6) -> Tuple[float, float, float]:
        """Snap a point to the nearest lane centerline point.

        Returns (snapped_x, snapped_y, distance) where distance is the snap offset.
        """
        best_d2: Optional[float] = None
        best_x, best_y = x, y
        cx, cy = self._cell(x, y)
        for radius in range(max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    points = self._cells.get((cx + dx, cy + dy))
                    if not points:
                        continue
                    for px, py, _lane_id in points:
                        d2 = (x - px) ** 2 + (y - py) ** 2
                        if best_d2 is None or d2 < best_d2:
                            best_d2 = d2
                            best_x, best_y = px, py
            if best_d2 is not None and radius >= 1:
                break
        if best_d2 is None:
            for px, py, _lane_id in self._points:
                d2 = (x - px) ** 2 + (y - py) ** 2
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best_x, best_y = px, py
        dist = math.sqrt(best_d2) if best_d2 is not None else 0.0
        return best_x, best_y, dist


def _load_geojson(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _load_globals(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _collect_lane_points(map_graph: Dict[str, Any]) -> List[Tuple[float, float, int]]:
    lanes = map_graph.get("lanes") or []
    points: List[Tuple[float, float, int]] = []
    for lane in lanes:
        lane_id = int(lane.get("lane_id", 0))
        for coord in lane.get("centerline", []) or []:
            if not isinstance(coord, list) or len(coord) < 2:
                continue
            points.append((float(coord[0]), float(coord[1]), lane_id))
    return points


def _default_keyframes_from_map(map_graph: Dict[str, Any], duration: float) -> List[Keyframe]:
    lanes = map_graph.get("lanes") or []
    for lane in lanes:
        centerline = lane.get("centerline") or []
        if len(centerline) < 2:
            continue
        start = centerline[0]
        end = centerline[min(len(centerline) - 1, 50)]
        return [
            Keyframe(t=0.0, x=float(start[0]), y=float(start[1])),
            Keyframe(t=duration, x=float(end[0]), y=float(end[1])),
        ]
    return []


def _prepare_keyframes(keyframes: List[Keyframe], duration: float) -> List[Keyframe]:
    if not keyframes:
        return []
    keyframes = sorted(keyframes, key=lambda k: k.t)
    if keyframes[0].t > 0.0:
        first = keyframes[0]
        keyframes.insert(0, Keyframe(t=0.0, x=first.x, y=first.y, v=first.v))
    if keyframes[-1].t < duration:
        last = keyframes[-1]
        keyframes.append(Keyframe(t=duration, x=last.x, y=last.y, v=last.v))
    return keyframes


def _as_offsets(points: Sequence[Sequence[float]]) -> np.ndarray:
    if not points:
        return np.empty((0, 2))
    return np.array(points, dtype=float)


def _interpolate_position(keyframes: List[Keyframe], t: float) -> Tuple[float, float]:
    if not keyframes:
        return (0.0, 0.0)
    keyframes = sorted(keyframes, key=lambda k: k.t)
    if t <= keyframes[0].t:
        return (keyframes[0].x, keyframes[0].y)
    if t >= keyframes[-1].t:
        return (keyframes[-1].x, keyframes[-1].y)
    for idx in range(len(keyframes) - 1):
        k0 = keyframes[idx]
        k1 = keyframes[idx + 1]
        if k0.t <= t <= k1.t:
            span = k1.t - k0.t
            if span <= 0:
                return (k0.x, k0.y)
            alpha = (t - k0.t) / span
            return (k0.x + (k1.x - k0.x) * alpha, k0.y + (k1.y - k0.y) * alpha)
    return (keyframes[-1].x, keyframes[-1].y)


# DocRef: technical_details.md#2.5
def _build_trajectory(
    keyframes: List[Keyframe],
    *,
    dt: float,
    duration: float,
    map_index: Optional[MapIndex],
) -> List[TrajectoryPoint]:
    if dt <= 0:
        raise ValueError("dt must be positive.")
    total_frames = max(1, int(duration / dt))
    keyframes = _prepare_keyframes(keyframes, duration)
    if not keyframes:
        return []
    points: List[TrajectoryPoint] = []
    keyframes = sorted(keyframes, key=lambda k: k.t)
    segment_idx = 0
    speed_hint: List[Optional[float]] = []
    for frame in range(total_frames):
        t = frame * dt
        while segment_idx < len(keyframes) - 2 and t > keyframes[segment_idx + 1].t:
            segment_idx += 1
        k0 = keyframes[segment_idx]
        k1 = keyframes[min(segment_idx + 1, len(keyframes) - 1)]
        span = k1.t - k0.t
        if span <= 0:
            alpha = 0.0
        else:
            alpha = min(1.0, max(0.0, (t - k0.t) / span))
        x = k0.x + (k1.x - k0.x) * alpha
        y = k0.y + (k1.y - k0.y) * alpha
        v_hint = None
        if k0.v is not None and k1.v is not None and span > 0:
            v_hint = k0.v + (k1.v - k0.v) * alpha
        lane_id = None
        if map_index is not None:
            _, lane_id = map_index.nearest_lane(x, y)
        points.append(
            TrajectoryPoint(
                t=t,
                x=x,
                y=y,
                yaw=0.0,
                v=0.0,
                a=0.0,
                lane_id=lane_id,
            )
        )
        speed_hint.append(v_hint)

    if len(points) > 1:
        for idx in range(len(points) - 1):
            dx = points[idx + 1].x - points[idx].x
            dy = points[idx + 1].y - points[idx].y
            points[idx].yaw = math.degrees(math.atan2(dy, dx))
        points[-1].yaw = points[-2].yaw

    speeds = [0.0 for _ in points]
    for idx in range(1, len(points)):
        dx = points[idx].x - points[idx - 1].x
        dy = points[idx].y - points[idx - 1].y
        speeds[idx] = math.sqrt(dx * dx + dy * dy) / dt
    for idx, hint in enumerate(speed_hint):
        if hint is not None:
            speeds[idx] = float(hint)
    for idx, point in enumerate(points):
        point.v = speeds[idx]
        if idx > 0:
            point.a = (speeds[idx] - speeds[idx - 1]) / dt
    return points


def _build_plan(
    scene: SceneEdit,
    *,
    map_index: Optional[MapIndex],
    voice_lead_time: float,
    robot_precue_lead: float,
) -> Tuple[Plan, List[EventPlan]]:
    actors = []
    for actor in scene.actors:
        trajectory = _build_trajectory(
            actor.keyframes,
            dt=scene.dt,
            duration=scene.duration,
            map_index=map_index,
        )
        actors.append(
            ActorPlan(
                actor_id=actor.actor_id,
                kind=actor.kind,
                role=actor.role,
                blueprint=actor.blueprint,
                controller=actor.controller,
                trajectory=trajectory,
            )
        )

    events_plan: List[EventPlan] = []
    for idx, event in enumerate(scene.events, start=1):
        t_event = float(event.t_event)
        t_voice = event.t_voice_start
        if t_voice is None:
            t_voice = max(0.0, t_event - voice_lead_time)
        t_robot = event.t_robot_precue
        if t_robot is None:
            t_robot = max(0.0, t_voice - robot_precue_lead)
        events_plan.append(
            EventPlan(
                t_event=t_event,
                event_type=event.event_type,
                expected_action=event.expected_action,
                audio_id=event.audio_id or f"{event.event_type}_{idx:02d}",
                t_voice_start=t_voice,
                t_robot_precue=t_robot,
            )
        )

    plan = Plan(
        episode_id=scene.episode_id,
        town=scene.town,
        seed=scene.seed,
        dt=scene.dt,
        duration=scene.duration,
        actors=actors,
        events_plan=events_plan,
    )
    return plan, events_plan


# DocRef: technical_details.md#4.1
def _load_scene(scene_path: Path) -> SceneEdit:
    raw = json.loads(scene_path.read_text())
    if not isinstance(raw, dict):
        raise ValueError("scene_edit.json must be a JSON object.")
    actors = []
    for item in raw.get("actors") or []:
        keyframes = [
            Keyframe(
                t=float(k["t"]),
                x=float(k["x"]),
                y=float(k["y"]),
                v=float(k["v"]) if k.get("v") is not None else None,
            )
            for k in item.get("keyframes") or []
        ]
        actors.append(
            ActorState(
                actor_id=str(item.get("id", "actor")),
                kind=str(item.get("kind", "vehicle")),
                role=str(item.get("role", "")),
                blueprint=str(item.get("blueprint", "")),
                controller=str(item.get("controller", "teleport")),
                keyframes=keyframes,
                color=str(item.get("color", "#6b9080")),
            )
        )
    events = []
    for item in raw.get("events") or []:
        events.append(
            EventMarker(
                t_event=float(item.get("t_event", 0.0)),
                event_type=str(item.get("event_type", "")),
                expected_action=str(item.get("expected_action", "")),
                audio_id=item.get("audio_id"),
                t_voice_start=item.get("t_voice_start"),
                t_robot_precue=item.get("t_robot_precue"),
            )
        )
    return SceneEdit(
        version=str(raw.get("version", "0.1")),
        episode_id=str(raw.get("episode_id", "episode")),
        town=str(raw.get("town", "")),
        dt=float(raw.get("dt", 0.05)),
        duration=float(raw.get("duration", 60.0)),
        map_dir=str(raw.get("map_dir", "")),
        seed=int(raw.get("seed", 0)),
        actors=actors,
        events=events,
    )


# DocRef: technical_details.md#4.1
def _save_scene(scene: SceneEdit, scene_path: Path) -> None:
    payload = {
        "version": scene.version,
        "episode_id": scene.episode_id,
        "town": scene.town,
        "dt": scene.dt,
        "duration": scene.duration,
        "map_dir": scene.map_dir,
        "seed": scene.seed,
        "actors": [
            {
                "id": actor.actor_id,
                "kind": actor.kind,
                "role": actor.role,
                "blueprint": actor.blueprint,
                "controller": actor.controller,
                "color": actor.color,
                "keyframes": [
                    {"t": k.t, "x": k.x, "y": k.y, "v": k.v} for k in actor.keyframes
                ],
            }
            for actor in scene.actors
        ],
        "events": [
            {
                "t_event": event.t_event,
                "event_type": event.event_type,
                "expected_action": event.expected_action,
                "audio_id": event.audio_id,
                "t_voice_start": event.t_voice_start,
                "t_robot_precue": event.t_robot_precue,
            }
            for event in scene.events
        ],
    }
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    scene_path.write_text(json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8")


# DocRef: technical_details.md#2
class SceneEditor:
    def __init__(
        self,
        *,
        map_dir: Path,
        scene: SceneEdit,
        map_graph: Dict[str, Any],
        lane_geo: Dict[str, Any],
        junction_geo: Dict[str, Any],
        map_index: Optional[MapIndex],
        out_dir: Path,
        scene_path: Path,
        voice_lead: float,
        robot_precue: float,
        conflict_threshold_m: float,
        vehicle_lane_threshold_m: float,
        walker_lane_threshold_m: float,
    ) -> None:
        if plt is None:
            _init_matplotlib(False)
        self.map_dir = map_dir
        self.scene = scene
        self.map_graph = map_graph
        self.lane_geo = lane_geo
        self.junction_geo = junction_geo
        self.map_index = map_index
        self.out_dir = out_dir
        self.scene_path = scene_path
        self.voice_lead = voice_lead
        self.robot_precue = robot_precue
        self.conflict_threshold_m = conflict_threshold_m
        self.vehicle_lane_threshold_m = vehicle_lane_threshold_m
        self.walker_lane_threshold_m = walker_lane_threshold_m

        self.active_actor_id = scene.actors[0].actor_id if scene.actors else ""
        self.selected_keyframe: Optional[Tuple[str, int]] = None
        self.mode = "move"
        self.dragging = False
        self._suppress_text = False
        self.snap_enabled = False
        self.show_ttc = True
        self.show_kinematic = True
        self._undo_stack: List[str] = []
        self._redo_stack: List[str] = []
        self._max_undo = 50

        self.fig = plt.figure(figsize=(14, 10))
        self.ax_map = self.fig.add_axes([0.05, 0.10, 0.65, 0.85])
        self.ax_slider = self.fig.add_axes([0.05, 0.04, 0.65, 0.025])
        self.ax_actor = self.fig.add_axes([0.72, 0.88, 0.26, 0.10])
        self.ax_buttons = []
        self._buttons = []
        self._build_ui()
        self._plot_map()
        self._refresh_actor_radio()
        self._draw_all_actors()
        self._reset_view(None)  # Reset view to show all actors
        self._update_event_markers()
        self._update_time_marker(self.time_slider.val)
        self._connect_events()

    def _build_ui(self) -> None:
        self.time_slider = Slider(
            self.ax_slider,
            "t(s)",
            0.0,
            max(0.1, self.scene.duration),
            valinit=0.0,
        )
        self.time_slider.on_changed(self._update_time_marker)

        btn_h = 0.030
        btn_gap = 0.033
        col1_x = 0.72
        col2_x = 0.86
        btn_w = 0.12
        panel_w = 0.26

        # --- Mode toggle (RadioButtons replacing 3 separate buttons) ---
        ax_mode = self.fig.add_axes([col1_x, 0.82, panel_w, 0.08])
        self.mode_radio = RadioButtons(ax_mode, ["Add", "Move", "Delete"], active=1)
        self.mode_radio.on_clicked(self._on_mode_radio)

        # --- Action buttons (2 columns) ---
        left_buttons = [
            ("Snap", self._toggle_snap),
            ("Analyze", self._analyze),
            ("Save", self._save_scene_action),
        ]
        right_buttons = [
            ("Export", self._export_action),
            ("Undo", self._undo),
            ("Redo", self._redo),
        ]

        y = 0.78
        for label, callback in left_buttons:
            ax = self.fig.add_axes([col1_x, y, btn_w, btn_h])
            btn = Button(ax, label)
            btn.on_clicked(callback)
            self._buttons.append(btn)
            self.ax_buttons.append(ax)
            y -= btn_gap

        y = 0.78
        for label, callback in right_buttons:
            ax = self.fig.add_axes([col2_x, y, btn_w, btn_h])
            btn = Button(ax, label)
            btn.on_clicked(callback)
            self._buttons.append(btn)
            self.ax_buttons.append(ax)
            y -= btn_gap

        # --- Reset button ---
        ax_reset = self.fig.add_axes([col1_x, 0.78 - 3 * btn_gap, panel_w, btn_h])
        btn_reset = Button(ax_reset, "Reset View")
        btn_reset.on_clicked(self._reset_view)
        self._buttons.append(btn_reset)

        # --- Event section ---
        input_h = 0.026
        input_gap = 0.032
        evt_y = 0.62

        ax_event_type = self.fig.add_axes([col1_x, evt_y, panel_w, input_h])
        self.event_type_box = TextBox(ax_event_type, "Event", initial="lane_change")
        evt_y -= input_gap

        ax_event_action = self.fig.add_axes([col1_x, evt_y, panel_w, input_h])
        self.event_action_box = TextBox(ax_event_action, "Action", initial="lane_change")
        evt_y -= input_gap

        ax_add_evt = self.fig.add_axes([col1_x, evt_y, btn_w, btn_h])
        btn_add_evt = Button(ax_add_evt, "AddEvt")
        btn_add_evt.on_clicked(self._add_event)
        self._buttons.append(btn_add_evt)

        ax_del_evt = self.fig.add_axes([col2_x, evt_y, btn_w, btn_h])
        btn_del_evt = Button(ax_del_evt, "DelEvt")
        btn_del_evt.on_clicked(self._delete_event_btn)
        self._buttons.append(btn_del_evt)

        # --- Keyframe edit fields ---
        kf_y = evt_y - input_gap - 0.005

        ax_kf_time = self.fig.add_axes([col1_x, kf_y, panel_w, input_h])
        self.kf_time_box = TextBox(ax_kf_time, "KF t", initial="")
        self.kf_time_box.on_submit(self._update_selected_time)
        kf_y -= input_gap

        ax_kf_speed = self.fig.add_axes([col1_x, kf_y, panel_w, input_h])
        self.kf_speed_box = TextBox(ax_kf_speed, "KF v", initial="")
        self.kf_speed_box.on_submit(self._update_selected_speed)
        kf_y -= input_gap

        # --- Actor management ---
        ax_new_actor = self.fig.add_axes([col1_x, kf_y, panel_w, input_h])
        self.new_actor_box = TextBox(ax_new_actor, "Actor", initial="npc1,vehicle,npc")
        kf_y -= input_gap

        ax_add_actor = self.fig.add_axes([col1_x, kf_y, btn_w, btn_h])
        btn_add_actor = Button(ax_add_actor, "Add Actor")
        btn_add_actor.on_clicked(self._add_actor)
        self._buttons.append(btn_add_actor)

        ax_del_actor = self.fig.add_axes([col2_x, kf_y, btn_w, btn_h])
        btn_del_actor = Button(ax_del_actor, "Del Actor")
        btn_del_actor.on_clicked(self._delete_actor)
        self._buttons.append(btn_del_actor)

        # --- Keyframe table (in-GUI) ---
        kf_table_y = kf_y - btn_gap - 0.005
        kf_table_h = max(0.01, kf_table_y - 0.01)
        self.ax_kf_table = self.fig.add_axes([col1_x, 0.01, panel_w, kf_table_h])
        self.ax_kf_table.set_axis_off()
        self._kf_table_text = self.ax_kf_table.text(
            0.0, 1.0, "", fontsize=6.5, va="top", ha="left", family="monospace",
            transform=self.ax_kf_table.transAxes,
        )

        # --- Status bar ---
        self.ax_status = self.fig.add_axes([0.05, 0.005, 0.65, 0.025])
        self.ax_status.set_axis_off()
        self.status_text = self.ax_status.text(
            0.0, 0.5,
            "Mode: move | Snap: OFF | Keys: a/m/d=mode s=snap space=analyze t=table h=help",
            fontsize=8, va="center", ha="left", family="monospace"
        )
        self._analysis_summary = ""

    def _delete_event_btn(self, _event: Any) -> None:
        """Button callback to delete nearest event at current slider time."""
        t = float(self.time_slider.val)
        if not self._delete_nearest_event(t, tolerance_s=2.0):
            logging.info("No event near t=%.2fs to delete", t)

    def _update_status(self) -> None:
        snap_str = "ON" if self.snap_enabled else "OFF"
        # Find selected actor info
        actor_id = "?"
        kf_info = ""
        if hasattr(self, "actor_radio") and self.scene.actors:
            actor_id = self.scene.actors[0].actor_id
            for a in self.scene.actors:
                if hasattr(self.actor_radio, 'value_selected') and self.actor_radio.value_selected == a.actor_id:
                    actor_id = a.actor_id
                    break
            actor = self._actor_by_id(actor_id)
            if actor:
                kf_info = f" | KF: {len(actor.keyframes)}"
        status = f"Mode: {self.mode} | Snap: {snap_str} | Actor: {actor_id}{kf_info}"
        if self._analysis_summary:
            status += f" | {self._analysis_summary}"
        else:
            status += " | Keys: a/m/d s=snap space=analyze h=help"
        self.status_text.set_text(status)
        self.fig.canvas.draw_idle()

    def _refresh_kf_table(self) -> None:
        """Update the in-GUI keyframe table for the selected actor."""
        if not hasattr(self, "_kf_table_text"):
            return
        actor = None
        if hasattr(self, "actor_radio"):
            for a in self.scene.actors:
                if hasattr(self.actor_radio, 'value_selected') and self.actor_radio.value_selected == a.actor_id:
                    actor = a
                    break
        if actor is None and self.scene.actors:
            actor = self.scene.actors[0]
        if actor is None:
            self._kf_table_text.set_text("")
            return
        lines = [f"KF Table: {actor.actor_id} ({len(actor.keyframes)} pts)"]
        lines.append(f"{'#':<3} {'t':>6} {'x':>8} {'y':>8} {'v':>6}")
        lines.append("-" * 35)
        for idx, kf in enumerate(actor.keyframes[:15]):  # limit display
            v_str = f"{kf.v:.1f}" if kf.v is not None else "-"
            lines.append(f"{idx:<3} {kf.t:>6.1f} {kf.x:>8.1f} {kf.y:>8.1f} {v_str:>6}")
        if len(actor.keyframes) > 15:
            lines.append(f"  ... +{len(actor.keyframes) - 15} more")
        self._kf_table_text.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()

    def _plot_map(self) -> None:
        for feature in self.junction_geo.get("features", []):
            geom = feature.get("geometry") or {}
            if geom.get("type") != "Polygon":
                continue
            coords = geom.get("coordinates") or []
            if not coords:
                continue
            poly = coords[0]
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            self.ax_map.fill(xs, ys, color="#f2d3a4", alpha=0.25, linewidth=0)
        for feature in self.lane_geo.get("features", []):
            geom = feature.get("geometry") or {}
            if geom.get("type") != "LineString":
                continue
            coords = geom.get("coordinates") or []
            if not coords:
                continue
            xs = [p[0] for p in coords]
            ys = [p[1] for p in coords]
            self.ax_map.plot(xs, ys, color="#3b6ea5", linewidth=0.6, alpha=0.5)
        self.ax_map.set_aspect("equal")
        self.ax_map.set_title("Scene Editor MVP")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")

        self.selected_marker = self.ax_map.scatter([], [], s=80, facecolors="none", edgecolors="#ffd166")
        self.event_scatter = self.ax_map.scatter([], [], s=40, color="#f4a261", edgecolors="#222222", linewidth=0.5)

    def _actor_by_id(self, actor_id: str) -> Optional[ActorState]:
        for actor in self.scene.actors:
            if actor.actor_id == actor_id:
                return actor
        return None

    def _refresh_actor_radio(self) -> None:
        if self.ax_actor:
            self.ax_actor.clear()
        labels = [actor.actor_id for actor in self.scene.actors] or ["<none>"]
        self.actor_radio = RadioButtons(self.ax_actor, labels, active=0 if labels else 0)
        self.actor_radio.on_clicked(self._on_actor_select)

    def _draw_all_actors(self) -> None:
        colors = ["#d1495b", "#6b9080", "#577590", "#bc4749", "#386641"]
        logging.info("Drawing %d actors", len(self.scene.actors))
        for idx, actor in enumerate(self.scene.actors):
            actor.color = colors[idx % len(colors)]
            logging.info("  Drawing actor %s with color %s, %d keyframes", actor.actor_id, actor.color, len(actor.keyframes))
            self._draw_actor(actor)

    def _draw_actor(self, actor: ActorState) -> None:
        xs = [k.x for k in actor.keyframes]
        ys = [k.y for k in actor.keyframes]
        logging.debug("  Actor %s: xs=%s, ys=%s", actor.actor_id, xs[:3], ys[:3])
        offsets = _as_offsets(list(zip(xs, ys)))
        if actor.line is None:
            actor.line, = self.ax_map.plot(xs, ys, color=actor.color, linewidth=1.8, alpha=0.9)
        else:
            actor.line.set_data(xs, ys)
        if actor.scatter is None:
            actor.scatter = self.ax_map.scatter(xs, ys, s=30, color=actor.color, zorder=3)
        else:
            actor.scatter.set_offsets(offsets)
        if actor.ghost is None:
            actor.ghost = self.ax_map.scatter([], [], s=25, color=actor.color, alpha=0.6)

    def _update_actor_artists(self) -> None:
        for actor in self.scene.actors:
            self._draw_actor(actor)
        self._update_selected_marker()
        self._update_event_markers()
        self._refresh_kf_table()
        self._update_status()
        self.fig.canvas.draw_idle()

    def _update_selected_marker(self) -> None:
        if self.selected_keyframe is None:
            self.selected_marker.set_offsets(np.empty((0, 2)))
            self._suppress_text = True
            self.kf_time_box.set_val("")
            self.kf_speed_box.set_val("")
            self._suppress_text = False
            self.fig.canvas.draw_idle()
            return
        actor_id, index = self.selected_keyframe
        actor = self._actor_by_id(actor_id)
        if actor is None or index >= len(actor.keyframes):
            self.selected_marker.set_offsets(np.empty((0, 2)))
            self.fig.canvas.draw_idle()
            return
        kf = actor.keyframes[index]
        self.selected_marker.set_offsets([[kf.x, kf.y]])
        self._suppress_text = True
        self.kf_time_box.set_val(f"{kf.t:.2f}")
        self.kf_speed_box.set_val("" if kf.v is None else f"{kf.v:.2f}")
        self._suppress_text = False
        self.fig.canvas.draw_idle()

    def _on_mode_radio(self, label: str) -> None:
        """Handle mode RadioButtons selection."""
        self.mode = label.lower()
        self._update_status()

    def _set_mode_add(self, _event: Any) -> None:
        self.mode = "add"
        self._sync_mode_radio()
        self._update_status()

    def _set_mode_move(self, _event: Any) -> None:
        self.mode = "move"
        self._sync_mode_radio()
        self._update_status()

    def _set_mode_delete(self, _event: Any) -> None:
        self.mode = "delete"
        self._sync_mode_radio()
        self._update_status()

    def _sync_mode_radio(self) -> None:
        """Sync RadioButtons widget to current mode (called from keyboard shortcuts)."""
        mode_map = {"add": 0, "move": 1, "delete": 2}
        idx = mode_map.get(self.mode, 1)
        if hasattr(self, "mode_radio"):
            self.mode_radio.set_active(idx)

    def _toggle_snap(self, _event: Any) -> None:
        self.snap_enabled = not self.snap_enabled
        status = "ON" if self.snap_enabled else "OFF"
        logging.info("Snap to centerline: %s", status)
        self._update_status()

    # DocRef: technical_details.md#2.7
    def _push_undo(self) -> None:
        """Save current state to undo stack."""
        state = json.dumps(self._scene_to_dict())
        self._undo_stack.append(state)
        if len(self._undo_stack) > self._max_undo:
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def _scene_to_dict(self) -> Dict[str, Any]:
        """Serialize scene state for undo/redo."""
        return {
            "actors": [
                {
                    "id": actor.actor_id,
                    "kind": actor.kind,
                    "role": actor.role,
                    "blueprint": actor.blueprint,
                    "controller": actor.controller,
                    "color": actor.color,
                    "keyframes": [{"t": k.t, "x": k.x, "y": k.y, "v": k.v} for k in actor.keyframes],
                }
                for actor in self.scene.actors
            ],
            "events": [
                {
                    "t_event": e.t_event,
                    "event_type": e.event_type,
                    "expected_action": e.expected_action,
                    "audio_id": e.audio_id,
                    "t_voice_start": e.t_voice_start,
                    "t_robot_precue": e.t_robot_precue,
                }
                for e in self.scene.events
            ],
        }

    def _clear_actor_artists(self) -> None:
        """Remove all matplotlib artists for actors."""
        for actor in self.scene.actors:
            if actor.line is not None:
                actor.line.remove()
                actor.line = None
            if actor.scatter is not None:
                actor.scatter.remove()
                actor.scatter = None
            if actor.ghost is not None:
                actor.ghost.remove()
                actor.ghost = None
            if actor.conflict_scatter is not None:
                actor.conflict_scatter.remove()
                actor.conflict_scatter = None
            if actor.infeasible_scatter is not None:
                actor.infeasible_scatter.remove()
                actor.infeasible_scatter = None
            if actor.ttc_scatter is not None:
                actor.ttc_scatter.remove()
                actor.ttc_scatter = None
            if actor.kinematic_scatter is not None:
                actor.kinematic_scatter.remove()
                actor.kinematic_scatter = None

    def _restore_from_dict(self, data: Dict[str, Any]) -> None:
        """Restore scene state from dict."""
        self._clear_actor_artists()
        self.scene.actors = []
        for item in data.get("actors", []):
            keyframes = [
                Keyframe(t=k["t"], x=k["x"], y=k["y"], v=k.get("v"))
                for k in item.get("keyframes", [])
            ]
            actor = ActorState(
                actor_id=str(item.get("id", "actor")),
                kind=str(item.get("kind", "vehicle")),
                role=str(item.get("role", "")),
                blueprint=str(item.get("blueprint", "")),
                controller=str(item.get("controller", "teleport")),
                keyframes=keyframes,
                color=str(item.get("color", "#6b9080")),
            )
            self.scene.actors.append(actor)
        self.scene.events = []
        for item in data.get("events", []):
            self.scene.events.append(
                EventMarker(
                    t_event=float(item.get("t_event", 0.0)),
                    event_type=str(item.get("event_type", "")),
                    expected_action=str(item.get("expected_action", "")),
                    audio_id=item.get("audio_id"),
                    t_voice_start=item.get("t_voice_start"),
                    t_robot_precue=item.get("t_robot_precue"),
                )
            )
        if self.scene.actors:
            self.active_actor_id = self.scene.actors[0].actor_id
        self.selected_keyframe = None

    def _undo(self, _event: Any) -> None:
        if not self._undo_stack:
            logging.info("Nothing to undo")
            return
        current = json.dumps(self._scene_to_dict())
        self._redo_stack.append(current)
        state = self._undo_stack.pop()
        self._restore_from_dict(json.loads(state))
        self._refresh_actor_radio()
        self._draw_all_actors()
        self._update_actor_artists()
        logging.info("Undo applied")

    def _redo(self, _event: Any) -> None:
        if not self._redo_stack:
            logging.info("Nothing to redo")
            return
        current = json.dumps(self._scene_to_dict())
        self._undo_stack.append(current)
        state = self._redo_stack.pop()
        self._restore_from_dict(json.loads(state))
        self._refresh_actor_radio()
        self._draw_all_actors()
        self._update_actor_artists()
        logging.info("Redo applied")

    def _on_actor_select(self, label: str) -> None:
        self.active_actor_id = label
        self.selected_keyframe = None
        self._update_selected_marker()
        self._refresh_kf_table()
        self._update_status()

    def _update_selected_time(self, text: str) -> None:
        if self._suppress_text:
            return
        if self.selected_keyframe is None:
            return
        actor_id, index = self.selected_keyframe
        actor = self._actor_by_id(actor_id)
        if actor is None or index >= len(actor.keyframes):
            return
        try:
            value = float(text)
        except ValueError:
            return
        keyframe = actor.keyframes[index]
        keyframe.t = max(0.0, min(value, self.scene.duration))
        actor.keyframes.sort(key=lambda k: k.t)
        self.selected_keyframe = (actor_id, actor.keyframes.index(keyframe))
        self._update_actor_artists()

    def _update_selected_speed(self, text: str) -> None:
        if self._suppress_text:
            return
        if self.selected_keyframe is None:
            return
        actor_id, index = self.selected_keyframe
        actor = self._actor_by_id(actor_id)
        if actor is None or index >= len(actor.keyframes):
            return
        if not text.strip():
            actor.keyframes[index].v = None
        else:
            try:
                actor.keyframes[index].v = float(text)
            except ValueError:
                return
        self._update_actor_artists()

    @staticmethod
    def _textbox_value(box: TextBox) -> str:
        raw = box.text
        return raw if isinstance(raw, str) else raw.get_text()

    def _add_actor(self, _event: Any) -> None:
        text = self._textbox_value(self.new_actor_box).strip()
        if not text:
            return
        parts = [p.strip() for p in text.split(",")]
        if len(parts) < 3:
            return
        actor_id, kind, role = parts[:3]
        if any(actor.actor_id == actor_id for actor in self.scene.actors):
            return
        ego = self._actor_by_id("ego")
        if ego and ego.keyframes:
            start_kf = ego.keyframes[0]
            end_kf = ego.keyframes[-1]
            offset_y = 4.0 if kind == "vehicle" else 2.0
            keyframes = [
                Keyframe(t=0.0, x=start_kf.x, y=start_kf.y + offset_y),
                Keyframe(t=self.scene.duration, x=end_kf.x, y=end_kf.y + offset_y),
            ]
        else:
            keyframes = _default_keyframes_from_map(self.map_graph, self.scene.duration)
            if not keyframes:
                keyframes = [Keyframe(t=0.0, x=0.0, y=0.0), Keyframe(t=self.scene.duration, x=50.0, y=0.0)]
        actor = ActorState(
            actor_id=actor_id,
            kind=kind,
            role=role,
            blueprint="vehicle.tesla.model3" if kind == "vehicle" else "walker.pedestrian.0001",
            controller="teleport",
            keyframes=keyframes,
        )
        self.scene.actors.append(actor)
        self.active_actor_id = actor_id
        self._refresh_actor_radio()
        self._draw_all_actors()
        self._update_actor_artists()
        self.mode = "add"
        self._update_status()
        logging.info("Added actor '%s'. Switch to Add KF mode. Click on map to add keyframes.", actor_id)

    def _delete_actor(self, _event: Any) -> None:
        if not self.active_actor_id or self.active_actor_id == "ego":
            return
        self.scene.actors = [a for a in self.scene.actors if a.actor_id != self.active_actor_id]
        self.active_actor_id = self.scene.actors[0].actor_id if self.scene.actors else ""
        self._refresh_actor_radio()
        self._update_actor_artists()

    def _add_event(self, _event: Any) -> None:
        self._push_undo()
        event_type = self._textbox_value(self.event_type_box).strip() or "event"
        expected_action = self._textbox_value(self.event_action_box).strip() or event_type
        t_event = float(self.time_slider.val)
        self.scene.events.append(
            EventMarker(
                t_event=t_event,
                event_type=event_type,
                expected_action=expected_action,
            )
        )
        self.scene.events.sort(key=lambda e: e.t_event)
        logging.info("Added event '%s' at t=%.2fs", event_type, t_event)
        self._update_event_markers()

    def _delete_nearest_event(self, t: float, tolerance_s: float = 1.0) -> bool:
        """Delete the event nearest to time t within tolerance."""
        if not self.scene.events:
            return False
        best_idx = None
        best_dt = tolerance_s
        for idx, event in enumerate(self.scene.events):
            dt = abs(event.t_event - t)
            if dt < best_dt:
                best_dt = dt
                best_idx = idx
        if best_idx is not None:
            self._push_undo()
            removed = self.scene.events.pop(best_idx)
            logging.info("Deleted event '%s' at t=%.2fs", removed.event_type, removed.t_event)
            self._update_event_markers()
            return True
        return False

    def _update_event_markers(self) -> None:
        ego = self._actor_by_id("ego")
        if ego is None or not ego.keyframes or not self.scene.events:
            self.event_scatter.set_offsets(np.empty((0, 2)))
            return
        points = []
        for event in self.scene.events:
            x, y = _interpolate_position(ego.keyframes, event.t_event)
            points.append((x, y))
        self.event_scatter.set_offsets(points)
        self.fig.canvas.draw_idle()

    def _update_time_marker(self, t: float) -> None:
        for actor in self.scene.actors:
            x, y = _interpolate_position(actor.keyframes, float(t))
            if actor.ghost is not None:
                actor.ghost.set_offsets([[x, y]])
        self.fig.canvas.draw_idle()

    def _closest_keyframe(self, actor: ActorState, x: float, y: float, radius: float = 3.0) -> Optional[int]:
        best = None
        best_d2 = radius * radius
        for idx, kf in enumerate(actor.keyframes):
            d2 = (kf.x - x) ** 2 + (kf.y - y) ** 2
            if d2 <= best_d2:
                best = idx
                best_d2 = d2
        return best

    def _on_press(self, event: Any) -> None:
        if event.inaxes != self.ax_map:
            return
        actor = self._actor_by_id(self.active_actor_id)
        if actor is None:
            return
        if self.mode == "add":
            self._push_undo()
            t = float(self.time_slider.val)
            v = None
            text = self._textbox_value(self.kf_speed_box).strip()
            if text:
                try:
                    v = float(text)
                except ValueError:
                    v = None
            kf_x, kf_y = event.xdata, event.ydata
            if self.snap_enabled and self.map_index is not None:
                snap_x, snap_y, snap_dist = self.map_index.snap_to_centerline(kf_x, kf_y)
                if snap_dist < 10.0:
                    kf_x, kf_y = snap_x, snap_y
                    logging.info("Snapped keyframe to centerline (offset: %.2fm)", snap_dist)
            new_kf = Keyframe(t=t, x=kf_x, y=kf_y, v=v)
            actor.keyframes.append(new_kf)
            actor.keyframes.sort(key=lambda k: k.t)
            self.selected_keyframe = (actor.actor_id, actor.keyframes.index(new_kf))
            self._update_actor_artists()
            return
        idx = self._closest_keyframe(actor, event.xdata, event.ydata)
        if idx is None:
            self.selected_keyframe = None
            self._update_selected_marker()
            return
        if self.mode == "delete":
            self._push_undo()
            actor.keyframes.pop(idx)
            self.selected_keyframe = None
            self._update_actor_artists()
            return
        if self.mode == "move":
            self._push_undo()
            self.selected_keyframe = (actor.actor_id, idx)
            self.dragging = True
            self._update_selected_marker()

    def _on_motion(self, event: Any) -> None:
        if not self.dragging or self.selected_keyframe is None or event.inaxes != self.ax_map:
            return
        actor_id, index = self.selected_keyframe
        actor = self._actor_by_id(actor_id)
        if actor is None or index >= len(actor.keyframes):
            return
        kf_x, kf_y = float(event.xdata), float(event.ydata)
        if self.snap_enabled and self.map_index is not None:
            snap_x, snap_y, snap_dist = self.map_index.snap_to_centerline(kf_x, kf_y)
            if snap_dist < 10.0:
                kf_x, kf_y = snap_x, snap_y
        actor.keyframes[index].x = kf_x
        actor.keyframes[index].y = kf_y
        self._update_actor_artists()

    def _on_release(self, _event: Any) -> None:
        self.dragging = False

    def _connect_events(self) -> None:
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)

    def _on_key_press(self, event: Any) -> None:
        """Handle keyboard shortcuts."""
        if event.key == "a":
            self._set_mode_add(None)
            logging.info("Mode: Add keyframe")
        elif event.key == "m":
            self._set_mode_move(None)
            logging.info("Mode: Move keyframe")
        elif event.key == "d":
            self._set_mode_delete(None)
            logging.info("Mode: Delete keyframe")
        elif event.key == "s":
            self._toggle_snap(None)
        elif event.key == "e":
            t = float(self.time_slider.val)
            if not self._delete_nearest_event(t):
                logging.info("No event near t=%.2fs to delete", t)
        elif event.key == "ctrl+z":
            self._undo(None)
        elif event.key == "ctrl+y" or event.key == "ctrl+shift+z":
            self._redo(None)
        elif event.key == "ctrl+s":
            self._save_scene_action(None)
        elif event.key == "ctrl+e":
            self._export_action(None)
        elif event.key == "space" or event.key == " ":
            logging.info("Running analysis...")
            self._analyze(None)
        elif event.key == "r":
            self._reset_view(None)
        elif event.key == "h":
            self._show_help()
        elif event.key == "t":
            self._show_keyframe_table()
        elif event.key == "delete" or event.key == "backspace":
            if self.selected_keyframe is not None:
                actor_id, idx = self.selected_keyframe
                actor = self._actor_by_id(actor_id)
                if actor and idx < len(actor.keyframes):
                    self._push_undo()
                    actor.keyframes.pop(idx)
                    self.selected_keyframe = None
                    self._update_actor_artists()

    def _show_help(self) -> None:
        """Print keyboard shortcuts to console."""
        help_text = """
=== Interactive Editor Keyboard Shortcuts ===
  a         - Add keyframe mode
  m         - Move keyframe mode
  d         - Delete keyframe mode
  s         - Toggle snap to centerline
  e         - Delete nearest event at current time
  space     - Run analysis (conflicts, kinematic, TTC)
  r         - Reset view to fit all actors
  t         - Show keyframe table
  h         - Show this help
  Delete    - Delete selected keyframe
  Ctrl+Z    - Undo
  Ctrl+Y    - Redo
  Ctrl+S    - Save scene
  Ctrl+E    - Export plan.json
  Scroll    - Zoom in/out
=========================================="""
        print(help_text)
        logging.info("Help printed to console")

    def _show_keyframe_table(self) -> None:
        """Print keyframe table to console for quick editing reference."""
        print("\n" + "=" * 70)
        print("KEYFRAME TABLE")
        print("=" * 70)
        for actor in self.scene.actors:
            print(f"\n[{actor.actor_id}] ({actor.kind}, {actor.role})")
            print("-" * 50)
            print(f"{'Idx':<4} {'Time(s)':<10} {'X':<12} {'Y':<12} {'V(m/s)':<10}")
            print("-" * 50)
            for idx, kf in enumerate(actor.keyframes):
                v_str = f"{kf.v:.2f}" if kf.v is not None else "-"
                print(f"{idx:<4} {kf.t:<10.2f} {kf.x:<12.2f} {kf.y:<12.2f} {v_str:<10}")
        print("\n" + "=" * 70)
        print("Events:")
        print("-" * 50)
        if self.scene.events:
            for idx, evt in enumerate(self.scene.events):
                print(f"  {idx}: t={evt.t_event:.2f}s  type={evt.event_type}  action={evt.expected_action}")
        else:
            print("  (no events)")
        print("=" * 70 + "\n")
        logging.info("Keyframe table printed to console")

    def _on_scroll(self, event: Any) -> None:
        """Handle mouse scroll for zooming."""
        if event.inaxes != self.ax_map:
            return
        scale = 1.15 if event.button == "down" else 1 / 1.15
        xlim = self.ax_map.get_xlim()
        ylim = self.ax_map.get_ylim()
        xc = event.xdata
        yc = event.ydata
        new_xlim = [xc + (x - xc) * scale for x in xlim]
        new_ylim = [yc + (y - yc) * scale for y in ylim]
        self.ax_map.set_xlim(new_xlim)
        self.ax_map.set_ylim(new_ylim)
        self.fig.canvas.draw_idle()

    def _reset_view(self, _event: Any) -> None:
        """Reset map view to show all actors."""
        all_x = []
        all_y = []
        for actor in self.scene.actors:
            for kf in actor.keyframes:
                all_x.append(kf.x)
                all_y.append(kf.y)
        if not all_x:
            return
        margin = 50
        self.ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
        self.fig.canvas.draw_idle()
        logging.info("View reset to fit all actors")

    def _save_scene_action(self, _event: Any) -> None:
        _save_scene(self.scene, self.scene_path)
        logging.info("Scene saved: %s", self.scene_path)

    def _export_action(self, _event: Any) -> None:
        plan, events_plan = _build_plan(
            self.scene,
            map_index=self.map_index,
            voice_lead_time=self.voice_lead,
            robot_precue_lead=self.robot_precue,
        )
        self.out_dir.mkdir(parents=True, exist_ok=True)
        save_plan(self.out_dir / "plan.json", plan)
        save_events_plan(self.out_dir / "events_plan.json", events_plan)
        _save_scene(self.scene, self.scene_path)
        logging.info("Exported plan/events to %s", self.out_dir)

    # DocRef: technical_details.md#2.6
    def _analyze(self, _event: Any) -> None:
        plan, _ = _build_plan(
            self.scene,
            map_index=self.map_index,
            voice_lead_time=self.voice_lead,
            robot_precue_lead=self.robot_precue,
        )
        conflict_points: Dict[str, List[Tuple[float, float]]] = {a.actor_id: [] for a in plan.actors}
        infeasible_points: Dict[str, List[Tuple[float, float]]] = {a.actor_id: [] for a in plan.actors}
        kinematic_points: Dict[str, List[Tuple[float, float]]] = {a.actor_id: [] for a in plan.actors}
        ttc_points: Dict[str, List[Tuple[float, float, float]]] = {a.actor_id: [] for a in plan.actors}

        trajectories = {a.actor_id: a.trajectory for a in plan.actors}
        actor_ids = list(trajectories.keys())
        total_frames = min((len(traj) for traj in trajectories.values()), default=0)

        max_speed_mps = 30.0
        max_accel_mps2 = 8.0
        max_decel_mps2 = 10.0
        max_lat_accel_mps2 = 5.0
        ttc_warning_s = 3.0

        for actor_plan in plan.actors:
            traj = actor_plan.trajectory
            for idx, point in enumerate(traj):
                if abs(point.v) > max_speed_mps:
                    kinematic_points[actor_plan.actor_id].append((point.x, point.y))
                if point.a > max_accel_mps2 or point.a < -max_decel_mps2:
                    kinematic_points[actor_plan.actor_id].append((point.x, point.y))
                if idx > 0 and idx < len(traj) - 1:
                    prev_pt = traj[idx - 1]
                    next_pt = traj[idx + 1]
                    dx1 = point.x - prev_pt.x
                    dy1 = point.y - prev_pt.y
                    dx2 = next_pt.x - point.x
                    dy2 = next_pt.y - point.y
                    cross = dx1 * dy2 - dy1 * dx2
                    ds1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
                    ds2 = math.sqrt(dx2 * dx2 + dy2 * dy2)
                    if ds1 > 0.01 and ds2 > 0.01:
                        curvature = abs(cross) / (ds1 * ds2 * max(ds1, ds2))
                        v = max(0.1, abs(point.v))
                        lat_accel = curvature * v * v
                        if lat_accel > max_lat_accel_mps2:
                            kinematic_points[actor_plan.actor_id].append((point.x, point.y))

        for frame in range(total_frames):
            positions = {}
            velocities = {}
            for actor_id, traj in trajectories.items():
                point = traj[frame]
                positions[actor_id] = point
                yaw_rad = math.radians(point.yaw)
                velocities[actor_id] = (point.v * math.cos(yaw_rad), point.v * math.sin(yaw_rad))
                if self.map_index is not None:
                    dist, _ = self.map_index.nearest_lane(point.x, point.y)
                    threshold = self.vehicle_lane_threshold_m
                    if actor_id != "ego":
                        actor = self._actor_by_id(actor_id)
                        if actor and actor.kind.lower() in {"walker", "vru", "pedestrian"}:
                            threshold = self.walker_lane_threshold_m
                    if dist > threshold:
                        infeasible_points[actor_id].append((point.x, point.y))
            for i in range(len(actor_ids)):
                for j in range(i + 1, len(actor_ids)):
                    a_id = actor_ids[i]
                    b_id = actor_ids[j]
                    pa = positions.get(a_id)
                    pb = positions.get(b_id)
                    if pa is None or pb is None:
                        continue
                    dx = pa.x - pb.x
                    dy = pa.y - pb.y
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist < self.conflict_threshold_m:
                        conflict_points[a_id].append((pa.x, pa.y))
                        conflict_points[b_id].append((pb.x, pb.y))
                    va = velocities.get(a_id, (0, 0))
                    vb = velocities.get(b_id, (0, 0))
                    dvx = va[0] - vb[0]
                    dvy = va[1] - vb[1]
                    rel_v_sq = dvx * dvx + dvy * dvy
                    if rel_v_sq > 0.01 and dist > self.conflict_threshold_m:
                        approach = -(dx * dvx + dy * dvy)
                        if approach > 0:
                            ttc = dist / math.sqrt(rel_v_sq)
                            if ttc < ttc_warning_s:
                                ttc_points[a_id].append((pa.x, pa.y, ttc))
                                ttc_points[b_id].append((pb.x, pb.y, ttc))

        n_conflicts = sum(len(v) for v in conflict_points.values())
        n_infeasible = sum(len(v) for v in infeasible_points.values())
        n_kinematic = sum(len(v) for v in kinematic_points.values())
        n_ttc = sum(len(v) for v in ttc_points.values())

        for actor in self.scene.actors:
            if actor.conflict_scatter is None:
                actor.conflict_scatter = self.ax_map.scatter([], [], s=20, color="#e63946", alpha=0.8, label="Conflict")
            if actor.infeasible_scatter is None:
                actor.infeasible_scatter = self.ax_map.scatter([], [], s=18, color="#f6bd60", alpha=0.8, label="Off-lane")
            if actor.kinematic_scatter is None:
                actor.kinematic_scatter = self.ax_map.scatter([], [], s=16, color="#9d4edd", alpha=0.7, marker="^", label="Kinematic")
            if actor.ttc_scatter is None:
                actor.ttc_scatter = self.ax_map.scatter([], [], s=22, color="#ff006e", alpha=0.6, marker="x", label="TTC")
            actor.conflict_scatter.set_offsets(_as_offsets(conflict_points.get(actor.actor_id, [])))
            actor.infeasible_scatter.set_offsets(_as_offsets(infeasible_points.get(actor.actor_id, [])))
            kinematic_pts = kinematic_points.get(actor.actor_id, [])
            actor.kinematic_scatter.set_offsets(_as_offsets(kinematic_pts) if self.show_kinematic else np.empty((0, 2)))
            ttc_pts = [(p[0], p[1]) for p in ttc_points.get(actor.actor_id, [])]
            actor.ttc_scatter.set_offsets(_as_offsets(ttc_pts) if self.show_ttc else np.empty((0, 2)))

        self._analysis_summary = (
            f"C:{n_conflicts} OL:{n_infeasible} K:{n_kinematic} TTC:{n_ttc}"
        )
        logging.info(
            "Analysis: conflicts=%d, off-lane=%d, kinematic=%d, TTC warnings=%d",
            n_conflicts, n_infeasible, n_kinematic, n_ttc
        )
        self._update_status()
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        logging.info("Starting GUI with %d actors:", len(self.scene.actors))
        for actor in self.scene.actors:
            logging.info("  - %s: %d keyframes, line=%s, scatter=%s",
                        actor.actor_id, len(actor.keyframes),
                        actor.line is not None, actor.scatter is not None)
        xlim = self.ax_map.get_xlim()
        ylim = self.ax_map.get_ylim()
        logging.info("View bounds: x=[%.1f, %.1f], y=[%.1f, %.1f]", xlim[0], xlim[1], ylim[0], ylim[1])
        plt.show()


def _init_matplotlib(headless: bool) -> None:
    global plt, Button, RadioButtons, Slider, TextBox
    import matplotlib

    if headless:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt_module
    from matplotlib.widgets import Button as ButtonModule
    from matplotlib.widgets import RadioButtons as RadioButtonsModule
    from matplotlib.widgets import Slider as SliderModule
    from matplotlib.widgets import TextBox as TextBoxModule

    plt = plt_module
    Button = ButtonModule
    RadioButtons = RadioButtonsModule
    Slider = SliderModule
    TextBox = TextBoxModule


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Interactive 2D scene editor MVP.")
    parser.add_argument("--map-dir", required=True, help="Path to exported map directory")
    parser.add_argument("--scene", help="Path to scene_edit.json (optional)")
    parser.add_argument("--out", default="outputs", help="Output directory root")
    parser.add_argument("--episode-id", default="episode", help="Episode ID for exports")
    parser.add_argument("--town", help="Town override (default from map-dir name)")
    parser.add_argument("--dt", type=float, default=0.05, help="Trajectory dt (seconds)")
    parser.add_argument("--duration", type=float, default=60.0, help="Scenario duration (seconds)")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--conflict-threshold-m", type=float, default=3.0)
    parser.add_argument("--vehicle-lane-threshold-m", type=float, default=1.8)
    parser.add_argument("--walker-lane-threshold-m", type=float, default=2.8)
    parser.add_argument("--headless", action="store_true", help="Run without GUI and export outputs")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    _init_matplotlib(args.headless)

    map_dir = Path(args.map_dir)
    map_graph_path = map_dir / "map_graph.json"
    lane_geo_path = map_dir / "lane_centerlines.geojson"
    junction_geo_path = map_dir / "junction_areas.geojson"
    if not lane_geo_path.exists():
        raise SystemExit(f"Missing lane_centerlines.geojson in {map_dir}")
    if not junction_geo_path.exists():
        raise SystemExit(f"Missing junction_areas.geojson in {map_dir}")
    map_graph = json.loads(map_graph_path.read_text()) if map_graph_path.exists() else {}
    lane_geo = _load_geojson(lane_geo_path)
    junction_geo = _load_geojson(junction_geo_path)

    globals_cfg = _load_globals(Path(args.globals))
    events_cfg = globals_cfg.get("events", {}) if isinstance(globals_cfg, dict) else {}
    voice_lead = float(events_cfg.get("voice_lead_time_s", 3.0))
    robot_precue = float(events_cfg.get("robot_precue_lead_s", 0.5))

    map_points = _collect_lane_points(map_graph) if map_graph else []
    map_index = MapIndex(map_points) if map_points else None

    town = args.town or map_dir.name
    out_dir = Path(args.out) / args.episode_id
    scene_path = Path(args.scene) if args.scene else out_dir / "scene_edit.json"
    if args.scene and Path(args.scene).exists():
        scene = _load_scene(Path(args.scene))
    else:
        ego_blueprint = str(globals_cfg.get("ego", {}).get("blueprint", "vehicle.tesla.model3"))
        keyframes = _default_keyframes_from_map(map_graph, args.duration)
        if not keyframes:
            keyframes = [Keyframe(t=0.0, x=0.0, y=0.0), Keyframe(t=args.duration, x=5.0, y=0.0)]
        scene = SceneEdit(
            version="0.1",
            episode_id=args.episode_id,
            town=town,
            dt=args.dt,
            duration=args.duration,
            map_dir=str(map_dir),
            seed=0,
            actors=[
                ActorState(
                    actor_id="ego",
                    kind="vehicle",
                    role="ego",
                    blueprint=ego_blueprint,
                    controller="teleport",
                    keyframes=keyframes,
                    color="#d1495b",
                )
            ],
            events=[],
        )

    editor = SceneEditor(
        map_dir=map_dir,
        scene=scene,
        map_graph=map_graph,
        lane_geo=lane_geo,
        junction_geo=junction_geo,
        map_index=map_index,
        out_dir=out_dir,
        scene_path=scene_path,
        voice_lead=voice_lead,
        robot_precue=robot_precue,
        conflict_threshold_m=args.conflict_threshold_m,
        vehicle_lane_threshold_m=args.vehicle_lane_threshold_m,
        walker_lane_threshold_m=args.walker_lane_threshold_m,
    )
    if args.headless:
        editor._export_action(None)
        plt.close(editor.fig)
        return 0
    editor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
