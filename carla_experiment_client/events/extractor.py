"""Decision event extraction from per-tick state."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import carla

from ..utils import clamp


@dataclass
class EventExtractor:
    world: carla.World
    ego_vehicle: carla.Vehicle
    map_obj: carla.Map
    fps: int
    voice_lead_time_s: float = 3.0
    robot_precue_lead_s: float = 0.5
    min_event_time_s: float = 0.0

    _prev_speed: float = 0.0
    _prev_lane_id: Optional[int] = None
    _last_event_time: Dict[str, float] = field(default_factory=dict)
    _events: List[dict] = field(default_factory=list)

    def tick(self, snapshot: carla.WorldSnapshot, frame_index: int) -> None:
        if self.fps > 0:
            t = float(frame_index) / float(self.fps)
        else:
            t = float(snapshot.timestamp.elapsed_seconds)
        dt = float(snapshot.timestamp.delta_seconds or 1.0 / max(self.fps, 1))

        velocity = self.ego_vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
        accel = (speed - self._prev_speed) / dt

        control = self.ego_vehicle.get_control()
        waypoint = self.map_obj.get_waypoint(
            self.ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        lane_id = waypoint.lane_id if waypoint else None

        if lane_id is not None and self._prev_lane_id is not None and lane_id != self._prev_lane_id:
            lane_type = "lane_change_right" if control.steer > 0.1 else "lane_change_left"
            self._emit(t, lane_type)

        if control.brake > 0.6 or accel < -3.5:
            self._emit(t, "brake_hard")
        elif accel < -1.5:
            self._emit(t, "slow_down")

        if self.ego_vehicle.is_at_traffic_light():
            traffic_light = self.ego_vehicle.get_traffic_light()
            if traffic_light and traffic_light.state == carla.TrafficLightState.Red and speed < 0.2:
                self._emit(t, "stop_for_red_light")

        emergency = self._nearest_actor_with_role("emergency", 30.0)
        if emergency and (speed < 3.0 or control.brake > 0.2):
            self._emit(t, "yield_to_emergency")

        pedestrian = self._nearest_walker(12.0)
        if pedestrian and control.brake > 0.2:
            self._emit(t, "avoid_pedestrian")

        self._prev_speed = speed
        self._prev_lane_id = lane_id

    def finalize(self) -> List[dict]:
        return list(self._events)

    def _emit(self, t: float, event_type: str) -> None:
        if t < self.min_event_time_s:
            return
        if not self._should_emit(t, event_type):
            return
        decision, reason = self._format_event(event_type)
        event_index = len(self._events)
        t_event = round(t, 3)
        t_voice_start = round(
            clamp(t_event - self.voice_lead_time_s, 0.0, t_event), 3
        )
        t_robot_precue = round(
            clamp(t_voice_start - self.robot_precue_lead_s, 0.0, t_voice_start), 3
        )
        payload = {
            "t": t_event,
            "t_event": t_event,
            "t_voice_start": t_voice_start,
            "t_robot_precue": t_robot_precue,
            "type": event_type,
            "decision_text": decision,
            "reason_text": reason,
            "robot_precue_t": t_robot_precue,
            "audio_id": f"{event_type}_{event_index:02d}",
        }
        self._events.append(payload)
        self._last_event_time[event_type] = t

    def _should_emit(self, t: float, event_type: str, cooldown: float = 2.0) -> bool:
        last_t = self._last_event_time.get(event_type)
        if last_t is None:
            return True
        return (t - last_t) >= cooldown

    def _format_event(self, event_type: str) -> tuple[str, str]:
        mapping = {
            "lane_change_left": ("Change lane left", "Need to adjust position"),
            "lane_change_right": ("Change lane right", "Need to adjust position"),
            "brake_hard": ("Brake hard", "Obstacle or conflict ahead"),
            "slow_down": ("Slow down", "Traffic condition requires caution"),
            "stop_for_red_light": ("Stop", "Red light ahead"),
            "yield_to_emergency": ("Yield", "Emergency vehicle approaching"),
            "avoid_pedestrian": ("Brake", "Pedestrian crossing"),
        }
        return mapping.get(event_type, (event_type, ""))

    def _nearest_actor_with_role(self, role_name: str, radius: float) -> Optional[carla.Actor]:
        ego_loc = self.ego_vehicle.get_location()
        vehicles = self.world.get_actors().filter("vehicle.*")
        closest = None
        min_dist = radius
        for actor in vehicles:
            if actor.id == self.ego_vehicle.id:
                continue
            if actor.attributes.get("role_name") != role_name:
                continue
            dist = actor.get_location().distance(ego_loc)
            if dist < min_dist:
                min_dist = dist
                closest = actor
        return closest

    def _nearest_walker(self, radius: float) -> Optional[carla.Actor]:
        ego_loc = self.ego_vehicle.get_location()
        walkers = self.world.get_actors().filter("walker.pedestrian.*")
        closest = None
        min_dist = radius
        for actor in walkers:
            dist = actor.get_location().distance(ego_loc)
            if dist < min_dist:
                min_dist = dist
                closest = actor
        return closest
