"""Decision event extraction from per-tick state."""

from __future__ import annotations

import logging
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
    _prev_road_id: Optional[int] = None
    _last_event_time: Dict[str, float] = field(default_factory=dict)
    _events: List[dict] = field(default_factory=list)
    _prev_steer: float = 0.0
    _lane_change_pending: bool = False
    # Track vehicles in ego lane for cut-in detection
    _prev_vehicles_in_lane: set = field(default_factory=set)
    # Track vehicle distances for sudden approach detection
    _prev_vehicle_distances: dict = field(default_factory=dict)

    def tick(self, snapshot: carla.WorldSnapshot, frame_index: int) -> None:
        if self.fps > 0:
            t = float(frame_index) / float(self.fps)
        else:
            t = float(snapshot.timestamp.elapsed_seconds)
        self._current_t = t  # Store for logging in helper methods
        dt = float(snapshot.timestamp.delta_seconds or 1.0 / max(self.fps, 1))

        velocity = self.ego_vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
        accel = (speed - self._prev_speed) / dt

        control = self.ego_vehicle.get_control()
        waypoint = self.map_obj.get_waypoint(
            self.ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        lane_id = waypoint.lane_id if waypoint else None
        road_id = waypoint.road_id if waypoint else None

        # --- NPC Event Detection (scenario-driven events) ---

        # 1. Cut-in / Merge detection: another vehicle enters ego's lane ahead
        cut_in_vehicle = self._detect_cut_in(waypoint)
        if cut_in_vehicle:
            self._emit(t, "vehicle_cut_in")

        # 2. Pedestrian crossing detection: pedestrian near ego
        # Detect any walker within 25m - close pedestrian is a hazard
        pedestrian_nearby = self._nearest_walker(25.0)
        if pedestrian_nearby:
            ped_dist = pedestrian_nearby.get_location().distance(self.ego_vehicle.get_location())
            # Log when pedestrian first gets close (after potential relocation)
            if t > 35.0 and not hasattr(self, "_ped_post_trigger_logged"):
                logging.info("Pedestrian post-trigger: dist=%.1fm speed=%.1f t=%.1f",
                           ped_dist, speed, t)
                self._ped_post_trigger_logged = True
            # Emit when pedestrian is within 22m (account for relocation distances)
            if ped_dist < 22.0:
                self._emit(t, "avoid_pedestrian")

        # 3. Emergency vehicle approaching - detect when nearby and ego is responding
        # Check for any emergency vehicle (ambulance, firetruck, police) within range
        emergency = self._nearest_emergency_vehicle(100.0)
        if emergency:
            emergency_dist = emergency.get_location().distance(self.ego_vehicle.get_location())
            # Track distance history to detect approach (over multiple ticks)
            if not hasattr(self, "_emergency_dist_history"):
                self._emergency_dist_history = {}
            history = self._emergency_dist_history.setdefault(emergency.id, [])
            history.append(emergency_dist)
            if len(history) > 20:  # Keep last 20 samples (~2s at 10fps)
                history.pop(0)
            # Approaching = distance decreased over recent history
            is_approaching = False
            if len(history) >= 5:
                # Compare current dist to average of older samples
                old_avg = sum(history[:len(history)//2]) / (len(history)//2)
                is_approaching = emergency_dist < old_avg - 1.0  # Getting closer by 1m+
            # Trigger when emergency vehicle is approaching and close
            # Note: CARLA autopilot doesn't automatically yield to emergency vehicles,
            # so we don't require ego to be yielding - just detect the situation
            if emergency_dist < 40.0 and is_approaching:
                self._emit(t, "yield_to_emergency")
            # Also trigger if emergency is very close regardless of approach
            elif emergency_dist < 20.0:
                self._emit(t, "yield_to_emergency")

        # --- EGO Behavior Detection ---

        # Detect lane change (actual lane_id change)
        if lane_id is not None and self._prev_lane_id is not None and lane_id != self._prev_lane_id:
            # Only emit if road_id is same (not a junction transition)
            if road_id == self._prev_road_id:
                lane_type = "lane_change_right" if control.steer > 0.1 else "lane_change_left"
                self._emit(t, lane_type)
            self._lane_change_pending = False

        # Detect lane change intention (steering without lane change yet)
        steer_delta = abs(control.steer - self._prev_steer)
        if abs(control.steer) > 0.15 and steer_delta > 0.05 and not self._lane_change_pending:
            self._lane_change_pending = True

        # Braking events (only if no specific cause detected)
        if control.brake > 0.6 or accel < -3.5:
            self._emit(t, "brake_hard")
        elif control.brake > 0.3 or accel < -1.5:
            self._emit(t, "slow_down")

        # Traffic light
        if self.ego_vehicle.is_at_traffic_light():
            traffic_light = self.ego_vehicle.get_traffic_light()
            if traffic_light and traffic_light.state == carla.TrafficLightState.Red and speed < 0.5:
                self._emit(t, "stop_for_red_light")

        # Unprotected left turn detection (junction or approaching junction)
        is_near_junction = waypoint and (waypoint.is_junction or self._is_approaching_junction(waypoint))
        if is_near_junction and control.steer < -0.10:  # Turning left
            # Yield if braking or slow while turning left at junction
            if control.brake > 0.05 or speed < 8.0:
                self._emit(t, "yield_left_turn")

        self._prev_speed = speed
        self._prev_lane_id = lane_id
        self._prev_road_id = road_id
        self._prev_steer = control.steer

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
        """Check if enough time has passed since last event of this type.

        Different event types have different cooldowns to reduce noise:
        - Scenario-specific events (cut_in, pedestrian, emergency): 8s cooldown
        - Traffic light events: 10s cooldown (one per approach)
        - Generic braking events: 4s cooldown
        """
        # Scenario-specific events should be rare (typically 1 per scenario)
        scenario_events = {"vehicle_cut_in", "avoid_pedestrian", "yield_to_emergency", "yield_left_turn"}
        if event_type in scenario_events:
            cooldown = 8.0
        # Traffic light should only emit once per stop
        elif event_type == "stop_for_red_light":
            cooldown = 10.0
        # Generic braking/slowing should have moderate cooldown
        elif event_type in {"brake_hard", "slow_down"}:
            cooldown = 4.0
        # Lane changes are discrete events
        elif event_type in {"lane_change_left", "lane_change_right"}:
            cooldown = 3.0

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
            "avoid_pedestrian": ("Brake", "Pedestrian crossing ahead"),
            "vehicle_cut_in": ("Caution", "Vehicle merging into lane"),
            "yield_left_turn": ("Yield", "Oncoming traffic at intersection"),
        }
        return mapping.get(event_type, (event_type, ""))

    def _nearest_actor_with_role(self, role_name: str, radius: float) -> Optional[carla.Actor]:
        """Find nearest vehicle with a specific role_name or type_id pattern."""
        ego_loc = self.ego_vehicle.get_location()
        vehicles = self.world.get_actors().filter("vehicle.*")
        closest = None
        min_dist = radius
        for actor in vehicles:
            if actor.id == self.ego_vehicle.id:
                continue
            # Check role_name attribute
            actor_role = actor.attributes.get("role_name", "")
            # For emergency vehicles, also check type_id
            is_emergency = (role_name == "emergency" and
                           ("ambulance" in actor.type_id or "firetruck" in actor.type_id or
                            "police" in actor.type_id or actor_role == "emergency"))
            if actor_role == role_name or is_emergency:
                dist = actor.get_location().distance(ego_loc)
                if dist < min_dist:
                    min_dist = dist
                    closest = actor
        return closest

    def _nearest_emergency_vehicle(self, radius: float) -> Optional[carla.Actor]:
        """Find nearest emergency vehicle (ambulance, firetruck, police) by type_id.

        This is more reliable than checking role_name attribute as it directly
        inspects the vehicle's type_id which always contains the vehicle model.
        """
        ego_loc = self.ego_vehicle.get_location()
        vehicles = self.world.get_actors().filter("vehicle.*")
        closest = None
        min_dist = radius
        emergency_keywords = ("ambulance", "firetruck", "police")
        emergency_found = []
        for actor in vehicles:
            if actor.id == self.ego_vehicle.id:
                continue
            type_id = actor.type_id.lower()
            # Check if this is an emergency vehicle by type_id
            if any(kw in type_id for kw in emergency_keywords):
                dist = actor.get_location().distance(ego_loc)
                emergency_found.append((type_id, dist))
                if dist < min_dist:
                    min_dist = dist
                    closest = actor
        if emergency_found and not hasattr(self, "_emergency_logged"):
            logging.info("Emergency vehicles found: %s", emergency_found)
            self._emergency_logged = True
        elif not emergency_found and not hasattr(self, "_no_emergency_logged"):
            # Log all vehicle type_ids to understand what's available
            all_types = [v.type_id for v in vehicles if v.id != self.ego_vehicle.id][:5]
            logging.warning("No emergency vehicles found. Sample vehicle types: %s", all_types)
            self._no_emergency_logged = True
        return closest

    def _nearest_walker(self, radius: float) -> Optional[carla.Actor]:
        ego_loc = self.ego_vehicle.get_location()
        walkers = self.world.get_actors().filter("walker.pedestrian.*")
        closest = None
        min_dist = radius
        walker_found = []
        for actor in walkers:
            try:
                actor_loc = actor.get_location()
                dist = actor_loc.distance(ego_loc)
                walker_found.append((actor.type_id, dist, actor_loc.x, actor_loc.y))
            except RuntimeError:
                continue  # Actor may be destroyed
            if dist < min_dist:
                min_dist = dist
                closest = actor
        if walker_found and not hasattr(self, "_walker_logged"):
            logging.info("Walkers found: %s (closest: %.1fm)", walker_found, min_dist if closest else -1)
            self._walker_logged = True
        # Log walker position periodically between t=37-45s to debug relocation
        t_now = getattr(self, "_current_t", 0)
        if t_now > 37.0 and t_now < 45.0:
            last_log = getattr(self, "_last_walker_track_t", 0)
            if (t_now - last_log) >= 1.0 and walker_found:
                logging.info("Walker track t=%.1f: dist=%.1f pos=(%.1f,%.1f)",
                           t_now, walker_found[0][1], walker_found[0][2], walker_found[0][3])
                self._last_walker_track_t = t_now
        return closest

    def _pedestrian_in_front(self, radius: float) -> Optional[carla.Actor]:
        """Find nearest pedestrian that is in front of the ego vehicle.

        Checks if pedestrian is within radius AND in the forward hemisphere of ego.
        This is more specific than _nearest_walker which finds any walker in radius.
        """
        ego_loc = self.ego_vehicle.get_location()
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()
        walkers = self.world.get_actors().filter("walker.pedestrian.*")
        closest = None
        min_dist = radius
        for walker in walkers:
            walker_loc = walker.get_location()
            dist = walker_loc.distance(ego_loc)
            if dist >= min_dist:
                continue
            # Check if walker is in front of ego
            dx = walker_loc.x - ego_loc.x
            dy = walker_loc.y - ego_loc.y
            mag = (dx * dx + dy * dy) ** 0.5
            if mag < 1.0:
                continue
            # Dot product: positive means in front
            dot_front = (ego_fwd.x * dx + ego_fwd.y * dy) / mag
            if dot_front > 0.2:  # In front hemisphere
                min_dist = dist
                closest = walker
        return closest

    def _nearest_oncoming_vehicle(self, radius: float) -> Optional[carla.Actor]:
        """Find nearest vehicle that appears to be oncoming (opposite direction)."""
        ego_loc = self.ego_vehicle.get_location()
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()
        vehicles = self.world.get_actors().filter("vehicle.*")
        closest = None
        min_dist = radius
        for actor in vehicles:
            if actor.id == self.ego_vehicle.id:
                continue
            actor_loc = actor.get_location()
            dist = actor_loc.distance(ego_loc)
            if dist >= min_dist:
                continue
            # Check if vehicle is roughly in front and heading toward ego
            to_actor = carla.Vector3D(
                actor_loc.x - ego_loc.x,
                actor_loc.y - ego_loc.y,
                0.0
            )
            # Normalize
            mag = (to_actor.x**2 + to_actor.y**2) ** 0.5
            if mag < 1.0:
                continue
            to_actor.x /= mag
            to_actor.y /= mag
            # Check if actor is in front (dot product > 0)
            dot_front = ego_fwd.x * to_actor.x + ego_fwd.y * to_actor.y
            if dot_front < 0.3:  # Not in front
                continue
            # Check if actor is heading toward ego (opposite direction)
            actor_fwd = actor.get_transform().get_forward_vector()
            dot_dir = ego_fwd.x * actor_fwd.x + ego_fwd.y * actor_fwd.y
            if dot_dir > -0.5:  # Not oncoming
                continue
            min_dist = dist
            closest = actor
        return closest

    def _detect_cut_in(self, ego_waypoint: Optional[carla.Waypoint]) -> Optional[carla.Actor]:
        """Detect if a vehicle has just cut into ego's lane ahead.

        Detection methods:
        1. A vehicle that wasn't in ego's lane is now in ego's lane
        2. A vehicle that was far away suddenly appears close (teleport/relocate detection)
        """
        if ego_waypoint is None:
            return None

        ego_loc = self.ego_vehicle.get_location()
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()
        ego_lane_id = ego_waypoint.lane_id
        ego_road_id = ego_waypoint.road_id

        vehicles = self.world.get_actors().filter("vehicle.*")
        current_vehicles_in_lane = set()
        current_distances = {}
        detected_cut_in = None

        for actor in vehicles:
            if actor.id == self.ego_vehicle.id:
                continue

            actor_loc = actor.get_location()
            dist = actor_loc.distance(ego_loc)

            # Only check vehicles within 40m
            if dist > 40.0:
                continue

            # Check if vehicle is in front
            to_actor = carla.Vector3D(actor_loc.x - ego_loc.x, actor_loc.y - ego_loc.y, 0.0)
            mag = (to_actor.x**2 + to_actor.y**2) ** 0.5
            if mag < 1.0:
                continue
            to_actor.x /= mag
            to_actor.y /= mag
            dot_front = ego_fwd.x * to_actor.x + ego_fwd.y * to_actor.y
            if dot_front < 0.3:  # Not in front (relaxed)
                continue

            current_distances[actor.id] = dist

            # Check if vehicle is in ego's lane
            actor_waypoint = self.map_obj.get_waypoint(
                actor_loc, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if actor_waypoint is None:
                continue

            if actor_waypoint.road_id == ego_road_id and actor_waypoint.lane_id == ego_lane_id:
                current_vehicles_in_lane.add(actor.id)

                # Method 1: Vehicle just entered ego's lane
                if actor.id not in self._prev_vehicles_in_lane:
                    detected_cut_in = actor

                # Method 2: Vehicle suddenly teleported close (distance jumped significantly)
                if actor.id in self._prev_vehicle_distances:
                    prev_dist = self._prev_vehicle_distances[actor.id]
                    # If vehicle was far (>35m) and now close (<28m), it teleported
                    if prev_dist > 35.0 and dist < 28.0:
                        detected_cut_in = actor

                # Method 3: Vehicle appears close in ego's lane but wasn't tracked before
                # This catches relocated vehicles from random spawn fallback
                if actor.id not in self._prev_vehicle_distances and dist < 25.0:
                    detected_cut_in = actor

        self._prev_vehicles_in_lane = current_vehicles_in_lane
        self._prev_vehicle_distances = current_distances
        return detected_cut_in

    def _detect_pedestrian_in_path(self) -> Optional[carla.Actor]:
        """Detect if a pedestrian is in ego's path (crossing in front).

        Returns closest walker in front of ego within 20m.
        """
        ego_loc = self.ego_vehicle.get_location()
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()

        # Get all walkers - use filter for pedestrian actors
        walkers = self.world.get_actors().filter("walker.pedestrian.*")

        closest_walker = None
        min_dist = 20.0

        for walker in walkers:
            walker_loc = walker.get_location()
            dist = walker_loc.distance(ego_loc)

            if dist >= min_dist:
                continue

            # Vector from ego to walker
            dx = walker_loc.x - ego_loc.x
            dy = walker_loc.y - ego_loc.y
            mag = (dx * dx + dy * dy) ** 0.5
            if mag < 1.0:
                continue

            # Check if walker is in front
            dot_front = (ego_fwd.x * dx + ego_fwd.y * dy) / mag
            if dot_front > 0.0:  # In front hemisphere
                min_dist = dist
                closest_walker = walker

        return closest_walker

    def _is_vehicle_behind(self, vehicle: carla.Actor) -> bool:
        """Check if a vehicle is behind the ego vehicle."""
        ego_loc = self.ego_vehicle.get_location()
        ego_fwd = self.ego_vehicle.get_transform().get_forward_vector()
        vehicle_loc = vehicle.get_location()

        to_vehicle = carla.Vector3D(
            vehicle_loc.x - ego_loc.x,
            vehicle_loc.y - ego_loc.y,
            0.0
        )
        mag = (to_vehicle.x**2 + to_vehicle.y**2) ** 0.5
        if mag < 1.0:
            return False
        to_vehicle.x /= mag
        to_vehicle.y /= mag

        # Negative dot product means vehicle is behind
        dot = ego_fwd.x * to_vehicle.x + ego_fwd.y * to_vehicle.y
        return dot < -0.3

    def _is_approaching_junction(self, waypoint: carla.Waypoint, distance: float = 20.0) -> bool:
        """Check if ego is approaching a junction within specified distance."""
        if waypoint is None:
            return False
        current = waypoint
        traveled = 0.0
        step = 5.0
        while traveled < distance:
            next_wps = current.next(step)
            if not next_wps:
                return False
            current = next_wps[0]
            traveled += step
            if current.is_junction:
                return True
        return False
