"""V3 lane change cut-in scenario — Director-driven, condition-based triggering.

Reuses the same actor spawning logic as V1 (scenario.py) but replaces the
frame-based cut_in callback with a ScenarioDirector that evaluates gap/TTC
conditions and executes via TM + short override controller.

Supports multiple cut-in vehicles and repeated events within a single run.
"""

from __future__ import annotations

import logging
import random

import carla

from ...director.director import ScenarioDirector
from ...director.event_dsl import parse_events
from ..base import (
    BaseScenario,
    ScenarioContext,
    find_spawn_point,
    get_spawn_point_by_index,
    log_spawn,
    offset_transform,
    pick_spawn_point,
)


class LaneChangeCutInV3(BaseScenario):
    """V3 cut-in scenario with Director-based event triggering."""

    def build(
        self,
        world: carla.World,
        tm: carla.TrafficManager,
        rng: random.Random,
    ) -> ScenarioContext:
        params = self.config.params
        spawn_points = world.get_map().get_spawn_points()

        # --- Shorten traffic light durations ---
        red_time = float(params.get("traffic_light_red_s", 8.0))
        green_time = float(params.get("traffic_light_green_s", 12.0))
        yellow_time = float(params.get("traffic_light_yellow_s", 2.0))
        self._configure_traffic_lights(world, red_time, green_time, yellow_time)

        # --- Ego spawn ---
        ego_spawn = get_spawn_point_by_index(
            spawn_points, params.get("ego_spawn_index")
        )
        if ego_spawn is None and bool(params.get("fast_spawn")):
            ego_spawn = pick_spawn_point(spawn_points, rng)
        if ego_spawn is None:
            ego_spawn = find_spawn_point(
                world,
                rng,
                min_lanes=3,
                avoid_junction=True,
                forward_clear_m=150.0,
                avoid_traffic_lights=True,
            )

        ego = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter=self.config.ego_vehicle,
            transform=ego_spawn,
            role_name="ego",
            autopilot=True,
        )
        log_spawn(ego, "ego", ego_spawn)
        self._apply_ego_tm(tm, ego)
        self._configure_tm_global(tm)

        ego_wp = world.get_map().get_waypoint(ego_spawn.location)

        # --- Find adjacent lane ---
        adjacent_wp = None
        cut_in_direction = "left"  # direction cut-in vehicle will change to
        right_wp = ego_wp.get_right_lane()
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            adjacent_wp = right_wp
            cut_in_direction = "left"
        else:
            left_wp = ego_wp.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                adjacent_wp = left_wp
                cut_in_direction = "right"

        # --- Spawn multiple cut-in vehicles ---
        cut_in_configs = params.get("cut_in_vehicles", [])
        if not cut_in_configs:
            # Fallback to single cut-in vehicle (backwards compat)
            cut_in_configs = [
                {
                    "tag": "cut_in_vehicle",
                    "ahead_m": float(params.get("cut_in_ahead_m", 12.0)),
                    "speed_delta": params.get("cut_in_speed_delta"),
                }
            ]

        actor_map = {"ego": ego}
        all_scenario_actors = []  # non-ego scenario actors
        exclude_locations = [ego_spawn.location]

        for ci_cfg in cut_in_configs:
            tag = ci_cfg["tag"]
            ahead_m = float(ci_cfg.get("ahead_m", 12.0))
            speed_delta = ci_cfg.get("speed_delta")

            if adjacent_wp:
                ahead_wps = adjacent_wp.next(ahead_m)
                if ahead_wps:
                    ci_spawn = ahead_wps[0].transform
                    ci_spawn.location.z += 0.3
                else:
                    ci_spawn = adjacent_wp.transform
                    ci_spawn.location.z += 0.3
            else:
                ci_spawn = offset_transform(
                    ego_spawn, forward=ahead_m, right=3.5
                )

            vehicle = self._spawn_vehicle(
                world,
                tm,
                rng,
                blueprint_filter="vehicle.*",
                transform=ci_spawn,
                role_name=tag,
                autopilot=True,
            )
            log_spawn(vehicle, tag, ci_spawn)
            self._configure_vehicle_tm(
                tm,
                vehicle,
                speed_delta=float(speed_delta) if speed_delta is not None else None,
                auto_lane_change=False,
                ignore_lights=0.0,
                ignore_vehicles=0.0,
            )
            actor_map[tag] = vehicle
            all_scenario_actors.append(vehicle)
            exclude_locations.append(ci_spawn.location)

        # --- Lead slow vehicle ---
        lead_distance = float(params.get("lead_slow_distance_m", 25.0))
        lead_speed_delta = float(params.get("lead_slow_speed_delta", 35.0))
        lead_spawn = offset_transform(ego_spawn, forward=lead_distance)
        lead_vehicle = self._spawn_vehicle(
            world,
            tm,
            rng,
            blueprint_filter="vehicle.*",
            transform=lead_spawn,
            role_name="lead_slow",
            autopilot=True,
        )
        tm.vehicle_percentage_speed_difference(lead_vehicle, lead_speed_delta)
        self._configure_vehicle_tm(
            tm, lead_vehicle,
            ignore_lights=0.0,
            ignore_vehicles=0.0,
        )
        log_spawn(lead_vehicle, "lead_slow", lead_spawn)
        actor_map["lead_slow"] = lead_vehicle
        all_scenario_actors.append(lead_vehicle)
        exclude_locations.append(lead_spawn.location)

        # --- Background traffic ---
        background_vehicle_count = int(params.get("background_vehicle_count", 20))
        background_walker_count = int(params.get("background_walker_count", 12))
        background_min_distance = float(params.get("background_min_distance_m", 20.0))
        background = self._spawn_background_traffic(
            world,
            tm,
            rng,
            vehicle_count=background_vehicle_count,
            walker_count=background_walker_count,
            exclude_locations=exclude_locations,
            min_distance=background_min_distance,
        )

        # Ensure all background vehicles respect traffic lights
        for actor in background:
            if hasattr(actor, 'set_autopilot'):
                self._configure_vehicle_tm(
                    tm, actor,
                    ignore_lights=0.0,
                    ignore_vehicles=0.0,
                )

        # --- Build scenario context ---
        ctx = ScenarioContext(
            world=world,
            ego_vehicle=ego,
            actors=[ego] + all_scenario_actors + background,
            camera_config=self.config.camera,
            fps=self.config.fps,
            duration=self.config.duration,
            fixed_delta_seconds=self.config.fixed_delta_seconds,
            seed=self.config.seed,
            scenario_id=self.config.scenario_id,
        )

        # Tag actors for Director lookup
        for tag, actor in actor_map.items():
            ctx.tag_actor(tag, actor)

        # --- V3: Director setup ---
        events_raw = params.get("events", [])
        if not events_raw:
            logging.warning("No events defined in V3 config, Director will be idle")

        event_specs = parse_events(events_raw)

        director = ScenarioDirector(
            world=world,
            tm=tm,
            map_obj=world.get_map(),
            events=event_specs,
            actor_map=actor_map,
            ego=ego,
            fps=self.config.fps,
        )

        # Store director reference for post-run gate checks
        ctx.tags["__director__"] = [director]

        # Add director.tick as the primary tick callback
        ctx.tick_callbacks.append(director.tick)

        # Ego brake (optional, same as V1)
        self._maybe_add_ego_brake(ctx, tm)

        logging.info(
            "V3 LaneChangeCutIn built: %d events, %d actors, direction=%s",
            len(event_specs),
            len(ctx.actors),
            cut_in_direction,
        )
        return ctx

    @staticmethod
    def _configure_traffic_lights(
        world: carla.World,
        red_s: float,
        green_s: float,
        yellow_s: float,
    ) -> None:
        """Set traffic light timings. Respects group coordination."""
        actors = world.get_actors().filter("traffic.traffic_light")
        count = 0
        reset_groups = set()
        for tl in actors:
            tl.set_red_time(red_s)
            tl.set_green_time(green_s)
            tl.set_yellow_time(yellow_s)
            count += 1
            # Reset each group once to apply new timings immediately
            group_id = id(tuple(sorted(g.id for g in tl.get_group_traffic_lights())))
            if group_id not in reset_groups:
                tl.reset_group()
                reset_groups.add(group_id)
        logging.info(
            "Configured %d traffic lights (%d groups): red=%.1fs green=%.1fs yellow=%.1fs",
            count, len(reset_groups), red_s, green_s, yellow_s,
        )
