"""Scenario Director — condition-based event orchestration for V3.

The Director evaluates trigger conditions each tick, executes TM parameter
changes or short override takeovers, and tracks event state transitions.
"""

from __future__ import annotations

import logging
import operator
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ..driver_backends.short_override import ShortOverrideBackend
from ..driver_backends.tm_backend import TMBackend
from .event_dsl import EventSpec, OverrideFallback, TMAction, TriggerSpec
from .metrics import ActorMetrics, compute_metrics

try:
    import carla
except ImportError:
    carla = None


# Comparison operators for trigger evaluation
_OPS = {
    "<=": operator.le,
    ">=": operator.ge,
    "<": operator.lt,
    ">": operator.gt,
    "==": lambda a, b: abs(a - b) < 1e-6,
}


@dataclass
class EventState:
    """Runtime state for a single event."""

    spec: EventSpec
    status: str = "pending"  # pending | triggered | override_active | post | done
    triggered_frame: Optional[int] = None
    metrics_at_trigger: Optional[ActorMetrics] = None
    override_backend: Optional[ShortOverrideBackend] = None
    metrics_history: List[ActorMetrics] = field(default_factory=list)


class ScenarioDirector:
    """Central V3 runtime orchestrator.

    Added as a tick callback to ScenarioContext. Each tick:
    1. Computes metrics for each pending event's actor pair
    2. Evaluates trigger conditions (AND logic)
    3. On trigger: applies TM params, executes force_lane_change, or
       activates short override controller
    4. Monitors override progress, applies post-event TM params
    5. Logs all state transitions for debugging

    Usage:
        director = ScenarioDirector(world, tm, map_obj, events, actor_map, ego, fps)
        ctx.tick_callbacks.append(director.tick)
    """

    def __init__(
        self,
        world: Any,
        tm: Any,
        map_obj: Any,
        events: List[EventSpec],
        actor_map: Dict[str, Any],
        ego: Any,
        fps: int,
    ) -> None:
        self._world = world
        self._tm = tm
        self._map_obj = map_obj
        self._actor_map = actor_map
        self._ego = ego
        self._fps = fps

        self._tm_backend = TMBackend(tm)
        self._event_states = [EventState(spec=e) for e in events]

        logging.info(
            "ScenarioDirector initialized with %d events: %s",
            len(events),
            [e.id for e in events],
        )

    def tick(self, frame_index: int) -> None:
        """Main director tick — evaluate triggers and manage events."""
        dt = 1.0 / max(1, self._fps)

        for es in self._event_states:
            actor = self._actor_map.get(es.spec.actor)
            if actor is None:
                continue
            if not actor.is_alive or not self._ego.is_alive:
                continue

            if es.status == "pending":
                self._tick_pending(es, actor, frame_index)

            elif es.status == "triggered":
                self._tick_triggered(es, actor, frame_index)

            elif es.status == "override_active":
                self._tick_override(es, actor, frame_index, dt)

            elif es.status == "post":
                es.status = "done"
                logging.info(
                    "Event %s completed at frame %d",
                    es.spec.id,
                    frame_index,
                )

    def _tick_pending(
        self, es: EventState, actor: Any, frame_index: int
    ) -> None:
        """Evaluate triggers for a pending event."""
        metrics = compute_metrics(
            self._ego, actor, self._map_obj, frame_index, self._fps
        )
        es.metrics_history.append(metrics)

        if self._evaluate_trigger(es.spec.trigger, metrics):
            es.status = "triggered"
            es.triggered_frame = frame_index
            es.metrics_at_trigger = metrics
            logging.info(
                "Event %s TRIGGERED at frame %d (t=%.2fs): "
                "gap=%.1fm, ttc=%.1fs, lateral=%.1fm",
                es.spec.id,
                frame_index,
                metrics.elapsed_s,
                metrics.gap_m,
                metrics.ttc_s,
                metrics.lateral_offset_m,
            )
            self._execute_event(es, actor)

    def _tick_triggered(
        self, es: EventState, actor: Any, frame_index: int
    ) -> None:
        """Handle TM-only triggered events (no override)."""
        elapsed = (frame_index - es.triggered_frame) / max(1, self._fps)
        # Wait 1 second for TM action to settle, then apply post params
        if elapsed >= 1.0:
            es.status = "post"
            self._apply_post(es, actor, frame_index)

    def _tick_override(
        self, es: EventState, actor: Any, frame_index: int, dt: float
    ) -> None:
        """Run the short override controller."""
        ctrl = es.override_backend.run_step(actor, frame_index, dt)
        if ctrl is not None:
            actor.apply_control(ctrl)

        if es.override_backend.is_done:
            logging.info(
                "Override completed for event %s at frame %d",
                es.spec.id,
                frame_index,
            )
            es.status = "post"
            self._apply_post(es, actor, frame_index)

    def _evaluate_trigger(
        self, trigger: TriggerSpec, metrics: ActorMetrics
    ) -> bool:
        """Check if all trigger conditions are satisfied."""
        for cond in trigger.all:
            val = getattr(metrics, cond.metric, None)
            if val is None:
                logging.warning("Unknown metric '%s' in trigger", cond.metric)
                return False
            op_fn = _OPS.get(cond.op)
            if op_fn is None:
                logging.warning("Unknown operator '%s' in trigger", cond.op)
                return False
            if not op_fn(val, cond.value):
                return False
        return True

    def _execute_event(self, es: EventState, actor: Any) -> None:
        """Execute event actions: TM params → force_lane_change → override."""
        # 1. Apply pre-event TM parameters
        if es.spec.action_tm:
            self._tm_backend.configure(actor, es.spec.action_tm)
            logging.info("Applied TM params for event %s", es.spec.id)

        # 2. Execute TM action (force_lane_change) — skip if fallback override
        #    will handle the lane change directly via physics control
        if es.spec.action_execute and es.spec.action_execute.force_lane_change:
            if not es.spec.fallback:
                direction = es.spec.action_execute.force_lane_change
                self._tm_backend.force_lane_change(actor, direction)

        # 3. If fallback override is configured, start it
        if es.spec.fallback:
            direction = "left"
            if es.spec.action_execute and es.spec.action_execute.force_lane_change:
                direction = es.spec.action_execute.force_lane_change

            override = ShortOverrideBackend(
                world=self._world,
                map_obj=self._map_obj,
                fps=self._fps,
                window_s=es.spec.fallback.controller_override_window_s,
                profile=es.spec.fallback.lane_change_profile,
                target_speed_mps=es.spec.fallback.target_speed_mps,
                direction=direction,
            )
            override.activate(actor)
            es.override_backend = override
            es.status = "override_active"
            logging.info(
                "Override activated for event %s (window=%.1fs, direction=%s)",
                es.spec.id,
                es.spec.fallback.controller_override_window_s,
                direction,
            )

    def _apply_post(
        self, es: EventState, actor: Any, frame_index: int
    ) -> None:
        """Apply post-event TM parameters and re-enable autopilot."""
        if es.spec.action_post:
            self._tm_backend.configure(actor, es.spec.action_post)
            logging.info(
                "Applied post-event TM params for event %s at frame %d",
                es.spec.id,
                frame_index,
            )
        self._tm_backend.activate(actor)

    # ------------------------------------------------------------------
    # Query methods
    # ------------------------------------------------------------------

    def get_event_states(self) -> List[EventState]:
        """Return all event states for reporting."""
        return list(self._event_states)

    def get_triggered_events(self) -> List[EventState]:
        """Return events that have been triggered."""
        return [
            es for es in self._event_states if es.status in ("triggered", "override_active", "post", "done")
        ]

    def get_event_log(self) -> List[Dict[str, Any]]:
        """Build structured event log for output."""
        log = []
        for es in self._event_states:
            entry = {
                "event_id": es.spec.id,
                "event_type": es.spec.type,
                "actor": es.spec.actor,
                "status": es.status,
            }
            if es.triggered_frame is not None:
                entry["triggered_frame"] = es.triggered_frame
                entry["triggered_t"] = round(
                    es.triggered_frame / max(1, self._fps), 3
                )
            if es.metrics_at_trigger is not None:
                m = es.metrics_at_trigger
                entry["metrics_at_trigger"] = {
                    "gap_m": m.gap_m,
                    "ttc_s": m.ttc_s if m.ttc_s != float("inf") else None,
                    "lateral_offset_m": m.lateral_offset_m,
                    "ego_speed_mps": m.ego_speed_mps,
                    "target_speed_mps": m.target_speed_mps,
                }
            log.append(entry)
        return log
