"""Trajectory plan schema helpers (plan.json / events_plan.json)."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any, Dict, List, Optional


PLAN_VERSION = "0.1"


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    a: float
    lane_id: Optional[int] = None
    s: Optional[float] = None
    tag: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        payload = {
            "t": round(self.t, 4),
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 3),
            "v": round(self.v, 3),
            "a": round(self.a, 3),
        }
        if self.lane_id is not None:
            payload["lane_id"] = int(self.lane_id)
        if self.s is not None:
            payload["s"] = round(self.s, 3)
        if self.tag:
            payload["tag"] = self.tag
        return payload

    @staticmethod
    def from_dict(raw: Dict[str, Any]) -> "TrajectoryPoint":
        return TrajectoryPoint(
            t=float(raw["t"]),
            x=float(raw["x"]),
            y=float(raw["y"]),
            yaw=float(raw["yaw"]),
            v=float(raw.get("v", 0.0)),
            a=float(raw.get("a", 0.0)),
            lane_id=raw.get("lane_id"),
            s=raw.get("s"),
            tag=raw.get("tag"),
        )


@dataclass
class ActorPlan:
    actor_id: str
    kind: str
    role: str
    blueprint: str
    controller: str = "teleport"
    trajectory: List[TrajectoryPoint] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.actor_id,
            "kind": self.kind,
            "role": self.role,
            "blueprint": self.blueprint,
            "controller": self.controller,
            "trajectory": [point.to_dict() for point in self.trajectory],
        }

    @staticmethod
    def from_dict(raw: Dict[str, Any]) -> "ActorPlan":
        return ActorPlan(
            actor_id=str(raw["id"]),
            kind=str(raw.get("kind", "vehicle")),
            role=str(raw.get("role", "")),
            blueprint=str(raw.get("blueprint", "")),
            controller=str(raw.get("controller", "teleport")),
            trajectory=[TrajectoryPoint.from_dict(p) for p in raw.get("trajectory", [])],
        )


@dataclass
class EventPlan:
    t_event: float
    event_type: str
    expected_action: str
    audio_id: Optional[str] = None
    t_voice_start: Optional[float] = None
    t_robot_precue: Optional[float] = None
    state_targets: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        payload = {
            "t_event": round(self.t_event, 3),
            "event_type": self.event_type,
            "expected_action": self.expected_action,
        }
        if self.audio_id:
            payload["audio_id"] = self.audio_id
        if self.t_voice_start is not None:
            payload["t_voice_start"] = round(self.t_voice_start, 3)
        if self.t_robot_precue is not None:
            payload["t_robot_precue"] = round(self.t_robot_precue, 3)
        if self.state_targets:
            payload["state_targets"] = self.state_targets
        return payload

    @staticmethod
    def from_dict(raw: Dict[str, Any]) -> "EventPlan":
        return EventPlan(
            t_event=float(raw["t_event"]),
            event_type=str(raw.get("event_type", "")),
            expected_action=str(raw.get("expected_action", "")),
            audio_id=raw.get("audio_id"),
            t_voice_start=raw.get("t_voice_start"),
            t_robot_precue=raw.get("t_robot_precue"),
            state_targets=raw.get("state_targets"),
        )


@dataclass
class Plan:
    episode_id: str
    town: str
    seed: int
    dt: float
    duration: float
    actors: List[ActorPlan] = field(default_factory=list)
    events_plan: List[EventPlan] = field(default_factory=list)
    version: str = PLAN_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "episode_id": self.episode_id,
            "town": self.town,
            "seed": int(self.seed),
            "dt": float(self.dt),
            "duration": float(self.duration),
            "actors": [actor.to_dict() for actor in self.actors],
            "events_plan": [event.to_dict() for event in self.events_plan],
        }

    @staticmethod
    def from_dict(raw: Dict[str, Any]) -> "Plan":
        return Plan(
            episode_id=str(raw.get("episode_id", "unknown")),
            town=str(raw.get("town", "")),
            seed=int(raw.get("seed", 0)),
            dt=float(raw.get("dt", 0.05)),
            duration=float(raw.get("duration", 0.0)),
            actors=[ActorPlan.from_dict(a) for a in raw.get("actors", [])],
            events_plan=[EventPlan.from_dict(e) for e in raw.get("events_plan", [])],
            version=str(raw.get("version", PLAN_VERSION)),
        )


def save_plan(path: Path, plan: Plan) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(plan.to_dict(), f, indent=2, ensure_ascii=True)


def load_plan(path: Path) -> Plan:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"Invalid plan file: {path}")
    return Plan.from_dict(raw)


def save_events_plan(path: Path, events: List[EventPlan]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump([event.to_dict() for event in events], f, indent=2, ensure_ascii=True)


def load_events_plan(path: Path) -> List[EventPlan]:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, list):
        raise ValueError(f"Invalid events plan file: {path}")
    return [EventPlan.from_dict(item) for item in raw]
