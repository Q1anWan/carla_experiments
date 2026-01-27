"""Automated tests for interactive editor functionality."""

from __future__ import annotations

import json
import logging
import math
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


def _create_mock_map_data(map_dir: Path) -> None:
    """Create minimal mock map data for testing."""
    map_dir.mkdir(parents=True, exist_ok=True)

    lane_centerlines = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"lane_id": 1},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0]],
                },
            },
            {
                "type": "Feature",
                "properties": {"lane_id": 2},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4]],
                },
            },
        ],
    }

    junction_areas = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"junction_id": 1},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[[45, -5], [55, -5], [55, 10], [45, 10], [45, -5]]],
                },
            },
        ],
    }

    map_graph = {
        "lanes": [
            {"lane_id": 1, "centerline": [[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0]]},
            {"lane_id": 2, "centerline": [[0, 4], [10, 4], [20, 4], [30, 4], [40, 4], [50, 4]]},
        ]
    }

    (map_dir / "lane_centerlines.geojson").write_text(json.dumps(lane_centerlines))
    (map_dir / "junction_areas.geojson").write_text(json.dumps(junction_areas))
    (map_dir / "map_graph.json").write_text(json.dumps(map_graph))


class TestResult:
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.error: Optional[str] = None

    def __repr__(self) -> str:
        status = "PASS" if self.passed else f"FAIL: {self.error}"
        return f"{self.name}: {status}"


def test_map_index_snap() -> TestResult:
    """Test MapIndex snap_to_centerline functionality."""
    result = TestResult("MapIndex.snap_to_centerline")
    try:
        from .interactive_editor import MapIndex

        points = [
            (0.0, 0.0, 1), (5.0, 0.0, 1), (10.0, 0.0, 1), (15.0, 0.0, 1), (20.0, 0.0, 1),
            (0.0, 4.0, 2), (5.0, 4.0, 2), (10.0, 4.0, 2), (15.0, 4.0, 2), (20.0, 4.0, 2),
        ]
        index = MapIndex(points)

        snap_x, snap_y, dist = index.snap_to_centerline(5.5, 1.0)
        if not (4.5 < snap_x < 5.5 and -0.5 < snap_y < 0.5):
            result.error = f"Expected snap near (5,0), got ({snap_x}, {snap_y})"
            return result

        snap_x2, snap_y2, dist2 = index.snap_to_centerline(5.5, 3.0)
        if not (4.5 < snap_x2 < 5.5 and 3.5 < snap_y2 < 4.5):
            result.error = f"Expected snap near (5,4), got ({snap_x2}, {snap_y2})"
            return result

        _, _, dist3 = index.snap_to_centerline(10.0, 0.0)
        if dist3 > 0.01:
            result.error = f"Expected zero distance for exact match, got {dist3}"
            return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_keyframe_operations() -> TestResult:
    """Test Keyframe dataclass and operations."""
    result = TestResult("Keyframe operations")
    try:
        from .interactive_editor import Keyframe, _interpolate_position, _prepare_keyframes

        kf1 = Keyframe(t=0.0, x=0.0, y=0.0, v=10.0)
        kf2 = Keyframe(t=10.0, x=100.0, y=0.0, v=10.0)
        keyframes = [kf1, kf2]

        x, y = _interpolate_position(keyframes, 5.0)
        if not (45 < x < 55 and -1 < y < 1):
            result.error = f"Interpolation at t=5 expected ~(50,0), got ({x}, {y})"
            return result

        x_start, y_start = _interpolate_position(keyframes, 0.0)
        if not (x_start == 0.0 and y_start == 0.0):
            result.error = f"Start position expected (0,0), got ({x_start}, {y_start})"
            return result

        x_end, y_end = _interpolate_position(keyframes, 10.0)
        if not (x_end == 100.0 and y_end == 0.0):
            result.error = f"End position expected (100,0), got ({x_end}, {y_end})"
            return result

        prepared = _prepare_keyframes([Keyframe(t=5.0, x=50.0, y=0.0)], 10.0)
        if len(prepared) != 3:
            result.error = f"Expected 3 keyframes after prepare, got {len(prepared)}"
            return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_trajectory_building() -> TestResult:
    """Test trajectory building from keyframes."""
    result = TestResult("Trajectory building")
    try:
        from .interactive_editor import Keyframe, MapIndex, _build_trajectory

        keyframes = [
            Keyframe(t=0.0, x=0.0, y=0.0, v=10.0),
            Keyframe(t=5.0, x=50.0, y=0.0, v=10.0),
        ]

        points = [(float(i), 0.0, 1) for i in range(0, 60, 5)]
        map_index = MapIndex(points)

        trajectory = _build_trajectory(keyframes, dt=0.1, duration=5.0, map_index=map_index)

        if len(trajectory) != 50:
            result.error = f"Expected 50 trajectory points, got {len(trajectory)}"
            return result

        if trajectory[0].x != 0.0:
            result.error = f"First point x expected 0, got {trajectory[0].x}"
            return result

        mid_point = trajectory[25]
        if not (24 < mid_point.x < 26):
            result.error = f"Mid point x expected ~25, got {mid_point.x}"
            return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_scene_serialization() -> TestResult:
    """Test scene save/load round-trip."""
    result = TestResult("Scene serialization")
    try:
        from .interactive_editor import (
            ActorState,
            EventMarker,
            Keyframe,
            SceneEdit,
            _load_scene,
            _save_scene,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            scene_path = Path(tmpdir) / "test_scene.json"

            original = SceneEdit(
                version="0.1",
                episode_id="test_episode",
                town="TestTown",
                dt=0.05,
                duration=30.0,
                map_dir="/tmp/maps",
                seed=42,
                actors=[
                    ActorState(
                        actor_id="ego",
                        kind="vehicle",
                        role="ego",
                        blueprint="vehicle.tesla.model3",
                        controller="teleport",
                        keyframes=[
                            Keyframe(t=0.0, x=0.0, y=0.0, v=10.0),
                            Keyframe(t=30.0, x=300.0, y=0.0, v=10.0),
                        ],
                    ),
                    ActorState(
                        actor_id="npc1",
                        kind="vehicle",
                        role="npc",
                        blueprint="vehicle.audi.a2",
                        controller="teleport",
                        keyframes=[
                            Keyframe(t=0.0, x=20.0, y=4.0, v=8.0),
                            Keyframe(t=30.0, x=260.0, y=4.0, v=8.0),
                        ],
                    ),
                ],
                events=[
                    EventMarker(
                        t_event=15.0,
                        event_type="lane_change",
                        expected_action="lane_change",
                    ),
                ],
            )

            _save_scene(original, scene_path)

            if not scene_path.exists():
                result.error = "Scene file was not created"
                return result

            loaded = _load_scene(scene_path)

            if loaded.episode_id != original.episode_id:
                result.error = f"Episode ID mismatch: {loaded.episode_id} != {original.episode_id}"
                return result

            if len(loaded.actors) != len(original.actors):
                result.error = f"Actor count mismatch: {len(loaded.actors)} != {len(original.actors)}"
                return result

            if len(loaded.events) != len(original.events):
                result.error = f"Event count mismatch: {len(loaded.events)} != {len(original.events)}"
                return result

            if loaded.actors[0].keyframes[0].v != 10.0:
                result.error = f"Keyframe velocity mismatch: {loaded.actors[0].keyframes[0].v}"
                return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_plan_export() -> TestResult:
    """Test plan.json export from scene."""
    result = TestResult("Plan export")
    try:
        from .interactive_editor import (
            ActorState,
            EventMarker,
            Keyframe,
            MapIndex,
            SceneEdit,
            _build_plan,
        )
        from ..planning.trajectory_schema import save_plan

        scene = SceneEdit(
            version="0.1",
            episode_id="export_test",
            town="TestTown",
            dt=0.1,
            duration=10.0,
            map_dir="/tmp/maps",
            seed=0,
            actors=[
                ActorState(
                    actor_id="ego",
                    kind="vehicle",
                    role="ego",
                    blueprint="vehicle.tesla.model3",
                    controller="teleport",
                    keyframes=[
                        Keyframe(t=0.0, x=0.0, y=0.0),
                        Keyframe(t=10.0, x=100.0, y=0.0),
                    ],
                ),
            ],
            events=[
                EventMarker(t_event=5.0, event_type="test_event", expected_action="test_action"),
            ],
        )

        points = [(float(i * 10), 0.0, 1) for i in range(11)]
        map_index = MapIndex(points)

        plan, events_plan = _build_plan(
            scene,
            map_index=map_index,
            voice_lead_time=3.0,
            robot_precue_lead=0.5,
        )

        if plan.episode_id != "export_test":
            result.error = f"Plan episode_id mismatch: {plan.episode_id}"
            return result

        if len(plan.actors) != 1:
            result.error = f"Expected 1 actor in plan, got {len(plan.actors)}"
            return result

        ego = plan.actors[0]
        if len(ego.trajectory) != 100:
            result.error = f"Expected 100 trajectory points, got {len(ego.trajectory)}"
            return result

        if len(events_plan) != 1:
            result.error = f"Expected 1 event, got {len(events_plan)}"
            return result

        if events_plan[0].t_event != 5.0:
            result.error = f"Event time mismatch: {events_plan[0].t_event}"
            return result

        with tempfile.TemporaryDirectory() as tmpdir:
            plan_path = Path(tmpdir) / "plan.json"
            save_plan(plan_path, plan)
            if not plan_path.exists():
                result.error = "plan.json was not created"
                return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_undo_redo_logic() -> TestResult:
    """Test undo/redo state management."""
    result = TestResult("Undo/Redo logic")
    try:
        from .interactive_editor import ActorState, EventMarker, Keyframe, SceneEdit

        states: List[str] = []
        max_undo = 5

        def push_state(scene: SceneEdit) -> None:
            state = json.dumps({
                "actors": [
                    {"id": a.actor_id, "keyframes": [{"t": k.t, "x": k.x, "y": k.y} for k in a.keyframes]}
                    for a in scene.actors
                ]
            })
            states.append(state)
            if len(states) > max_undo:
                states.pop(0)

        scene = SceneEdit(
            version="0.1",
            episode_id="undo_test",
            town="Test",
            dt=0.1,
            duration=10.0,
            map_dir="",
            actors=[
                ActorState(
                    actor_id="ego",
                    kind="vehicle",
                    role="ego",
                    blueprint="",
                    controller="teleport",
                    keyframes=[Keyframe(t=0.0, x=0.0, y=0.0)],
                )
            ],
            events=[],
        )

        push_state(scene)
        scene.actors[0].keyframes.append(Keyframe(t=5.0, x=50.0, y=0.0))
        push_state(scene)
        scene.actors[0].keyframes.append(Keyframe(t=10.0, x=100.0, y=0.0))
        push_state(scene)

        if len(states) != 3:
            result.error = f"Expected 3 states, got {len(states)}"
            return result

        state_before = json.loads(states[-2])
        if len(state_before["actors"][0]["keyframes"]) != 2:
            result.error = "State before last should have 2 keyframes"
            return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_headless_export() -> TestResult:
    """Test headless mode export functionality."""
    result = TestResult("Headless export")
    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            map_dir = Path(tmpdir) / "maps" / "TestTown"
            out_dir = Path(tmpdir) / "outputs"
            _create_mock_map_data(map_dir)

            from .interactive_editor import main as editor_main

            exit_code = editor_main([
                "--map-dir", str(map_dir),
                "--episode-id", "headless_test",
                "--out", str(out_dir),
                "--duration", "5.0",
                "--headless",
            ])

            if exit_code != 0:
                result.error = f"Editor returned non-zero exit code: {exit_code}"
                return result

            plan_path = out_dir / "headless_test" / "plan.json"
            if not plan_path.exists():
                result.error = "plan.json was not created"
                return result

            scene_path = out_dir / "headless_test" / "scene_edit.json"
            if not scene_path.exists():
                result.error = "scene_edit.json was not created"
                return result

            plan_data = json.loads(plan_path.read_text())
            if plan_data.get("episode_id") != "headless_test":
                result.error = f"Plan episode_id mismatch: {plan_data.get('episode_id')}"
                return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_kinematic_analysis() -> TestResult:
    """Test kinematic constraint checking in analysis."""
    result = TestResult("Kinematic analysis")
    try:
        from .interactive_editor import Keyframe, MapIndex, _build_trajectory

        keyframes = [
            Keyframe(t=0.0, x=0.0, y=0.0, v=50.0),
            Keyframe(t=1.0, x=50.0, y=0.0, v=50.0),
        ]

        points = [(float(i), 0.0, 1) for i in range(60)]
        map_index = MapIndex(points)

        trajectory = _build_trajectory(keyframes, dt=0.1, duration=1.0, map_index=map_index)

        max_speed_mps = 30.0
        violations = []
        for pt in trajectory:
            if abs(pt.v) > max_speed_mps:
                violations.append(pt)

        if len(violations) == 0:
            result.error = "Expected kinematic violations for v=50 m/s > 30 m/s limit"
            return result

        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def run_all_tests() -> Tuple[int, int, List[TestResult]]:
    """Run all tests and return (passed, total, results)."""
    tests = [
        test_map_index_snap,
        test_keyframe_operations,
        test_trajectory_building,
        test_scene_serialization,
        test_plan_export,
        test_undo_redo_logic,
        test_headless_export,
        test_kinematic_analysis,
    ]

    results = []
    passed = 0
    for test_func in tests:
        logging.info("Running %s...", test_func.__name__)
        result = test_func()
        results.append(result)
        if result.passed:
            passed += 1
            logging.info("  PASS")
        else:
            logging.error("  FAIL: %s", result.error)

    return passed, len(tests), results


def main() -> int:
    """Run automated tests."""
    print("=" * 60)
    print("Interactive Editor Automated Test Suite")
    print("=" * 60)

    passed, total, results = run_all_tests()

    print("\n" + "=" * 60)
    print(f"Results: {passed}/{total} tests passed")
    print("=" * 60)

    for r in results:
        status = "✓ PASS" if r.passed else "✗ FAIL"
        print(f"  {status}: {r.name}")
        if not r.passed and r.error:
            print(f"         Error: {r.error}")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
