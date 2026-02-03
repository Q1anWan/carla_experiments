"""Unified CLI with optional interactive menu for planning, rendering, and editing."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Optional, Sequence

import yaml

from .editor import interactive_editor
from .editor import mvp as editor_mvp


def _load_globals(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _prompt(text: str, default: Optional[str] = None) -> str:
    label = f"{text} [{default}]: " if default is not None else f"{text}: "
    value = input(label).strip()
    return value if value else (default or "")


def _prompt_float(text: str, default: float) -> float:
    raw = _prompt(text, f"{default}")
    try:
        return float(raw)
    except ValueError:
        return default


def _prompt_int(text: str, default: int) -> int:
    raw = _prompt(text, f"{default}")
    try:
        return int(raw)
    except ValueError:
        return default


def _render_image_size(args: argparse.Namespace) -> Optional[tuple[int, int]]:
    if args.quick:
        return (640, 360)
    if args.image_size:
        return (int(args.image_size[0]), int(args.image_size[1]))
    return None


def do_map(args: argparse.Namespace, globals_cfg: Dict[str, Any]) -> None:
    from .planning.map_exporter import MapExportConfig, export_map

    carla_cfg = globals_cfg.get("carla", {}) if isinstance(globals_cfg, dict) else {}
    host = args.host or carla_cfg.get("host", "127.0.0.1")
    port = int(args.port or carla_cfg.get("port", 2000))
    timeout = float(args.timeout or carla_cfg.get("timeout_s", 10.0))
    config = MapExportConfig(
        host=str(host),
        port=port,
        timeout=timeout,
        map_name=str(args.map),
        waypoint_distance=float(args.waypoint_distance),
    )
    out_dir = Path(args.out) / config.map_name
    export_map(config, out_dir, max_seconds=args.max_seconds)


def do_plan(args: argparse.Namespace) -> None:
    from .planning.planner_compiler import compile_episode

    out_dir = Path(args.out) / args.episode
    compile_episode(
        args.episode,
        globals_path=Path(args.globals),
        episodes_dir=Path(args.episodes_dir),
        out_dir=out_dir,
        max_seconds=args.max_seconds,
    )


def do_validate(args: argparse.Namespace) -> None:
    from .planning.validator import validate_plan

    plan_path = Path(args.plan) if args.plan else Path(args.out) / args.episode / "plan.json"
    out_dir = Path(args.out) / args.episode
    validate_plan(
        plan_path,
        Path(args.globals),
        out_dir,
        max_seconds=args.max_seconds,
    )


def do_render(args: argparse.Namespace) -> None:
    from .render.renderer import render_plan

    plan_path = Path(args.plan) if args.plan else Path(args.out) / args.episode / "plan.json"
    out_dir = Path(args.out) / args.episode
    render_plan(
        plan_path,
        Path(args.globals),
        out_dir,
        max_seconds=args.max_seconds,
        max_tick_seconds=args.max_tick_seconds,
        image_size=_render_image_size(args),
        encode_timeout_s=args.encode_timeout,
    )


# DocRef: technical_details.md#3
def do_pipeline(args: argparse.Namespace) -> None:
    from .planning.validator import validate_plan
    from .render.renderer import render_plan

    do_plan(args)
    report = validate_plan(
        Path(args.out) / args.episode / "plan.json",
        Path(args.globals),
        Path(args.out) / args.episode,
        max_seconds=args.max_validate_seconds,
    )
    if report.get("status") != "pass":
        logging.warning("Validation failed for %s, skipping render.", args.episode)
        return
    render_plan(
        Path(args.out) / args.episode / "plan.json",
        Path(args.globals),
        Path(args.out) / args.episode,
        max_seconds=args.max_render_seconds,
        max_tick_seconds=args.max_tick_seconds,
        image_size=_render_image_size(args),
        encode_timeout_s=args.encode_timeout,
    )


def do_preview(args: argparse.Namespace) -> None:
    if args.plan:
        plan_path = Path(args.plan)
    elif args.episode:
        plan_path = Path(args.out_root) / args.episode / "plan.json"
    else:
        raise SystemExit("preview requires --plan or --episode")
    if args.out:
        out_path = Path(args.out)
    elif args.episode:
        out_path = Path(args.out_root) / args.episode / "plan_preview.png"
    else:
        raise SystemExit("preview requires --out or --episode")
    editor_mvp.plot_map_preview(Path(args.map_dir), plan_path, out_path)


def do_editor(args: argparse.Namespace) -> None:
    argv = [
        "--map-dir",
        args.map_dir,
        "--episode-id",
        args.episode_id,
        "--out",
        args.out,
        "--globals",
        args.globals,
        "--dt",
        str(args.dt),
        "--duration",
        str(args.duration),
    ]
    if getattr(args, "headless", False):
        argv.append("--headless")
    if args.scene:
        argv.extend(["--scene", args.scene])
    if args.town:
        argv.extend(["--town", args.town])
    interactive_editor.main(argv)


def do_suite(args: argparse.Namespace) -> None:
    episodes = []
    suites_dir = Path(args.suites_dir)
    if args.all:
        suite_ids = ["P1", "P2", "P3", "P4", "P5", "P6"]
    else:
        suite_ids = [args.suite]
    for suite_id in suite_ids:
        suite_path = suites_dir / f"{suite_id}.yaml"
        if not suite_path.exists():
            logging.warning("Suite not found: %s", suite_path)
            continue
        raw = yaml.safe_load(suite_path.read_text()) or {}
        for item in raw.get("episodes") or []:
            episodes.append(str(item))
    image_size = _render_image_size(args)
    for episode_id in episodes:
        logging.info("Processing episode %s", episode_id)
        run_args = SimpleNamespace(
            episode=episode_id,
            globals=args.globals,
            episodes_dir=args.episodes_dir,
            out=args.out,
            max_seconds=args.max_plan_seconds,
            max_validate_seconds=args.max_validate_seconds,
            max_render_seconds=args.max_render_seconds,
            max_tick_seconds=args.max_tick_seconds,
            encode_timeout=args.encode_timeout,
            image_size=image_size,
            quick=args.quick,
        )
        do_pipeline(run_args)


def do_compare(args: argparse.Namespace) -> None:
    from .tools.make_comparison import generate_comparison
    generate_comparison(
        scene_path=Path(args.scene),
        csv_path=Path(args.telemetry_csv),
        output_dir=Path(args.output),
        events_path=Path(args.events) if args.events else None,
        actors_filter=args.actors.split(",") if args.actors else None,
    )


def do_convert(args: argparse.Namespace) -> None:
    from .tools.convert_telemetry_to_scene import convert_telemetry
    convert_telemetry(
        csv_path=Path(args.csv),
        json_path=Path(args.json),
        events_path=Path(args.events),
        output_path=Path(args.output),
        town=args.town,
        duration=args.duration,
        dt=args.dt,
    )


def do_bundle(args: argparse.Namespace) -> None:
    """Collect all episode artifacts into a standardised bundle directory.

    Bundle layout:
        <bundle_dir>/
            scene_edit.json
            plan.json
            telemetry.csv
            telemetry.json
            events.json
            comparison.png
            compare_report.json
            master_video.mp4
            run_metadata.json
    """
    import shutil
    from .tools.make_comparison import generate_comparison

    episode_dir = Path(args.out) / args.episode
    bundle_dir = Path(args.bundle_dir) if args.bundle_dir else episode_dir / "bundle"
    bundle_dir.mkdir(parents=True, exist_ok=True)

    # Locate scene_edit.json
    scene_path = None
    if args.scene:
        scene_path = Path(args.scene)
    else:
        candidate = Path(args.scene_designs) / args.episode / "scene_edit.json"
        if candidate.exists():
            scene_path = candidate

    # Locate render outputs (prefer render/ subdir, fallback to episode root)
    render_dir = episode_dir / "render"
    if not render_dir.exists():
        render_dir = episode_dir

    # Mapping: (source_path, bundle_filename)
    artifacts = []

    if scene_path and scene_path.exists():
        artifacts.append((scene_path, "scene_edit.json"))

    for name in [
        "plan.json",
        "telemetry.csv",
        "telemetry.json",
        "events.json",
        "master_video.mp4",
        "run_metadata.json",
    ]:
        # Try episode root first (latest render), then render subdir (older)
        for src_dir in [episode_dir, render_dir]:
            p = src_dir / name
            if p.exists():
                artifacts.append((p, name))
                break

    # Also grab episode/plan.json if render didn't have it
    ep_plan = episode_dir / "episode" / "plan.json"
    if ep_plan.exists() and not any(n == "plan.json" for _, n in artifacts):
        artifacts.append((ep_plan, "plan.json"))

    # Copy artifacts
    copied = []
    for src, dst_name in artifacts:
        dst = bundle_dir / dst_name
        shutil.copy2(src, dst)
        copied.append(dst_name)
        logging.info("Bundled %s", dst_name)

    # Run comparison if scene + telemetry exist
    csv_in_bundle = bundle_dir / "telemetry.csv"
    scene_in_bundle = bundle_dir / "scene_edit.json"
    events_in_bundle = bundle_dir / "events.json"
    if csv_in_bundle.exists() and scene_in_bundle.exists():
        logging.info("Running comparison...")
        generate_comparison(
            scene_path=scene_in_bundle,
            csv_path=csv_in_bundle,
            output_dir=bundle_dir,
            events_path=events_in_bundle if events_in_bundle.exists() else None,
        )
        copied.extend(["comparison.png", "compare_report.json"])

    print(f"Bundle created at {bundle_dir} ({len(copied)} files)")
    for name in sorted(set(copied)):
        print(f"  {name}")


def do_run_v3(args: argparse.Namespace, globals_cfg: Dict[str, Any]) -> None:
    """Run a V3 Director-driven scenario."""
    from .config import apply_client_overrides, load_client_config
    from .run_scenario import resolve_scenario_path, run_scenario_v3
    from .utils import utc_timestamp

    carla_cfg = globals_cfg.get("carla", {}) if isinstance(globals_cfg, dict) else {}
    client_config = load_client_config(Path(args.client_config))
    client_config = apply_client_overrides(
        client_config,
        host=args.host or carla_cfg.get("host"),
        port=args.port or carla_cfg.get("port"),
        tm_port=getattr(args, "tm_port", None) or carla_cfg.get("tm_port"),
        timeout=args.timeout or carla_cfg.get("timeout_s"),
        allow_version_mismatch=args.allow_version_mismatch,
    )

    out_dir = Path(args.out) if args.out else Path("runs") / utc_timestamp()
    scenario_path = resolve_scenario_path(args.scenario)

    run_scenario_v3(
        scenario_path,
        out_dir,
        host=client_config.host,
        port=client_config.port,
        tm_port=client_config.tm_port,
        timeout=client_config.timeout,
        allow_version_mismatch=client_config.allow_version_mismatch,
        render_preset=args.render_preset if hasattr(args, "render_preset") else None,
        render_presets_path=Path(args.render_presets) if hasattr(args, "render_preset") and args.render_preset else None,
    )


def do_test(_args: argparse.Namespace) -> None:
    from .editor import test_editor
    test_editor.run_all_tests()


def run_menu(base_args: argparse.Namespace, globals_cfg: Dict[str, Any]) -> None:
    while True:
        print("\n=== CARLA Experiment CLI ===")
        print("1) Export map")
        print("2) Launch interactive editor")
        print("3) Preview plan (PNG)")
        print("4) Compile plan")
        print("5) Validate plan")
        print("6) Render plan")
        print("7) Plan + Validate + Render")
        print("8) Run suite/all")
        print("9) Quit")
        choice = _prompt("Select", "9")
        if choice == "1":
            map_name = _prompt("Map name", "Town05")
            out = _prompt("Map output dir", "data/maps")
            waypoint_distance = _prompt_float("Waypoint distance (m)", 2.0)
            max_seconds = _prompt_float("Max seconds", 600.0)
            args = SimpleNamespace(
                map=map_name,
                out=out,
                host=None,
                port=None,
                timeout=None,
                waypoint_distance=waypoint_distance,
                max_seconds=max_seconds,
            )
            do_map(args, globals_cfg)
        elif choice == "2":
            map_dir = _prompt("Map dir", "data/maps/Town05")
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            out = _prompt("Output root", "outputs")
            duration = _prompt_float("Duration (s)", 60.0)
            dt = _prompt_float("dt (s)", 0.05)
            scene = _prompt("Scene path (optional)", "")
            headless = _prompt("Headless (y/n)", "n").lower().startswith("y")
            args = SimpleNamespace(
                map_dir=map_dir,
                episode_id=episode_id,
                out=out,
                globals=base_args.globals,
                dt=dt,
                duration=duration,
                scene=scene if scene else None,
                town=None,
                headless=headless,
            )
            do_editor(args)
        elif choice == "3":
            map_dir = _prompt("Map dir", "data/maps/Town05")
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            out_root = _prompt("Output root", "outputs")
            out_file = _prompt("Output PNG (optional)", "")
            args = SimpleNamespace(
                map_dir=map_dir,
                episode=episode_id,
                plan=None,
                out=out_file if out_file else None,
                out_root=out_root,
            )
            do_preview(args)
        elif choice == "4":
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            episodes_dir = _prompt("Episodes dir", "configs/episodes")
            out = _prompt("Output root", "outputs")
            max_seconds = _prompt_float("Max seconds", 300.0)
            args = SimpleNamespace(
                episode=episode_id,
                globals=base_args.globals,
                episodes_dir=episodes_dir,
                out=out,
                max_seconds=max_seconds,
            )
            do_plan(args)
        elif choice == "5":
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            out = _prompt("Output root", "outputs")
            max_seconds = _prompt_float("Max seconds", 300.0)
            args = SimpleNamespace(
                episode=episode_id,
                globals=base_args.globals,
                out=out,
                plan=None,
                max_seconds=max_seconds,
            )
            do_validate(args)
        elif choice == "6":
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            out = _prompt("Output root", "outputs")
            quick = _prompt("Quick mode (y/n)", "y").lower().startswith("y")
            max_seconds = _prompt_float("Max seconds", 1200.0)
            max_tick = _prompt_float("Max tick seconds", 10.0)
            encode_timeout = _prompt_float("Encode timeout", 300.0)
            args = SimpleNamespace(
                episode=episode_id,
                globals=base_args.globals,
                out=out,
                plan=None,
                max_seconds=max_seconds,
                max_tick_seconds=max_tick,
                encode_timeout=encode_timeout,
                quick=quick,
                image_size=None,
            )
            do_render(args)
        elif choice == "7":
            episode_id = _prompt("Episode ID", "P1_T2_lane_change")
            episodes_dir = _prompt("Episodes dir", "configs/episodes")
            out = _prompt("Output root", "outputs")
            quick = _prompt("Quick mode (y/n)", "y").lower().startswith("y")
            args = SimpleNamespace(
                episode=episode_id,
                globals=base_args.globals,
                episodes_dir=episodes_dir,
                out=out,
                max_seconds=300.0,
                max_validate_seconds=300.0,
                max_render_seconds=1200.0,
                max_tick_seconds=10.0,
                encode_timeout=300.0,
                quick=quick,
                image_size=None,
            )
            do_pipeline(args)
        elif choice == "8":
            suite = _prompt("Suite ID (P1..P6) or 'all'", "P1")
            out = _prompt("Output root", "outputs")
            episodes_dir = _prompt("Episodes dir", "configs/episodes")
            suites_dir = _prompt("Suites dir", "configs/suites")
            quick = _prompt("Quick mode (y/n)", "y").lower().startswith("y")
            args = SimpleNamespace(
                suite=suite if suite != "all" else None,
                all=(suite == "all"),
                globals=base_args.globals,
                episodes_dir=episodes_dir,
                suites_dir=suites_dir,
                out=out,
                max_plan_seconds=300.0,
                max_validate_seconds=300.0,
                max_render_seconds=1200.0,
                max_tick_seconds=10.0,
                encode_timeout=300.0,
                quick=quick,
                image_size=None,
            )
            do_suite(args)
        elif choice == "9":
            return
        else:
            print("Unknown selection.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified CARLA Experiment CLI.")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--menu", action="store_true", help="Launch interactive menu")

    subparsers = parser.add_subparsers(dest="command")

    map_cmd = subparsers.add_parser("map", help="Export map assets")
    map_cmd.add_argument("--map", required=True)
    map_cmd.add_argument("--out", default="data/maps")
    map_cmd.add_argument("--host")
    map_cmd.add_argument("--port", type=int)
    map_cmd.add_argument("--timeout", type=float)
    map_cmd.add_argument("--waypoint-distance", type=float, default=2.0)
    map_cmd.add_argument("--max-seconds", type=float, default=600.0)

    plan_cmd = subparsers.add_parser("plan", help="Compile plan.json")
    plan_cmd.add_argument("--episode", required=True)
    plan_cmd.add_argument("--episodes-dir", default="configs/episodes")
    plan_cmd.add_argument("--out", default="outputs")
    plan_cmd.add_argument("--max-seconds", type=float, default=300.0)

    validate_cmd = subparsers.add_parser("validate", help="Validate plan.json")
    validate_cmd.add_argument("--episode", required=True)
    validate_cmd.add_argument("--out", default="outputs")
    validate_cmd.add_argument("--plan")
    validate_cmd.add_argument("--max-seconds", type=float, default=300.0)

    render_cmd = subparsers.add_parser("render", help="Render a plan.json")
    render_cmd.add_argument("--episode", required=True)
    render_cmd.add_argument("--out", default="outputs")
    render_cmd.add_argument("--plan")
    render_cmd.add_argument("--max-seconds", type=float, default=1200.0)
    render_cmd.add_argument("--max-tick-seconds", type=float, default=10.0)
    render_cmd.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    render_cmd.add_argument("--encode-timeout", type=float, default=300.0)
    render_cmd.add_argument("--quick", action="store_true")

    pipeline_cmd = subparsers.add_parser("pipeline", help="Plan + validate + render")
    pipeline_cmd.add_argument("--episode", required=True)
    pipeline_cmd.add_argument("--episodes-dir", default="configs/episodes")
    pipeline_cmd.add_argument("--out", default="outputs")
    pipeline_cmd.add_argument("--max-seconds", type=float, default=300.0)
    pipeline_cmd.add_argument("--max-validate-seconds", type=float, default=300.0)
    pipeline_cmd.add_argument("--max-render-seconds", type=float, default=1200.0)
    pipeline_cmd.add_argument("--max-tick-seconds", type=float, default=10.0)
    pipeline_cmd.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    pipeline_cmd.add_argument("--encode-timeout", type=float, default=300.0)
    pipeline_cmd.add_argument("--quick", action="store_true")

    preview_cmd = subparsers.add_parser("preview", help="Render 2D preview PNG")
    preview_cmd.add_argument("--map-dir", required=True)
    preview_cmd.add_argument("--plan")
    preview_cmd.add_argument("--episode")
    preview_cmd.add_argument("--out")
    preview_cmd.add_argument("--out-root", default="outputs")

    editor_cmd = subparsers.add_parser("editor", help="Launch interactive scene editor")
    editor_cmd.add_argument("--map-dir", required=True)
    editor_cmd.add_argument("--scene")
    editor_cmd.add_argument("--out", default="outputs")
    editor_cmd.add_argument("--episode-id", default="episode")
    editor_cmd.add_argument("--town")
    editor_cmd.add_argument("--dt", type=float, default=0.05)
    editor_cmd.add_argument("--duration", type=float, default=60.0)
    editor_cmd.add_argument("--headless", action="store_true", help="Run without GUI and export outputs")

    suite_cmd = subparsers.add_parser("suite", help="Run a suite or all suites")
    suite_cmd.add_argument("--suite")
    suite_cmd.add_argument("--all", action="store_true")
    suite_cmd.add_argument("--episodes-dir", default="configs/episodes")
    suite_cmd.add_argument("--suites-dir", default="configs/suites")
    suite_cmd.add_argument("--out", default="outputs")
    suite_cmd.add_argument("--max-plan-seconds", type=float, default=300.0)
    suite_cmd.add_argument("--max-validate-seconds", type=float, default=300.0)
    suite_cmd.add_argument("--max-render-seconds", type=float, default=1200.0)
    suite_cmd.add_argument("--max-tick-seconds", type=float, default=10.0)
    suite_cmd.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    suite_cmd.add_argument("--encode-timeout", type=float, default=300.0)
    suite_cmd.add_argument("--quick", action="store_true")

    compare_cmd = subparsers.add_parser("compare", help="Generate telemetry vs keyframe comparison")
    compare_cmd.add_argument("--scene", required=True, help="Path to scene_edit.json")
    compare_cmd.add_argument("--telemetry-csv", required=True, help="Path to telemetry.csv")
    compare_cmd.add_argument("--events", help="Path to events.json")
    compare_cmd.add_argument("--output", required=True, help="Output directory")
    compare_cmd.add_argument("--actors", help="Comma-separated actor filter")

    convert_cmd = subparsers.add_parser("convert", help="Convert telemetry to scene_edit.json")
    convert_cmd.add_argument("--csv", required=True, help="Path to telemetry.csv")
    convert_cmd.add_argument("--json", required=True, help="Path to telemetry.json")
    convert_cmd.add_argument("--events", required=True, help="Path to events.json")
    convert_cmd.add_argument("--output", required=True, help="Output scene_edit.json path")
    convert_cmd.add_argument("--town", default="Town05")
    convert_cmd.add_argument("--duration", type=float, default=60.0)
    convert_cmd.add_argument("--dt", type=float, default=0.05)

    bundle_cmd = subparsers.add_parser("bundle", help="Collect episode artifacts into a bundle directory")
    bundle_cmd.add_argument("--episode", required=True, help="Episode ID")
    bundle_cmd.add_argument("--out", default="outputs", help="Output root containing episode dirs")
    bundle_cmd.add_argument("--scene", help="Explicit path to scene_edit.json")
    bundle_cmd.add_argument("--scene-designs", default="outputs/scene_designs", help="Scene designs root dir")
    bundle_cmd.add_argument("--bundle-dir", help="Override bundle output directory")

    test_cmd = subparsers.add_parser("test", help="Run automated editor tests")

    # V3 Director-driven scenario runner
    v3_cmd = subparsers.add_parser("run-v3", help="Run V3 Director-driven scenario")
    v3_cmd.add_argument("--scenario", required=True, help="Scenario ID or path to YAML config")
    v3_cmd.add_argument("--out", type=str, help="Output directory")
    v3_cmd.add_argument("--host")
    v3_cmd.add_argument("--port", type=int)
    v3_cmd.add_argument("--tm-port", type=int)
    v3_cmd.add_argument("--timeout", type=float)
    v3_cmd.add_argument("--render-preset", help="Render preset name")
    v3_cmd.add_argument("--render-presets", type=str, default="configs/render_presets.yaml")
    v3_cmd.add_argument("--client-config", type=str, default="configs/client.yaml")
    v3_cmd.add_argument("--allow-version-mismatch", action="store_true", default=False)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    parser = build_parser()
    args = parser.parse_args(argv)
    globals_cfg = _load_globals(Path(args.globals))

    if args.command is None or args.menu:
        run_menu(args, globals_cfg)
        return 0

    if args.command == "map":
        do_map(args, globals_cfg)
        return 0
    if args.command == "plan":
        do_plan(args)
        return 0
    if args.command == "validate":
        do_validate(args)
        return 0
    if args.command == "render":
        do_render(args)
        return 0
    if args.command == "pipeline":
        do_pipeline(args)
        return 0
    if args.command == "preview":
        do_preview(args)
        return 0
    if args.command == "editor":
        do_editor(args)
        return 0
    if args.command == "suite":
        if not args.suite and not args.all:
            raise SystemExit("suite requires --suite or --all")
        do_suite(args)
        return 0
    if args.command == "compare":
        do_compare(args)
        return 0
    if args.command == "convert":
        do_convert(args)
        return 0
    if args.command == "bundle":
        do_bundle(args)
        return 0
    if args.command == "test":
        do_test(args)
        return 0
    if args.command == "run-v3":
        do_run_v3(args, globals_cfg)
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
