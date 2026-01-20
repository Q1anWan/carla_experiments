#!/usr/bin/env python3
"""CLI entrypoint for plan/validate/render pipelines."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import List

import yaml

from carla_experiment_client.planning.planner_compiler import compile_episode
from carla_experiment_client.planning.validator import validate_plan
from carla_experiment_client.render.renderer import render_plan


def _load_suite(path: Path) -> List[str]:
    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        return []
    episodes = raw.get("episodes") or []
    return [str(item) for item in episodes]


def _run_episode(episode_id: str, args: argparse.Namespace) -> None:
    logging.info("Plan stage for %s", episode_id)
    out_dir = Path(args.out) / episode_id
    plan = compile_episode(
        episode_id,
        globals_path=Path(args.globals),
        episodes_dir=Path(args.episodes_dir),
        out_dir=out_dir,
        max_seconds=args.max_plan_seconds,
    )
    logging.info("Validate stage for %s", episode_id)
    report = validate_plan(
        out_dir / "plan.json",
        Path(args.globals),
        out_dir,
        max_seconds=args.max_validate_seconds,
    )
    if report.get("status") != "pass":
        logging.warning("Validation failed for %s, skipping render.", episode_id)
        return
    logging.info("Render stage for %s", episode_id)
    render_plan(
        out_dir / "plan.json",
        Path(args.globals),
        out_dir,
        max_seconds=args.max_render_seconds,
        max_tick_seconds=args.max_tick_seconds,
        image_size=args.image_size,
        encode_timeout_s=args.encode_timeout,
    )


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    parser = argparse.ArgumentParser(description="Plan/Validate/Render pipeline runner.")
    parser.add_argument("--globals", default="configs/globals.yaml", help="Global config YAML")
    parser.add_argument("--episodes-dir", default="configs/episodes", help="Episode YAML directory")
    parser.add_argument("--out", default="outputs", help="Output directory root")

    subparsers = parser.add_subparsers(dest="command", required=True)

    plan_cmd = subparsers.add_parser("plan", help="Compile plan.json for an episode")
    plan_cmd.add_argument("--episode", required=True)
    plan_cmd.add_argument("--max-seconds", type=float, default=300.0)

    validate_cmd = subparsers.add_parser("validate", help="Validate a plan.json")
    validate_cmd.add_argument("--episode", required=True)
    validate_cmd.add_argument("--max-seconds", type=float, default=300.0)

    render_cmd = subparsers.add_parser("render", help="Render a plan.json")
    render_cmd.add_argument("--episode", required=True)
    render_cmd.add_argument("--max-seconds", type=float, default=1200.0)
    render_cmd.add_argument("--max-tick-seconds", type=float, default=10.0)
    render_cmd.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    render_cmd.add_argument("--encode-timeout", type=float, default=300.0)
    render_cmd.add_argument("--quick", action="store_true", help="Use low resolution (640x360)")

    all_cmd = subparsers.add_parser("all", help="Run plan+validate+render for a suite or all episodes")
    all_cmd.add_argument("--suite", help="Suite ID (e.g., P1)")
    all_cmd.add_argument("--all", action="store_true", help="Run all suites (P1-P6)")
    all_cmd.add_argument("--max-plan-seconds", type=float, default=300.0)
    all_cmd.add_argument("--max-validate-seconds", type=float, default=300.0)
    all_cmd.add_argument("--max-render-seconds", type=float, default=1200.0)
    all_cmd.add_argument("--max-tick-seconds", type=float, default=10.0)
    all_cmd.add_argument("--image-size", nargs=2, type=int, metavar=("W", "H"))
    all_cmd.add_argument("--encode-timeout", type=float, default=300.0)
    all_cmd.add_argument("--quick", action="store_true", help="Use low resolution (640x360)")

    args = parser.parse_args()

    if args.command == "plan":
        compile_episode(
            args.episode,
            globals_path=Path(args.globals),
            episodes_dir=Path(args.episodes_dir),
            out_dir=Path(args.out) / args.episode,
            max_seconds=args.max_seconds,
        )
        return 0

    if args.command == "validate":
        validate_plan(
            Path(args.out) / args.episode / "plan.json",
            Path(args.globals),
            Path(args.out) / args.episode,
            max_seconds=args.max_seconds,
        )
        return 0

    if args.command == "render":
        image_size = tuple(args.image_size) if args.image_size else None
        if args.quick:
            image_size = (640, 360)
        render_plan(
            Path(args.out) / args.episode / "plan.json",
            Path(args.globals),
            Path(args.out) / args.episode,
            max_seconds=args.max_seconds,
            max_tick_seconds=args.max_tick_seconds,
            image_size=image_size,
            encode_timeout_s=args.encode_timeout,
        )
        return 0

    if args.command == "all":
        episodes: List[str] = []
        if args.all:
            for suite_id in ["P1", "P2", "P3", "P4", "P5", "P6"]:
                suite_path = Path("configs/suites") / f"{suite_id}.yaml"
                episodes.extend(_load_suite(suite_path))
        elif args.suite:
            suite_path = Path("configs/suites") / f"{args.suite}.yaml"
            episodes = _load_suite(suite_path)
        else:
            raise SystemExit("Specify --suite or --all.")
        image_size = tuple(args.image_size) if args.image_size else None
        if args.quick:
            image_size = (640, 360)
        for episode_id in episodes:
            logging.info("Processing episode %s", episode_id)
            args.image_size = image_size
            _run_episode(episode_id, args)
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
