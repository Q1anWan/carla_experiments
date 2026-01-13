"""Render 6 experimental variants from a master run."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from .audio.mux import mux_with_video, render_narration
from .utils import ensure_dir, write_json


def _load_events(path: Path) -> list[dict]:
    data = json.loads(path.read_text())
    return list(data.get("events", []))


def _write_robot_timeline(events: list[dict], path: Path) -> None:
    lines = ["t_start,t_end,action,intensity,notes"]
    for event in events:
        t_start = float(
            event.get("t_robot_precue", event.get("robot_precue_t", event.get("t", 0.0)))
        )
        t_end = t_start + 0.5
        lines.append(f"{t_start:.3f},{t_end:.3f},blink,1.0,{event.get('type','')}")
        lines.append(f"{t_start:.3f},{t_end:.3f},wiggle,0.6,{event.get('type','')}")
    path.write_text("\n".join(lines))


def _render_audio_assets(events: list[dict], run_dir: Path) -> None:
    audio_dir = run_dir / "audio_assets"
    ensure_dir(audio_dir)
    render_narration(events, audio_dir / "what.wav", voice_level=1)
    render_narration(events, audio_dir / "whatwhy.wav", voice_level=2)
    _write_robot_timeline(events, run_dir / "robot_timeline.csv")


def render_variants(run_dir: Path, *, audio_only: bool = False) -> int:
    events_path = run_dir / "events.json"
    master_video = run_dir / "master_video.mp4"
    if not events_path.exists():
        raise RuntimeError("Run directory missing events.json")
    if not audio_only and not master_video.exists():
        raise RuntimeError("Run directory missing events.json or master_video.mp4")

    events = _load_events(events_path)
    if audio_only:
        _render_audio_assets(events, run_dir)
        return 0

    variants_root = run_dir / "variants"
    ensure_dir(variants_root)

    for voice_level in (0, 1, 2):
        for robot_mode in (0, 1):
            variant_dir = variants_root / f"voice{voice_level}_robot{robot_mode}"
            ensure_dir(variant_dir)

            variant_events = {"events": events, "variant": {"voice_level": voice_level, "robot_mode": robot_mode}}
            write_json(variant_dir / "events.json", variant_events)

            stimulus_path = variant_dir / "stimulus.mp4"
            if voice_level == 0:
                shutil.copy2(master_video, stimulus_path)
            else:
                narration_path = variant_dir / "narration.wav"
                rendered = render_narration(events, narration_path, voice_level=voice_level)
                if rendered is None:
                    shutil.copy2(master_video, stimulus_path)
                else:
                    mux_with_video(master_video, rendered, stimulus_path)

            if robot_mode == 1:
                _write_robot_timeline(events, variant_dir / "robot_timeline.csv")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Render stimulus variants.")
    parser.add_argument("--run_dir", type=Path, required=True)
    parser.add_argument(
        "--audio-only",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Only generate audio_assets and robot_timeline.csv",
    )
    args = parser.parse_args()
    return render_variants(args.run_dir, audio_only=args.audio_only)


if __name__ == "__main__":
    raise SystemExit(main())
