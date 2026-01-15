"""Render experimental stimulus data from a master run.

Generates a single complete dataset with all audio assets and robot timeline.
Playback tool selects which modality to present (Voice V0/V1/V2 × Robot R0/R1).
"""

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
    """Generate robot precue timeline for all events."""
    lines = ["t_start,t_end,action,intensity,event_type,event_index"]
    for idx, event in enumerate(events):
        t_start = float(
            event.get("t_robot_precue", event.get("robot_precue_t", event.get("t", 0.0)))
        )
        t_end = t_start + 0.5
        event_type = event.get("type", "")
        lines.append(f"{t_start:.3f},{t_end:.3f},blink,1.0,{event_type},{idx}")
        lines.append(f"{t_start:.3f},{t_end:.3f},wiggle,0.6,{event_type},{idx}")
    path.write_text("\n".join(lines))


def render_stimulus(run_dir: Path, *, audio_only: bool = False) -> int:
    """Render single complete stimulus dataset.

    Output structure:
        run_dir/
            master_video.mp4      # Base video (no audio overlay)
            events.json           # Event timeline with all timing info
            robot_timeline.csv    # Robot precue commands
            audio/
                voice_what.wav    # V1: decision only
                voice_whatwhy.wav # V2: decision + reason
            stimulus_v1.mp4       # Pre-muxed video with V1 audio (optional)
            stimulus_v2.mp4       # Pre-muxed video with V2 audio (optional)

    Playback tool can:
        - Use master_video.mp4 for V0 (no voice)
        - Play audio/voice_*.wav synchronized for V1/V2
        - Or use pre-muxed stimulus_v*.mp4
        - Trigger robot via robot_timeline.csv for R1
    """
    events_path = run_dir / "events.json"
    master_video = run_dir / "master_video.mp4"
    if not events_path.exists():
        raise RuntimeError("Run directory missing events.json")
    if not audio_only and not master_video.exists():
        raise RuntimeError("Run directory missing master_video.mp4")

    events = _load_events(events_path)

    # Create audio directory
    audio_dir = run_dir / "audio"
    ensure_dir(audio_dir)

    # Generate robot timeline
    _write_robot_timeline(events, run_dir / "robot_timeline.csv")

    # Render audio assets
    v1_audio = audio_dir / "voice_what.wav"
    v2_audio = audio_dir / "voice_whatwhy.wav"
    render_narration(events, v1_audio, voice_level=1)
    render_narration(events, v2_audio, voice_level=2)

    if audio_only:
        return 0

    # Optionally generate pre-muxed videos for convenience
    if v1_audio.exists():
        mux_with_video(master_video, v1_audio, run_dir / "stimulus_v1.mp4")
    if v2_audio.exists():
        mux_with_video(master_video, v2_audio, run_dir / "stimulus_v2.mp4")

    return 0


def render_variants(run_dir: Path, *, audio_only: bool = False) -> int:
    """Legacy wrapper - now calls render_stimulus for single dataset."""
    return render_stimulus(run_dir, audio_only=audio_only)


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
