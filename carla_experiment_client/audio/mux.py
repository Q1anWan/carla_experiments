"""Audio timeline construction and muxing."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional

from ..utils import ensure_dir, require_binary, run_command
from .tts import synthesize


def render_narration(
    events: Iterable[dict],
    out_path: Path,
    *,
    voice_level: int,
    voice: str = "en-US-AriaNeural",
) -> Optional[Path]:
    events_list = list(events)
    if voice_level <= 0 or not events_list:
        return None

    require_binary("ffmpeg", "Install with: sudo apt-get install ffmpeg")
    temp_dir = out_path.parent / "tts_clips"
    ensure_dir(temp_dir)

    clip_paths = []
    delays_ms = []
    for idx, event in enumerate(events_list):
        decision = event.get("decision_text", "")
        reason = event.get("reason_text", "")
        if voice_level == 1:
            text = decision
        else:
            text = f"{decision}. {reason}".strip()
        clip_path = temp_dir / f"clip_{idx:03d}.wav"
        synthesize(text, clip_path, voice=voice)
        clip_paths.append(clip_path)
        delays_ms.append(int(float(event.get("t", 0.0)) * 1000))

    if not clip_paths:
        return None

    filter_parts = []
    mix_inputs = []
    for idx, delay in enumerate(delays_ms):
        filter_parts.append(f"[{idx}:a]adelay={delay}|{delay}[a{idx}]")
        mix_inputs.append(f"[a{idx}]")
    mix_str = "".join(mix_inputs) + f"amix=inputs={len(mix_inputs)}:normalize=0[aout]"
    filter_complex = ";".join(filter_parts + [mix_str])

    args = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error"]
    for path in clip_paths:
        args.extend(["-i", str(path)])
    args.extend(["-filter_complex", filter_complex, "-map", "[aout]", "-ac", "1", str(out_path)])
    run_command(args)
    return out_path


def mux_with_video(video_path: Path, audio_path: Path, out_path: Path) -> None:
    require_binary("ffmpeg", "Install with: sudo apt-get install ffmpeg")
    run_command(
        [
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(video_path),
            "-i",
            str(audio_path),
            "-c:v",
            "copy",
            "-c:a",
            "aac",
            "-shortest",
            str(out_path),
        ]
    )
