"""Video encoding helpers."""

from __future__ import annotations

from pathlib import Path

from .utils import require_binary, run_command


def encode_frames_to_mp4(
    frames_dir: Path,
    out_path: Path,
    fps: int,
    *,
    timeout_s: float | None = None,
) -> None:
    require_binary("ffmpeg", "Install with: sudo apt-get install ffmpeg")
    input_pattern = frames_dir / "%06d.png"
    run_command(
        [
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            str(fps),
            "-i",
            str(input_pattern),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            str(out_path),
        ],
        timeout=timeout_s,
    )


def mux_audio_to_video(
    video_path: Path,
    audio_path: Path,
    out_path: Path,
    *,
    timeout_s: float | None = None,
) -> None:
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
        ],
        timeout=timeout_s,
    )
