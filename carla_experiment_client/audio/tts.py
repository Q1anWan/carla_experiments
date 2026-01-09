"""Text-to-speech helpers with a stub fallback."""

from __future__ import annotations

import asyncio
import math
import struct
import wave
from pathlib import Path
from typing import Optional


def estimate_duration(text: str, words_per_minute: int = 150) -> float:
    words = max(1, len(text.split()))
    minutes = words / float(words_per_minute)
    return max(0.6, minutes * 60.0)


def write_tone_wav(path: Path, duration: float, *, freq: float = 440.0) -> None:
    sample_rate = 22050
    total_frames = int(sample_rate * duration)
    amplitude = 0.2
    with wave.open(str(path), "w") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        for i in range(total_frames):
            value = int(amplitude * math.sin(2 * math.pi * freq * i / sample_rate) * 32767)
            wav_file.writeframesraw(struct.pack("<h", value))


def synthesize(text: str, out_path: Path, *, voice: str = "en-US-AriaNeural") -> None:
    try:
        import edge_tts  # type: ignore
    except ImportError:
        duration = estimate_duration(text)
        write_tone_wav(out_path, duration)
        return

    async def _run() -> None:
        communicate = edge_tts.Communicate(text, voice=voice)
        await communicate.save(str(out_path))

    asyncio.run(_run())


def wav_duration(path: Path) -> float:
    with wave.open(str(path), "rb") as wav_file:
        frames = wav_file.getnframes()
        rate = wav_file.getframerate()
    return frames / float(rate)
