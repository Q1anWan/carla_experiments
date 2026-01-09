"""Utility helpers."""

from __future__ import annotations

import json
import logging
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2))


def run_command(args: Sequence[str], *, cwd: Path | None = None) -> None:
    logging.debug("Running command: %s", " ".join(args))
    try:
        subprocess.run(args, cwd=str(cwd) if cwd else None, check=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Command failed: {' '.join(args)}") from exc


def require_binary(name: str, install_hint: str) -> str:
    path = shutil.which(name)
    if not path:
        raise RuntimeError(f"Missing required binary '{name}'. {install_hint}")
    return path


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def chunked(values: Iterable[Any], size: int) -> Iterable[list[Any]]:
    chunk: list[Any] = []
    for value in values:
        chunk.append(value)
        if len(chunk) >= size:
            yield chunk
            chunk = []
    if chunk:
        yield chunk
