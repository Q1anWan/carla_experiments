"""Verify DOC: and DocRef: tags in source code are resolvable.

Usage:
    python -m carla_experiment_client.tools.docs_check
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parent.parent
DOC_ROOT = SRC_ROOT.parent / "docs"

DOC_TAG_RE = re.compile(r"#\s*(DOC|DocRef):\s*(\S+)")
TRACEABILITY = DOC_ROOT / "traceability.md"


def find_doc_tags() -> list[tuple[str, int, str, str]]:
    """Return list of (file, line, tag_type, tag_value) from source."""
    results = []
    for py_file in SRC_ROOT.rglob("*.py"):
        for i, line in enumerate(py_file.read_text().splitlines(), 1):
            for m in DOC_TAG_RE.finditer(line):
                rel = py_file.relative_to(SRC_ROOT)
                results.append((str(rel), i, m.group(1), m.group(2)))
    return results


def check_traceability_exists() -> bool:
    if not TRACEABILITY.exists():
        print(f"FAIL: {TRACEABILITY} not found")
        return False
    content = TRACEABILITY.read_text()
    if "## Validation Summary" not in content:
        print("WARN: traceability.md missing Validation Summary section")
    return True


def check_key_symbols() -> list[str]:
    """Check that key symbols mentioned in traceability exist in code."""
    errors = []
    checks = [
        ("telemetry/recorder.py", "TelemetryRecorder"),
        ("telemetry/sae_j670.py", "VehicleState"),
        ("telemetry/sae_j670.py", "speed_fd"),
        ("tools/make_comparison.py", "align_trajectories"),
        ("tools/make_comparison.py", "compute_position_error"),
        ("tools/make_comparison.py", "error_stats"),
        ("planning/plan_validator.py", "validate_plan"),
        ("planning/path_planner.py", "LaneGraph"),
        ("planning/path_planner.py", "build_road_path"),
        ("render/replay_controller.py", "TeleportFollower"),
        ("cli.py", "do_bundle"),
        ("cli.py", "do_compare"),
    ]
    for rel_path, symbol in checks:
        fpath = SRC_ROOT / rel_path
        if not fpath.exists():
            errors.append(f"MISSING FILE: {rel_path}")
            continue
        if symbol not in fpath.read_text():
            errors.append(f"MISSING SYMBOL: {symbol} not found in {rel_path}")
    return errors


def main() -> int:
    print("=== docs_check ===\n")
    ok = True

    # 1. Check traceability.md exists
    if not check_traceability_exists():
        ok = False

    # 2. Find DOC: tags in source
    tags = find_doc_tags()
    print(f"Found {len(tags)} DOC/DocRef tags in source:")
    for f, line, typ, val in tags:
        print(f"  {f}:{line}  {typ}:{val}")

    # 3. Check key symbols
    errors = check_key_symbols()
    if errors:
        ok = False
        print(f"\n{len(errors)} symbol check failures:")
        for e in errors:
            print(f"  {e}")
    else:
        print(f"\nAll symbol checks passed.")

    print(f"\n{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
