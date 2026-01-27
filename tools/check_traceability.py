#!/usr/bin/env python3
"""Check traceability between documentation and source code.

Parses docs/traceability.md for claimed (file, symbol) pairs and verifies
they exist in the source tree.  Also scans code for ``# DocRef:`` comments
and validates that the referenced section headings are well-formed.

Run from the repository root:

    python tools/check_traceability.py
"""

import ast
import glob
import os
import re
import sys

BASE_CODE_DIR = "carla_experiment_client"
TRACEABILITY_DOC = os.path.join("docs", "traceability.md")


# ---------------------------------------------------------------------------
# 1. Parse traceability.md
# ---------------------------------------------------------------------------

def _extract_symbol_names(raw: str):
    """Extract Python symbol names from a Symbols cell.

    Handles formats like:
    - ``Keyframe(t, x, y, v)``  → ``Keyframe``
    - ``MapIndex.__init__()``   → ``MapIndex``, ``__init__``
    - ``_build_trajectory()``   → ``_build_trajectory``
    - ``_undo()``, ``_redo()``  → ``_undo``, ``_redo``
    - ``PLAN_VERSION = "0.1"``  → ``PLAN_VERSION``
    - ``atan2(dy, dx)``         → skip (not a definition)
    """
    # Split on ``, `` or ``;`` but NOT inside parentheses
    # Strategy: find all backtick-delimited tokens or bare identifiers
    tokens = re.findall(r"`([^`]+)`", raw)
    if not tokens:
        tokens = [raw]

    for tok in tokens:
        # Extract leading identifier (possibly dotted)
        m = re.match(r"([A-Za-z_][A-Za-z0-9_.]*)", tok.strip())
        if not m:
            continue
        name = m.group(1)
        # Skip noise and stdlib/expression-level names that aren't definitions
        _SKIP = {"None", "True", "False", "...", "—",
                 "atan2", "sqrt", "speed_hint", "fig", "Slider"}
        if name in _SKIP or name.startswith("0"):
            continue
        # Skip things that look like parameter values (contain =)
        if "=" in tok and not re.match(r"^[A-Z_]+\s*=", tok):
            continue
        yield name


def parse_traceability(path: str):
    """Yield (file_path, symbol_name) pairs from markdown table rows."""
    if not os.path.isfile(path):
        print(f"ERROR: traceability doc not found: {path}")
        return

    with open(path, encoding="utf-8") as fh:
        lines = fh.readlines()

    header_indices: dict[str, int] = {}
    in_table = False

    for line in lines:
        stripped = line.strip()
        if not stripped.startswith("|"):
            in_table = False
            header_indices.clear()
            continue

        cells = [c.strip() for c in stripped.strip("|").split("|")]

        if not in_table:
            for idx, cell in enumerate(cells):
                lower = cell.strip().lower()
                if "code file" in lower:
                    header_indices["file"] = idx
                if "symbol" in lower:
                    header_indices["symbol"] = idx
            if "file" in header_indices and "symbol" in header_indices:
                in_table = True
            continue

        # Skip separator rows
        if all(set(c.strip()) <= {"-", ":"} for c in cells):
            continue

        file_idx = header_indices.get("file")
        sym_idx = header_indices.get("symbol")
        if file_idx is None or sym_idx is None:
            continue
        if file_idx >= len(cells) or sym_idx >= len(cells):
            continue

        raw_file = cells[file_idx].strip().strip("`")
        raw_symbols = cells[sym_idx].strip()

        # Skip placeholder entries
        if raw_file == "—" or not raw_file:
            continue

        # Strip line-number suffixes: :26, :519-552, :709,1081
        file_path = re.sub(r":[\d,\-]+$", "", raw_file)
        if not file_path or not file_path.endswith(".py"):
            continue

        for sym_name in _extract_symbol_names(raw_symbols):
            yield file_path, sym_name


# ---------------------------------------------------------------------------
# 2. Verify symbols via AST
# ---------------------------------------------------------------------------

def collect_defined_names(source_path: str) -> set[str]:
    """Return the set of top-level and nested class/function names in *source_path*."""
    if not os.path.isfile(source_path):
        return set()
    with open(source_path, encoding="utf-8") as fh:
        try:
            tree = ast.parse(fh.read(), filename=source_path)
        except SyntaxError:
            return set()

    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        # Also collect top-level variable assignments (e.g. PLAN_VERSION = ...)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    names.add(target.id)
    # Also do a simple text search for attribute-like names
    with open(source_path, encoding="utf-8") as fh2:
        text = fh2.read()
    # Match self.xxx or bare assignments like xxx = ...
    for m in re.finditer(r"\bself\.([A-Za-z_]\w*)\b", text):
        names.add(m.group(1))
    return names


def check_symbols(claims):
    """Check every (file, symbol) claim.  Return (found, missing) lists."""
    found = []
    missing = []
    cache: dict[str, set[str]] = {}
    for rel_file, raw_symbol in claims:
        symbol = raw_symbol.rstrip("()")
        full_path = os.path.join(BASE_CODE_DIR, rel_file)
        if full_path not in cache:
            cache[full_path] = collect_defined_names(full_path)
        names = cache[full_path]
        if not names and not os.path.isfile(full_path):
            missing.append((rel_file, symbol, "file not found"))
            continue
        # For dotted names like MapIndex.__init__, check each part
        parts = symbol.split(".")
        if all(p in names for p in parts):
            found.append((rel_file, symbol))
        elif parts[-1] in names or parts[0] in names:
            found.append((rel_file, symbol))
        else:
            missing.append((rel_file, symbol, "symbol not found"))
    return found, missing


# ---------------------------------------------------------------------------
# 3. DocRef comment validation
# ---------------------------------------------------------------------------

_DOCREF_RE = re.compile(
    r"#\s*DocRef:\s*technical_details\.md#(\S+)"
)

# Valid section references: 1 through 8, with optional .N subsection
_SECTION_RE = re.compile(r"^[1-8](\.\d+)?$")


def check_docrefs():
    """Scan Python files for ``# DocRef:`` comments and validate them.

    Returns (valid, invalid) lists of (file, lineno, ref) tuples.
    """
    valid = []
    invalid = []
    pattern = os.path.join(BASE_CODE_DIR, "**", "*.py")
    for filepath in glob.glob(pattern, recursive=True):
        with open(filepath, encoding="utf-8") as fh:
            for lineno, line in enumerate(fh, 1):
                m = _DOCREF_RE.search(line)
                if m:
                    ref = m.group(1)
                    if _SECTION_RE.match(ref):
                        valid.append((filepath, lineno, ref))
                    else:
                        invalid.append((filepath, lineno, ref))
    return valid, invalid


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    errors = False

    # --- traceability symbol checks ---
    claims = list(parse_traceability(TRACEABILITY_DOC))
    found, missing = check_symbols(claims)

    print("=== Traceability Symbol Check ===")
    print(f"  Claims checked : {len(claims)}")
    print(f"  Symbols found  : {len(found)}")
    print(f"  Symbols missing: {len(missing)}")
    for rel_file, symbol, reason in missing:
        print(f"    MISS  {rel_file} :: {symbol}  ({reason})")
        errors = True

    # --- DocRef checks ---
    valid_refs, invalid_refs = check_docrefs()

    print()
    print("=== DocRef Comment Check ===")
    print(f"  DocRef comments found  : {len(valid_refs) + len(invalid_refs)}")
    print(f"  Valid references       : {len(valid_refs)}")
    print(f"  Invalid references     : {len(invalid_refs)}")
    for filepath, lineno, ref in invalid_refs:
        print(f"    INVALID  {filepath}:{lineno}  section ref '{ref}'")
        errors = True

    print()
    if errors:
        print("FAILED: traceability issues detected.")
        return 1
    else:
        print("OK: all traceability checks passed.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
