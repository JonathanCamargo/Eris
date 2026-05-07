#!/usr/bin/env python3
"""Detect drift between duplicated firmware files across Eris flavors.

The Arduino IDE forces each flavor to keep its own copy of `serialcommand.cpp`,
`streaming.cpp`, etc. Over time these copies drift — a bug fix in one flavor
silently leaves the rest stale.

This script picks a reference flavor (default: ErisServo) and compares each of
the canonical "common" files against the same file in every other flavor.
Files that differ are reported with line-count delta and the first ~5 differing
lines so you can decide whether the drift is intentional (flavor-specific) or
unwanted (forgot to propagate).

Usage:
    python tools/check_drift.py
    python tools/check_drift.py --ref Eris
    python tools/check_drift.py --files serialcommand.cpp streaming.cpp
    python tools/check_drift.py --diff             # show full unified diffs
    python tools/check_drift.py --json             # machine-readable output
"""
from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import sys
from pathlib import Path

# Files that are duplicated across most flavors and worth checking.
DEFAULT_FILES = [
    "serialcommand.cpp",
    "serialcommand.h",
    "streaming.cpp",
    "streaming.h",
    "sinewave.cpp",
    "sinewave.h",
    "customtypes.h",
]

# Flavors we explicitly skip (work-in-progress, archived).
SKIP_DIRS = {"wip", "old"}


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def short_diff(a: Path, b: Path, context: int = 0, max_lines: int = 8) -> list[str]:
    a_lines = a.read_text(errors="replace").splitlines()
    b_lines = b.read_text(errors="replace").splitlines()
    diff = list(
        difflib.unified_diff(
            a_lines, b_lines, fromfile=str(a), tofile=str(b), n=context, lineterm=""
        )
    )
    return diff[:max_lines] if max_lines else diff


def list_flavors(root: Path) -> list[Path]:
    return sorted(
        d for d in root.iterdir()
        if d.is_dir() and d.name not in SKIP_DIRS
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=".", help="Path to Eris repo root")
    parser.add_argument("--ref", default="ErisServo",
                        help="Reference flavor (default: ErisServo)")
    parser.add_argument("--files", nargs="+", default=DEFAULT_FILES,
                        help="Files to compare across flavors")
    parser.add_argument("--diff", action="store_true",
                        help="Print full unified diffs for each drift")
    parser.add_argument("--json", action="store_true",
                        help="Emit JSON instead of human-readable text")
    args = parser.parse_args()

    flavors_dir = Path(args.repo) / "Firmware" / "Flavors"
    if not flavors_dir.exists():
        print(f"Flavors directory not found: {flavors_dir}", file=sys.stderr)
        return 2

    ref_dir = flavors_dir / args.ref
    if not ref_dir.exists():
        print(f"Reference flavor not found: {ref_dir}", file=sys.stderr)
        return 2

    report: dict[str, dict] = {}
    for flavor in list_flavors(flavors_dir):
        if flavor == ref_dir:
            continue
        flavor_report: dict[str, dict] = {}
        for fname in args.files:
            ref_file = ref_dir / fname
            this_file = flavor / fname
            if not ref_file.exists() or not this_file.exists():
                continue
            ref_hash = file_hash(ref_file)
            this_hash = file_hash(this_file)
            if ref_hash == this_hash:
                continue
            ref_lines = ref_file.read_text(errors="replace").count("\n")
            this_lines = this_file.read_text(errors="replace").count("\n")
            entry = {
                "ref_lines": ref_lines,
                "lines": this_lines,
                "delta": this_lines - ref_lines,
            }
            if args.diff:
                entry["diff"] = list(
                    difflib.unified_diff(
                        ref_file.read_text(errors="replace").splitlines(),
                        this_file.read_text(errors="replace").splitlines(),
                        fromfile=str(ref_file),
                        tofile=str(this_file),
                        n=2,
                        lineterm="",
                    )
                )
            else:
                entry["sample"] = short_diff(ref_file, this_file, context=0, max_lines=6)
            flavor_report[fname] = entry
        if flavor_report:
            report[flavor.name] = flavor_report

    if args.json:
        print(json.dumps(report, indent=2))
        return 0 if not report else 1

    if not report:
        print(f"No drift detected against reference '{args.ref}' "
              f"for files: {', '.join(args.files)}")
        return 0

    print(f"Drift report (reference: {args.ref})")
    print("=" * 60)
    for flavor, files in report.items():
        print(f"\n[{flavor}]")
        for fname, entry in files.items():
            sign = f"{entry['delta']:+d}"
            print(f"  {fname:24}  ref={entry['ref_lines']:>5} "
                  f"this={entry['lines']:>5}  delta={sign:>4} lines")
            if args.diff and entry.get("diff"):
                for line in entry["diff"]:
                    print(f"    {line}")
            elif entry.get("sample"):
                for line in entry["sample"]:
                    print(f"    {line}")
    flavor_count = len(report)
    print(f"\n{flavor_count} flavor(s) drifted from {args.ref}.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
