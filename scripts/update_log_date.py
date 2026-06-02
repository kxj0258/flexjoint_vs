#!/usr/bin/env python3
"""Replace a log folder date in file contents and path names.

Default run is a dry-run. Add --apply to make changes.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass
from datetime import date
from pathlib import Path


@dataclass(frozen=True)
class ContentChange:
    path: Path
    compact_count: int
    iso_count: int

    @property
    def total(self) -> int:
        return self.compact_count + self.iso_count


@dataclass(frozen=True)
class RenameChange:
    src: Path
    dst: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replace 20260530/2026-05-30 style dates in a log tree, "
            "including file contents and file/directory names."
        )
    )
    parser.add_argument(
        "root",
        nargs="?",
        default="data/log/20260530_two_features2",
        help="root folder to update (default: data/log/20260530_two_features2)",
    )
    parser.add_argument(
        "--old-date",
        default="2026-05-30",
        help="old ISO date, used with compact YYYYMMDD form too (default: 2026-05-30)",
    )
    parser.add_argument(
        "--new-date",
        default="2026-06-01",
        help="new ISO date, used with compact YYYYMMDD form too (default: 2026-06-01)",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="actually write file contents and rename paths; without this it only previews",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        help="before changing a file, keep a .bak copy next to it",
    )
    parser.add_argument(
        "--skip-binary",
        action="store_true",
        help="skip files that look binary instead of scanning all files byte-for-byte",
    )
    parser.add_argument(
        "--list-limit",
        type=int,
        default=80,
        help="maximum number of planned changes to print in each section (default: 80)",
    )
    return parser.parse_args()


def date_forms(value: str) -> tuple[str, str]:
    parsed = date.fromisoformat(value)
    return parsed.isoformat(), parsed.strftime("%Y%m%d")


def looks_binary(path: Path) -> bool:
    try:
        with path.open("rb") as handle:
            sample = handle.read(4096)
    except OSError as exc:
        raise RuntimeError(f"failed reading {path}: {exc}") from exc
    return b"\0" in sample


def iter_regular_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for path in root.rglob("*"):
        if path.is_symlink():
            continue
        if path.is_file():
            files.append(path)
    return files


def plan_content_changes(
    files: list[Path],
    old_compact: bytes,
    new_compact: bytes,
    old_iso: bytes,
    new_iso: bytes,
    skip_binary: bool,
) -> list[ContentChange]:
    changes: list[ContentChange] = []
    for path in files:
        if skip_binary and looks_binary(path):
            continue
        try:
            data = path.read_bytes()
        except OSError as exc:
            raise RuntimeError(f"failed reading {path}: {exc}") from exc

        compact_count = data.count(old_compact)
        iso_count = data.count(old_iso)
        if compact_count or iso_count:
            changes.append(ContentChange(path, compact_count, iso_count))
    return changes


def apply_content_changes(
    changes: list[ContentChange],
    old_compact: bytes,
    new_compact: bytes,
    old_iso: bytes,
    new_iso: bytes,
    backup: bool,
) -> None:
    for change in changes:
        data = change.path.read_bytes()
        new_data = data.replace(old_compact, new_compact).replace(old_iso, new_iso)
        if new_data == data:
            continue
        if backup:
            backup_path = change.path.with_name(change.path.name + ".bak")
            if backup_path.exists():
                raise RuntimeError(f"backup already exists: {backup_path}")
            shutil.copy2(change.path, backup_path)
        change.path.write_bytes(new_data)


def replace_name(name: str, old_compact: str, new_compact: str, old_iso: str, new_iso: str) -> str:
    return name.replace(old_compact, new_compact).replace(old_iso, new_iso)


def plan_renames(
    root: Path,
    old_compact: str,
    new_compact: str,
    old_iso: str,
    new_iso: str,
) -> list[RenameChange]:
    paths = [root]
    paths.extend(root.rglob("*"))

    changes: list[RenameChange] = []
    for path in paths:
        if path.is_symlink():
            continue
        new_name = replace_name(path.name, old_compact, new_compact, old_iso, new_iso)
        if new_name != path.name:
            changes.append(RenameChange(path, path.with_name(new_name)))

    changes.sort(key=lambda change: len(change.src.parts), reverse=True)
    return changes


def check_rename_collisions(changes: list[RenameChange]) -> None:
    destinations = {}
    sources = {change.src for change in changes}
    for change in changes:
        destinations.setdefault(change.dst, []).append(change.src)

    for dst, srcs in destinations.items():
        if len(srcs) > 1:
            joined = ", ".join(str(src) for src in srcs)
            raise RuntimeError(f"multiple paths would rename to {dst}: {joined}")
        if dst.exists() and dst not in sources:
            raise RuntimeError(f"rename destination already exists: {dst}")


def apply_renames(changes: list[RenameChange]) -> None:
    check_rename_collisions(changes)
    for change in changes:
        if not change.src.exists():
            raise RuntimeError(f"rename source disappeared: {change.src}")
        change.src.rename(change.dst)


def print_limited(title: str, rows: list[str], limit: int) -> None:
    print(f"\n{title}: {len(rows)}")
    for row in rows[: max(limit, 0)]:
        print(f"  {row}")
    extra = len(rows) - max(limit, 0)
    if extra > 0:
        print(f"  ... {extra} more")


def main() -> int:
    args = parse_args()
    root = Path(args.root)
    if not root.exists() or not root.is_dir():
        print(f"error: root folder does not exist or is not a directory: {root}", file=sys.stderr)
        return 2

    old_iso, old_compact = date_forms(args.old_date)
    new_iso, new_compact = date_forms(args.new_date)

    replacements = [
        (old_compact.encode("utf-8"), new_compact.encode("utf-8")),
        (old_iso.encode("utf-8"), new_iso.encode("utf-8")),
    ]
    for old, new in replacements:
        if len(old) != len(new):
            print(
                "error: old and new date byte lengths differ; refusing binary-safe replacement",
                file=sys.stderr,
            )
            return 2

    files = iter_regular_files(root)
    content_changes = plan_content_changes(
        files,
        old_compact.encode("utf-8"),
        new_compact.encode("utf-8"),
        old_iso.encode("utf-8"),
        new_iso.encode("utf-8"),
        args.skip_binary,
    )
    rename_changes = plan_renames(root, old_compact, new_compact, old_iso, new_iso)

    check_rename_collisions(rename_changes)

    content_rows = [
        f"{change.path}  ({old_compact}: {change.compact_count}, {old_iso}: {change.iso_count})"
        for change in content_changes
    ]
    rename_rows = [f"{change.src} -> {change.dst}" for change in rename_changes]

    print(f"Root: {root}")
    print(f"Replace: {old_compact} -> {new_compact}, {old_iso} -> {new_iso}")
    print(f"Scanned regular files: {len(files)}")
    print(f"Total content replacements: {sum(change.total for change in content_changes)}")
    print_limited("Files with content changes", content_rows, args.list_limit)
    print_limited("Path renames", rename_rows, args.list_limit)

    if not args.apply:
        print("\nDry-run only. Re-run with --apply to modify files and directories.")
        return 0

    apply_content_changes(
        content_changes,
        old_compact.encode("utf-8"),
        new_compact.encode("utf-8"),
        old_iso.encode("utf-8"),
        new_iso.encode("utf-8"),
        args.backup,
    )
    apply_renames(rename_changes)
    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
