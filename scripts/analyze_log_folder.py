#!/usr/bin/env python3
"""Batch-analyze every flexjoint_vs run directory under a log folder."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import yaml

try:
    from analyze_run import (
        FileDialogError,
        analyze_run,
        clean_for_json,
        compute_metrics,
        default_run_root,
        format_metric,
        load_run,
        resolve_path,
        select_directory_dialog,
    )
except ModuleNotFoundError:  # pragma: no cover - supports package-style imports
    from scripts.analyze_run import (
        FileDialogError,
        analyze_run,
        clean_for_json,
        compute_metrics,
        default_run_root,
        format_metric,
        load_run,
        resolve_path,
        select_directory_dialog,
    )


SUMMARY_FIELDS = [
    "run_name",
    "run_dir",
    "analysis_dir",
    "status",
    "error",
    "controller_mode",
    "feature_count",
    "desired_coords",
    "task_key",
    "exit_reason",
    "sample_count",
    "duration_s",
    "initial_rms_image_error_px",
    "final_rms_image_error_px",
    "convergence_time_s",
    "max_abs_velocity_command_rad_s",
    "mean_abs_velocity_command_rad_s",
    "rms_zf1_nm",
    "max_abs_zf1_nm",
    "rms_zf2_nm_s",
    "max_abs_zf2_nm_s",
    "vision_valid_ratio",
    "feedback_valid_ratio",
]


def has_run_files(path: Path) -> bool:
    return (path / "dataFile.txt").exists() and (path / "run_config.yaml").exists()


def find_run_dirs(log_root: Path, recursive: bool) -> List[Path]:
    if has_run_files(log_root):
        return [log_root]

    if recursive:
        dirs = {path.parent for path in log_root.rglob("dataFile.txt")}
        return sorted(path for path in dirs if has_run_files(path))

    return sorted(
        child for child in log_root.iterdir()
        if child.is_dir() and has_run_files(child)
    )


def metric_value(metrics: Dict[str, Any], key: str) -> Any:
    value = metrics.get(key)
    if isinstance(value, list):
        return json.dumps(clean_for_json(value), ensure_ascii=False)
    return clean_for_json(value)


def nested_get(mapping: Dict[str, Any], keys: Sequence[str],
               default: Any = None) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def normalize_desired_coords(value: Any) -> List[float]:
    if not isinstance(value, list):
        return []
    return [float(item) for item in value]


def format_desired_coords(coords: Sequence[float]) -> str:
    return "[" + ", ".join(f"{coord:.6g}" for coord in coords) + "]"


def make_task_key(feature_count: Any, desired_coords: Sequence[float]) -> str:
    return f"N={feature_count}; yd={format_desired_coords(desired_coords)}"


def load_run_metadata(run_dir: Path) -> Dict[str, Any]:
    config_file = run_dir / "run_config.yaml"
    metadata: Dict[str, Any] = {
        "controller_mode": "",
        "feature_count": "",
        "desired_coords": [],
        "task_key": "",
    }
    try:
        with config_file.open("r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}
    except Exception as exc:
        metadata["config_error"] = str(exc)
        return metadata

    controller_mode = nested_get(config, ["experiment", "controller_mode"], "")
    desired_coords = normalize_desired_coords(
        nested_get(config, ["vision", "desired_coords"], [])
    )
    feature_count = nested_get(config, ["vision", "feature_count"], None)
    if feature_count is None and desired_coords:
        feature_count = len(desired_coords) // 2

    metadata["controller_mode"] = str(controller_mode or "")
    metadata["feature_count"] = feature_count if feature_count is not None else ""
    metadata["desired_coords"] = desired_coords
    metadata["task_key"] = make_task_key(metadata["feature_count"], desired_coords)
    return metadata


def result_row(result: Dict[str, Any]) -> Dict[str, Any]:
    metrics = result.get("metrics") or {}
    metadata = result.get("metadata") or {}
    row: Dict[str, Any] = {
        "run_name": result["run_dir"].name,
        "run_dir": str(result["run_dir"]),
        "analysis_dir": str(result.get("analysis_dir") or ""),
        "status": result["status"],
        "error": result.get("error", ""),
        "controller_mode": metadata.get("controller_mode", ""),
        "feature_count": metadata.get("feature_count", ""),
        "desired_coords": format_desired_coords(metadata.get("desired_coords") or []),
        "task_key": metadata.get("task_key", ""),
    }
    for field in SUMMARY_FIELDS:
        if field in row:
            continue
        row[field] = metric_value(metrics, field)
    return row


def write_csv(results: Sequence[Dict[str, Any]], out_dir: Path) -> Path:
    path = out_dir / "batch_metrics.csv"
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for result in results:
            writer.writerow(result_row(result))
    return path


def write_json(results: Sequence[Dict[str, Any]], log_root: Path,
               out_dir: Path) -> Path:
    path = out_dir / "batch_metrics.json"
    payload = {
        "log_root": str(log_root),
        "runs": [
            {
                "run_name": result["run_dir"].name,
                "run_dir": str(result["run_dir"]),
                "analysis_dir": str(result.get("analysis_dir") or ""),
                "status": result["status"],
                "error": result.get("error", ""),
                "metadata": result.get("metadata"),
                "metrics": result.get("metrics"),
            }
            for result in results
        ],
    }
    path.write_text(
        json.dumps(clean_for_json(payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return path


def write_markdown(results: Sequence[Dict[str, Any]], log_root: Path,
                   out_dir: Path) -> Path:
    path = out_dir / "batch_summary.md"
    ok_count = sum(1 for result in results if result["status"] == "ok")
    failed_count = len(results) - ok_count
    lines = [
        "# flexjoint_vs Batch Analysis",
        "",
        f"- Log root: {log_root}",
        f"- Runs found: {len(results)}",
        f"- Successful: {ok_count}",
        f"- Failed: {failed_count}",
        "- Fast-state metrics use Zf1 in Nm and Zf2 in Nm/s; values are logged "
        "controller-state values without analysis-time rescaling.",
        "",
        "## By Task And Controller",
        "",
    ]

    grouped: Dict[str, Dict[str, List[Dict[str, Any]]]] = {}
    for result in results:
        metadata = result.get("metadata") or {}
        key = metadata.get("task_key") or "unknown task"
        mode = metadata.get("controller_mode") or "unknown_controller"
        grouped.setdefault(key, {}).setdefault(mode, []).append(result)

    for key in sorted(grouped):
        lines.extend([f"### {key}", ""])
        lines.append("| Controller | Runs | Run directories |")
        lines.append("|---|---:|---|")
        for mode in sorted(grouped[key]):
            runs_for_mode = grouped[key][mode]
            names = ", ".join(result["run_dir"].name for result in runs_for_mode)
            lines.append(f"| {mode} | {len(runs_for_mode)} | {names} |")
        lines.append("")

    lines.extend([
        "## Runs",
        "",
        "| Run | Controller | Feature Count | Desired Coords | Status | Exit | Duration (s) | Initial RMS (px) | Final RMS (px) | Convergence (s) | Vision valid | Feedback valid |",
        "|---|---|---:|---|---|---|---:|---:|---:|---:|---:|---:|",
    ])
    for result in results:
        metrics = result.get("metrics") or {}
        metadata = result.get("metadata") or {}
        lines.append(
            "| {run} | {controller} | {feature_count} | `{desired}` | {status} | {exit_reason} | {duration} | {initial} | {final} | {conv} | {vision} | {feedback} |".format(
                run=result["run_dir"].name,
                controller=metadata.get("controller_mode", ""),
                feature_count=metadata.get("feature_count", ""),
                desired=format_desired_coords(metadata.get("desired_coords") or []),
                status=result["status"],
                exit_reason=metrics.get("exit_reason", result.get("error", "")),
                duration=format_metric(metrics.get("duration_s")),
                initial=format_metric(metrics.get("initial_rms_image_error_px")),
                final=format_metric(metrics.get("final_rms_image_error_px")),
                conv=format_metric(metrics.get("convergence_time_s")),
                vision=format_metric(metrics.get("vision_valid_ratio")),
                feedback=format_metric(metrics.get("feedback_valid_ratio")),
            )
        )
    lines.extend([
        "",
        "## Output Files",
        "",
        "- batch_metrics.csv",
        "- batch_metrics.json",
        "- batch_summary.md",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def analyze_one_run(run_dir: Path, paper_style: bool, skip_existing: bool,
                    stop_on_error: bool) -> Dict[str, Any]:
    analysis_dir = run_dir / "analysis"
    metadata = load_run_metadata(run_dir)
    try:
        if not skip_existing or not (analysis_dir / "metrics.json").exists():
            analysis_dir = analyze_run(run_dir, analysis_dir, paper_style)

        run = load_run(run_dir)
        metrics = clean_for_json(compute_metrics(run))
        return {
            "run_dir": run_dir,
            "analysis_dir": analysis_dir,
            "status": "ok",
            "metadata": metadata,
            "metrics": metrics,
        }
    except Exception as exc:
        if stop_on_error:
            raise
        return {
            "run_dir": run_dir,
            "analysis_dir": analysis_dir,
            "status": "failed",
            "error": str(exc),
            "metadata": metadata,
            "metrics": None,
        }


def analyze_log_folder(log_root: str | Path, out: Optional[str | Path],
                       recursive: bool, paper_style: bool,
                       skip_existing: bool, stop_on_error: bool) -> Path:
    log_root_path = resolve_path(log_root)
    if not log_root_path.exists() or not log_root_path.is_dir():
        raise FileNotFoundError(f"Log folder not found: {log_root_path}")

    run_dirs = find_run_dirs(log_root_path, recursive)
    if not run_dirs:
        raise FileNotFoundError(
            f"No run directories with dataFile.txt and run_config.yaml under {log_root_path}"
        )

    out_dir = resolve_path(out, Path.cwd()) if out else log_root_path / "batch_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    results: List[Dict[str, Any]] = []
    for idx, run_dir in enumerate(run_dirs, start=1):
        print(f"[{idx}/{len(run_dirs)}] analyzing {run_dir}")
        result = analyze_one_run(run_dir, paper_style, skip_existing, stop_on_error)
        results.append(result)
        if result["status"] == "failed":
            print(f"  failed: {result['error']}", file=sys.stderr)
        else:
            print(f"  ok: {result['analysis_dir']}")

    write_csv(results, out_dir)
    write_json(results, log_root_path, out_dir)
    write_markdown(results, log_root_path, out_dir)
    return out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze every flexjoint_vs run directory under a log folder."
    )
    parser.add_argument(
        "log_root",
        nargs="?",
        default=None,
        help=(
            "Folder containing run subfolders, usually data/log. If omitted, "
            "a folder picker opens."
        ),
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Batch summary output directory. Defaults to <log_root>/batch_analysis.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search recursively for run folders instead of only direct children.",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Do not regenerate per-run analysis when analysis/metrics.json exists.",
    )
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop at the first failed run instead of recording the failure.",
    )
    parser.add_argument(
        "--paper-style",
        action="store_true",
        dest="paper_style",
        default=True,
        help="Use compact serif styling similar to paper figures (default).",
    )
    parser.add_argument(
        "--no-paper-style",
        action="store_false",
        dest="paper_style",
        help="Use Matplotlib's default sans-serif styling instead.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_root = args.log_root
    if log_root is None:
        try:
            log_root = select_directory_dialog(
                "Select flexjoint_vs log folder", default_run_root()
            )
        except FileDialogError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 2

    try:
        out_dir = analyze_log_folder(
            log_root,
            args.out,
            args.recursive,
            args.paper_style,
            args.skip_existing,
            args.stop_on_error,
        )
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Batch analysis written to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
