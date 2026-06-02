#!/usr/bin/env python3
"""Compare multiple flexjoint_vs experiment runs.

The manifest-driven workflow is intended for proposed/no-fast/baseline
comparisons like the paper's comparative experiments.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from analyze_run import (
    FileDialogError,
    configure_style,
    load_run,
    compute_metrics,
    clean_for_json,
    default_experiment_root,
    resolve_path,
    select_file_dialog,
    set_time_axis_from_zero,
)


def load_manifest(path: str | Path) -> tuple[Path, Dict[str, Any]]:
    manifest_path = resolve_path(path)
    if not manifest_path.exists():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle) or {}
    if not isinstance(manifest, dict):
        raise ValueError(f"Manifest must be a YAML mapping: {manifest_path}")
    experiments = manifest.get("experiments")
    if not isinstance(experiments, list) or not experiments:
        raise ValueError(
            f"{manifest_path} must contain a non-empty experiments list."
        )
    return manifest_path, manifest


def experiment_run_path(entry: Dict[str, Any], manifest_path: Path) -> Path:
    raw_path = entry.get("run")
    if not raw_path:
        raise ValueError("Each experiment entry must contain a run path.")
    return resolve_path(raw_path, manifest_path.parent)


def experiment_label(entry: Dict[str, Any], fallback: str) -> str:
    return str(entry.get("label") or entry.get("name") or fallback)


def default_output_dir(manifest: Dict[str, Any], manifest_path: Path) -> Path:
    configured = manifest.get("output_dir")
    if configured:
        return resolve_path(configured, manifest_path.parent)
    return manifest_path.parent / "comparison"


def save_figure(fig: plt.Figure, out_dir: Path, stem: str) -> List[Path]:
    paths = []
    for suffix in ("png", "pdf", "svg"):
        path = out_dir / f"{stem}.{suffix}"
        fig.savefig(path, bbox_inches="tight")
        paths.append(path)
    plt.close(fig)
    return paths


def plot_compare_image_error(runs: Sequence[Dict[str, Any]], out_dir: Path) -> List[Path]:
    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    for item in runs:
        run = item["run_data"]
        ax.plot(run.t, run.rms_image_error, label=item["label"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RMS image error (pixel)")
    ax.legend(loc="best")
    set_time_axis_from_zero(
        ax, np.concatenate([item["run_data"].t for item in runs])
    )
    return save_figure(fig, out_dir, "fig_compare_image_error")


def plot_compare_trajectories(runs: Sequence[Dict[str, Any]], out_dir: Path) -> List[Path]:
    fig, ax = plt.subplots(figsize=(6.2, 4.6))
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    line_styles = ["-", "--", ":", "-."]
    markers = ["o", "s", "D", "^"]
    for run_idx, item in enumerate(runs):
        run = item["run_data"]
        color = colors[run_idx % len(colors)] if colors else None
        for point_idx in range(run.feature_count):
            u = run.df[f"img_u{point_idx + 1}"].to_numpy(dtype=float).copy()
            v = run.df[f"img_v{point_idx + 1}"].to_numpy(dtype=float).copy()
            u[~run.vision_mask] = np.nan
            v[~run.vision_mask] = np.nan
            ax.plot(
                u,
                v,
                label=f"{item['label']} y{point_idx + 1}",
                color=color,
                linestyle=line_styles[point_idx % len(line_styles)],
            )
            ax.scatter(
                run.desired[2 * point_idx],
                run.desired[2 * point_idx + 1],
                color=color or "0.1",
                marker=markers[point_idx % len(markers)],
                s=48,
                linewidths=1.5,
                facecolors="none",
                label=f"{item['label']} y{point_idx + 1} target",
            )
    ax.set_xlabel("u (pixel)")
    ax.set_ylabel("v (pixel)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=7)
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_compare_trajectories")


def plot_compare_control_inputs(runs: Sequence[Dict[str, Any]], out_dir: Path) -> List[Path]:
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
    for item in runs:
        run = item["run_data"]
        axes[0].plot(run.t, run.df["velocity_command_rad_s"], label=item["label"])
        axes[1].plot(run.t, run.df["state_tau"], label=item["label"])
    axes[0].set_ylabel("Velocity command (rad/s)")
    axes[0].legend(loc="best")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel(r"$\tau$")
    axes[1].legend(loc="best")
    set_time_axis_from_zero(
        axes, np.concatenate([item["run_data"].t for item in runs])
    )
    return save_figure(fig, out_dir, "fig_compare_control_inputs")


def format_metric(value: Any, precision: int = 4) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{precision}g}"
    return str(value)


def write_summary(
    runs: Sequence[Dict[str, Any]], figure_paths: Sequence[Path], out_dir: Path
) -> Path:
    path = out_dir / "compare_summary.md"
    lines = [
        "# flexjoint_vs Experiment Comparison",
        "",
        "## Metrics",
        "",
        "Fast-state metrics use Zf1 in Nm and Zf2 in Nm/s; values are logged controller-state values without analysis-time rescaling.",
        "",
        "| Experiment | Exit | Duration (s) | Initial RMS (px) | Final RMS (px) | Convergence (s) | Max abs u_cmd (rad/s) | Mean abs u_cmd (rad/s) | RMS Zf1 (Nm) | Max abs Zf1 (Nm) | RMS Zf2 (Nm/s) | Max abs Zf2 (Nm/s) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in runs:
        metrics = item["metrics"]
        lines.append(
            "| {label} | {exit_reason} | {duration} | {initial} | {final} | {conv} | {max_cmd} | {mean_cmd} | {rms_zf1} | {max_zf1} | {rms_zf2} | {max_zf2} |".format(
                label=item["label"],
                exit_reason=metrics.get("exit_reason", "unknown"),
                duration=format_metric(metrics.get("duration_s")),
                initial=format_metric(metrics.get("initial_rms_image_error_px")),
                final=format_metric(metrics.get("final_rms_image_error_px")),
                conv=format_metric(metrics.get("convergence_time_s")),
                max_cmd=format_metric(metrics.get("max_abs_velocity_command_rad_s")),
                mean_cmd=format_metric(metrics.get("mean_abs_velocity_command_rad_s")),
                rms_zf1=format_metric(metrics.get("rms_zf1_nm")),
                max_zf1=format_metric(metrics.get("max_abs_zf1_nm")),
                rms_zf2=format_metric(metrics.get("rms_zf2_nm_s")),
                max_zf2=format_metric(metrics.get("max_abs_zf2_nm_s")),
            )
        )
    lines.extend(["", "## Figures", ""])
    for figure in figure_paths:
        lines.append(f"- {figure.name}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def compare_runs(
    manifest_arg: str | Path,
    out_arg: Optional[str | Path],
    paper_style: bool,
) -> Path:
    configure_style(paper_style)
    manifest_path, manifest = load_manifest(manifest_arg)
    out_dir = resolve_path(out_arg, Path.cwd()) if out_arg else default_output_dir(
        manifest, manifest_path
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    run_items: List[Dict[str, Any]] = []
    for idx, entry in enumerate(manifest["experiments"], start=1):
        if not isinstance(entry, dict):
            raise ValueError("Each experiments item must be a mapping.")
        run_data = load_run(experiment_run_path(entry, manifest_path))
        metrics = clean_for_json(compute_metrics(run_data))
        run_items.append(
            {
                "label": experiment_label(entry, f"run {idx}"),
                "entry": entry,
                "run_data": run_data,
                "metrics": metrics,
            }
        )

    figure_paths: List[Path] = []
    figure_paths.extend(plot_compare_image_error(run_items, out_dir))
    figure_paths.extend(plot_compare_trajectories(run_items, out_dir))
    figure_paths.extend(plot_compare_control_inputs(run_items, out_dir))

    metrics_path = out_dir / "compare_metrics.json"
    metrics_path.write_text(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "experiments": [
                    {
                        "label": item["label"],
                        "run_dir": str(item["run_data"].run_dir),
                        "metrics": item["metrics"],
                    }
                    for item in run_items
                ],
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    write_summary(run_items, figure_paths, out_dir)
    return out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare multiple flexjoint_vs experiment runs from a YAML manifest."
    )
    parser.add_argument(
        "manifest",
        nargs="?",
        default=None,
        help=(
            "YAML manifest containing experiments. If omitted, a file picker opens "
            "so you can select a manifest."
        ),
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output directory. Defaults to manifest output_dir or <manifest-dir>/comparison.",
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
    manifest = args.manifest
    if manifest is None:
        try:
            manifest = select_file_dialog(
                "Select flexjoint_vs comparison manifest",
                default_experiment_root(),
                (
                    ("YAML files", "*.yaml *.yml"),
                    ("All files", "*"),
                ),
            )
        except FileDialogError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 2

    out_dir = compare_runs(manifest, args.out, args.paper_style)
    print(f"Comparison written to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
