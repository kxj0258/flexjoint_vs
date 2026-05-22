#!/usr/bin/env python3
"""Offline analysis for a flexjoint_vs run directory.

The figures are modeled after the Experimental Validations section of the
paper, using the signals already logged by src/main.cpp.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


THETA_COLUMNS = [f"state_theta_{i}" for i in range(4)]
RHO_COLUMNS = [f"state_rho_{i}" for i in range(5)]
OBS_COLUMNS = [f"state_obs_{i}" for i in range(4)]
CONTROL_COLUMNS = [
    "velocity_command_rad_s",
    "joint_cal_rad_s",
    "state_tau",
    "state_tau_s",
    "state_tau_f_c",
]

REQUIRED_COLUMNS = [
    "frame_index",
    "elapsed_time_s",
    "joint_angle_rad",
    "joint_velocity_rad_s",
    "feedback_ok",
    "vision_ok",
    *THETA_COLUMNS,
    *RHO_COLUMNS,
    *OBS_COLUMNS,
    *CONTROL_COLUMNS,
]


class FileDialogError(RuntimeError):
    """Raised when an interactive file/folder picker cannot provide a path."""


@dataclass
class RunData:
    run_dir: Path
    data_file: Path
    config_file: Path
    summary_file: Optional[Path]
    df: pd.DataFrame
    config: Dict[str, Any]
    desired: np.ndarray
    feature_count: int
    image_columns: List[str]
    image_tolerance_px: float
    exit_reason: str
    t: np.ndarray
    image_errors: np.ndarray
    point_error_norms: np.ndarray
    rms_image_error: np.ndarray
    vision_mask: np.ndarray
    feedback_mask: np.ndarray


def resolve_path(path: str | Path, base: Optional[Path] = None) -> Path:
    path = Path(path).expanduser()
    if path.is_absolute():
        return path.resolve()
    if base is None:
        base = Path.cwd()
    return (base / path).resolve()


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_run_root() -> Path:
    candidate = project_root() / "data" / "log"
    return candidate if candidate.exists() else Path.cwd()


def default_experiment_root() -> Path:
    candidate = project_root() / "data" / "experiments"
    return candidate if candidate.exists() else Path.cwd()


def select_directory_dialog(title: str, initialdir: Optional[Path] = None) -> Path:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception as exc:  # pragma: no cover - depends on local Python build
        raise FileDialogError(
            "Tkinter is not available; pass the path explicitly with --run."
        ) from exc

    root = tk.Tk()
    root.withdraw()
    root.update()
    try:
        selected = filedialog.askdirectory(
            title=title,
            initialdir=str(initialdir or Path.cwd()),
            mustexist=True,
        )
    except Exception as exc:  # pragma: no cover - depends on desktop session
        raise FileDialogError(
            "Could not open the folder picker; pass the path explicitly with --run."
        ) from exc
    finally:
        root.destroy()

    if not selected:
        raise FileDialogError("No run directory selected.")
    return Path(selected).expanduser().resolve()


def select_file_dialog(
    title: str,
    initialdir: Optional[Path] = None,
    filetypes: Optional[Sequence[Tuple[str, str]]] = None,
) -> Path:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception as exc:  # pragma: no cover - depends on local Python build
        raise FileDialogError(
            "Tkinter is not available; pass the file path explicitly."
        ) from exc

    root = tk.Tk()
    root.withdraw()
    root.update()
    try:
        selected = filedialog.askopenfilename(
            title=title,
            initialdir=str(initialdir or Path.cwd()),
            filetypes=filetypes or (("All files", "*"),),
        )
    except Exception as exc:  # pragma: no cover - depends on desktop session
        raise FileDialogError(
            "Could not open the file picker; pass the file path explicitly."
        ) from exc
    finally:
        root.destroy()

    if not selected:
        raise FileDialogError("No file selected.")
    return Path(selected).expanduser().resolve()


def nested_get(mapping: Dict[str, Any], keys: Sequence[str], default: Any = None) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def require_columns(df: pd.DataFrame, columns: Iterable[str], data_file: Path) -> None:
    missing = [column for column in columns if column not in df.columns]
    if missing:
        raise ValueError(
            f"{data_file} is missing required columns: {', '.join(missing)}"
        )


def numeric_column(df: pd.DataFrame, column: str, data_file: Path) -> None:
    try:
        df[column] = pd.to_numeric(df[column])
    except Exception as exc:  # pragma: no cover - pandas exception text is enough
        raise ValueError(f"{data_file} column {column!r} is not numeric") from exc


def bool_array(series: pd.Series) -> np.ndarray:
    if pd.api.types.is_bool_dtype(series):
        return series.to_numpy(dtype=bool)
    values = pd.to_numeric(series, errors="coerce").fillna(0)
    return values.to_numpy(dtype=float) != 0.0


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return loaded


def feature_count_from_config(config: Dict[str, Any], desired: Sequence[Any],
                              config_file: Path) -> int:
    configured = nested_get(config, ["vision", "feature_count"])
    if configured is not None:
        count = int(configured)
        if count not in (3, 4):
            raise ValueError(
                f"{config_file} vision.feature_count must be 3 or 4, got {count}."
            )
    else:
        count = len(desired) // 2
    if count not in (3, 4):
        raise ValueError(
            f"{config_file} desired_coords imply {count} feature points; "
            "only 3 or 4 are supported."
        )
    if len(desired) != 2 * count:
        raise ValueError(
            f"{config_file} vision.desired_coords must contain {2 * count} "
            f"numbers for feature_count={count}, got {len(desired)}."
        )
    return count


def image_columns_for_count(feature_count: int) -> List[str]:
    columns: List[str] = []
    for point_idx in range(feature_count):
        columns.extend([f"img_u{point_idx + 1}", f"img_v{point_idx + 1}"])
    return columns


def desired_from_config(config: Dict[str, Any], config_file: Path) -> tuple[np.ndarray, int]:
    desired = nested_get(config, ["vision", "desired_coords"])
    if desired is None:
        raise ValueError(
            f"{config_file} does not contain vision.desired_coords; "
            "cannot compute image errors."
        )
    feature_count = feature_count_from_config(config, desired, config_file)
    return np.asarray(desired, dtype=float), feature_count


def tolerance_from_config(config: Dict[str, Any]) -> float:
    value = nested_get(config, ["task", "image_error_tolerance_px"], 5.0)
    return float(value)


def parse_exit_reason(summary_file: Optional[Path]) -> str:
    if summary_file is None or not summary_file.exists():
        return "unknown"
    text = summary_file.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"^- Exit reason:\s*(.+?)\s*$", text, re.MULTILINE)
    return match.group(1).strip() if match else "unknown"


def load_run(run_dir: str | Path) -> RunData:
    run_dir = resolve_path(run_dir)
    data_file = run_dir / "dataFile.txt"
    config_file = run_dir / "run_config.yaml"
    summary_file = run_dir / "run_summary.md"
    if not summary_file.exists():
        summary_file = None
    if not data_file.exists():
        raise FileNotFoundError(f"Data log not found: {data_file}")

    df = pd.read_csv(data_file)
    config = load_yaml(config_file)
    desired, feature_count = desired_from_config(config, config_file)
    image_columns = image_columns_for_count(feature_count)

    require_columns(df, [*REQUIRED_COLUMNS, *image_columns], data_file)
    for column in [*REQUIRED_COLUMNS, *image_columns]:
        if column != "safety_stop_reason":
            numeric_column(df, column, data_file)

    image_tolerance_px = tolerance_from_config(config)

    elapsed = df["elapsed_time_s"].to_numpy(dtype=float)
    if len(elapsed) == 0:
        raise ValueError(f"{data_file} contains no samples")
    t = elapsed - elapsed[0]

    image_points = df[image_columns].to_numpy(dtype=float)
    image_errors = image_points - desired.reshape(1, 2 * feature_count)
    point_error_norms = np.sqrt(
        image_errors[:, 0::2] ** 2 + image_errors[:, 1::2] ** 2
    )
    rms_image_error = np.sqrt(
        np.sum(point_error_norms**2, axis=1) / float(feature_count)
    )

    return RunData(
        run_dir=run_dir,
        data_file=data_file,
        config_file=config_file,
        summary_file=summary_file,
        df=df,
        config=config,
        desired=desired,
        feature_count=feature_count,
        image_columns=image_columns,
        image_tolerance_px=image_tolerance_px,
        exit_reason=parse_exit_reason(summary_file),
        t=t,
        image_errors=image_errors,
        point_error_norms=point_error_norms,
        rms_image_error=rms_image_error,
        vision_mask=bool_array(df["vision_ok"]),
        feedback_mask=bool_array(df["feedback_ok"]),
    )


def first_sustained_time(
    t: np.ndarray, values: np.ndarray, threshold: float, mask: np.ndarray
) -> Optional[float]:
    valid_idx = np.flatnonzero(mask & np.isfinite(values))
    if valid_idx.size == 0:
        return None
    valid_values = values[valid_idx]
    valid_t = t[valid_idx]
    below = valid_values <= threshold
    if not below[-1]:
        return None
    suffix_all = np.ones_like(below, dtype=bool)
    running = True
    for i in range(len(below) - 1, -1, -1):
        running = running and bool(below[i])
        suffix_all[i] = running
    indices = np.flatnonzero(suffix_all)
    return float(valid_t[indices[0]]) if indices.size else None


def safe_max_abs(values: np.ndarray) -> Optional[float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.max(np.abs(finite)))


def safe_mean_abs(values: np.ndarray) -> Optional[float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.mean(np.abs(finite)))


def compute_metrics(run: RunData) -> Dict[str, Any]:
    valid_image_idx = np.flatnonzero(run.vision_mask & np.isfinite(run.rms_image_error))
    first_idx = int(valid_image_idx[0]) if valid_image_idx.size else None
    last_idx = int(valid_image_idx[-1]) if valid_image_idx.size else None

    feedback_idx = np.flatnonzero(run.feedback_mask)
    if feedback_idx.size:
        angle_error = (
            run.df["joint_angle_rad"].to_numpy(dtype=float)[feedback_idx]
            - run.df["state_obs_1"].to_numpy(dtype=float)[feedback_idx]
        )
        velocity_error = (
            run.df["joint_velocity_rad_s"].to_numpy(dtype=float)[feedback_idx]
            - run.df["state_obs_3"].to_numpy(dtype=float)[feedback_idx]
        )
    else:
        angle_error = np.asarray([], dtype=float)
        velocity_error = np.asarray([], dtype=float)

    velocity_command = run.df["velocity_command_rad_s"].to_numpy(dtype=float)
    fast_angle = run.df["state_obs_0"].to_numpy(dtype=float) - run.df[
        "state_obs_1"
    ].to_numpy(dtype=float)
    fast_velocity = run.df["state_obs_2"].to_numpy(dtype=float) - run.df[
        "state_obs_3"
    ].to_numpy(dtype=float)
    fast_norm = np.sqrt(fast_angle**2 + fast_velocity**2)
    duration = float(run.t[-1] - run.t[0]) if len(run.t) else 0.0
    final_point_errors = (
        run.point_error_norms[last_idx].tolist() if last_idx is not None else None
    )

    return {
        "run_dir": str(run.run_dir),
        "sample_count": int(len(run.df)),
        "duration_s": duration,
        "exit_reason": run.exit_reason,
        "image_error_tolerance_px": float(run.image_tolerance_px),
        "initial_rms_image_error_px": (
            float(run.rms_image_error[first_idx]) if first_idx is not None else None
        ),
        "final_rms_image_error_px": (
            float(run.rms_image_error[last_idx]) if last_idx is not None else None
        ),
        "final_point_error_norms_px": final_point_errors,
        "convergence_time_s": first_sustained_time(
            run.t, run.rms_image_error, run.image_tolerance_px, run.vision_mask
        ),
        "max_abs_velocity_command_rad_s": safe_max_abs(velocity_command),
        "mean_abs_velocity_command_rad_s": safe_mean_abs(velocity_command),
        "rms_fast_state_norm": (
            float(np.sqrt(np.mean(fast_norm[np.isfinite(fast_norm)] ** 2)))
            if np.any(np.isfinite(fast_norm))
            else None
        ),
        "max_fast_state_norm": safe_max_abs(fast_norm),
        "max_abs_motor_angle_estimation_error_rad": safe_max_abs(angle_error),
        "max_abs_motor_velocity_estimation_error_rad_s": safe_max_abs(velocity_error),
        "vision_valid_ratio": float(np.mean(run.vision_mask)),
        "feedback_valid_ratio": float(np.mean(run.feedback_mask)),
    }


def clean_for_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: clean_for_json(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_for_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return clean_for_json(value.tolist())
    if isinstance(value, (np.floating, float)):
        if not math.isfinite(float(value)):
            return None
        return float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def configure_style(paper_style: bool) -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 120,
            "savefig.dpi": 300,
            "axes.grid": True,
            "grid.color": "0.85",
            "grid.linewidth": 0.7,
            "axes.linewidth": 0.8,
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "legend.fontsize": 8,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
        }
    )
    if paper_style:
        plt.rcParams.update(
            {
                "font.family": "serif",
                "lines.linewidth": 1.1,
                "figure.figsize": (6.0, 3.4),
            }
        )


def save_figure(fig: plt.Figure, out_dir: Path, stem: str) -> List[Path]:
    paths = []
    for suffix in ("png", "pdf"):
        path = out_dir / f"{stem}.{suffix}"
        fig.savefig(path, bbox_inches="tight")
        paths.append(path)
    plt.close(fig)
    return paths


def masked(values: np.ndarray, mask: np.ndarray) -> np.ndarray:
    out = values.astype(float).copy()
    out[~mask] = np.nan
    return out


def plot_image_errors(run: RunData, out_dir: Path) -> List[Path]:
    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    labels = [
        label
        for point_idx in range(run.feature_count)
        for label in (rf"$\Delta u_{point_idx + 1}$",
                      rf"$\Delta v_{point_idx + 1}$")
    ]
    line_styles = ["-", "--", "-.", ":"]
    colors = ["tab:red", "tab:red", "tab:blue", "tab:blue",
              "0.15", "0.15", "tab:green", "tab:green"]
    for i, label in enumerate(labels):
        ax.plot(
            run.t,
            masked(run.image_errors[:, i], run.vision_mask),
            linestyle=line_styles[i % len(line_styles)],
            color=colors[i],
            label=label,
        )
    ax.axhline(0.0, color="0.35", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (pixel)")
    ax.legend(ncol=3, loc="best")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_image_errors")


def plot_image_trajectories(run: RunData, out_dir: Path) -> List[Path]:
    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    point_styles = [
        ("red", "-", "o"),
        ("blue", "--", "s"),
        ("black", ":", "D"),
        ("green", "-.", "^"),
    ]
    for point_idx in range(run.feature_count):
        color, linestyle, marker = point_styles[point_idx % len(point_styles)]
        traj_label = rf"$y_{point_idx + 1}$"
        target_label = rf"$y_{{{point_idx + 1}d}}$"
        u = masked(
            run.df[f"img_u{point_idx + 1}"].to_numpy(dtype=float),
            run.vision_mask,
        )
        v = masked(
            run.df[f"img_v{point_idx + 1}"].to_numpy(dtype=float),
            run.vision_mask,
        )
        ax.plot(u, v, color=color, linestyle=linestyle, label=traj_label)
        ax.plot(
            run.desired[2 * point_idx],
            run.desired[2 * point_idx + 1],
            color=color,
            linestyle="None",
            marker=marker,
            markerfacecolor="none",
            markeredgewidth=1.4,
            markersize=5.5,
            label=target_label,
        )
    ax.set_xlabel("u (pixel)")
    ax.set_ylabel("v (pixel)")
    ax.set_xlim(auto=True)
    ax.set_ylim(auto=True)
    ax.legend(loc="best")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_image_trajectories")


def plot_observer_states(run: RunData, out_dir: Path) -> List[Path]:
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
    axes[0].plot(run.t, run.df["state_obs_0"], label="rigid-link angle estimate")
    axes[0].plot(run.t, run.df["state_obs_1"], "--", label="motor angle estimate")
    axes[0].set_ylabel("Angle (rad)")
    axes[0].legend(loc="best")
    axes[1].plot(run.t, run.df["state_obs_2"], label="rigid-link velocity estimate")
    axes[1].plot(run.t, run.df["state_obs_3"], "--", label="motor velocity estimate")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity (rad/s)")
    axes[1].legend(loc="best")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_observer_states")


def plot_motor_actual_vs_estimated(run: RunData, out_dir: Path) -> List[Path]:
    fig, axes = plt.subplots(2, 2, figsize=(8.8, 5.6), sharex="col")
    angle_actual = masked(run.df["joint_angle_rad"].to_numpy(dtype=float), run.feedback_mask)
    vel_actual = masked(run.df["joint_velocity_rad_s"].to_numpy(dtype=float), run.feedback_mask)
    angle_est = run.df["state_obs_1"].to_numpy(dtype=float)
    vel_est = run.df["state_obs_3"].to_numpy(dtype=float)

    axes[0, 0].plot(run.t, angle_actual, label="actual motor angle")
    axes[0, 0].plot(run.t, angle_est, "--", label="estimated motor angle")
    axes[0, 0].set_ylabel("Angle (rad)")
    axes[0, 0].legend(loc="best")

    axes[1, 0].plot(run.t, angle_actual - angle_est, color="0.15")
    axes[1, 0].axhline(0.0, color="0.45", linewidth=0.8)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Angle error (rad)")

    axes[0, 1].plot(run.t, vel_actual, label="actual motor velocity")
    axes[0, 1].plot(run.t, vel_est, "--", label="estimated motor velocity")
    axes[0, 1].set_ylabel("Velocity (rad/s)")
    axes[0, 1].legend(loc="best")

    axes[1, 1].plot(run.t, vel_actual - vel_est, color="0.15")
    axes[1, 1].axhline(0.0, color="0.45", linewidth=0.8)
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Velocity error (rad/s)")

    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_motor_actual_vs_estimated")


def plot_parameters(run: RunData, out_dir: Path) -> List[Path]:
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    for i, column in enumerate(THETA_COLUMNS):
        axes[0].plot(run.t, run.df[column], label=rf"$\theta_{i + 1}$")
    axes[0].set_ylabel("Camera parameter estimate")
    axes[0].legend(ncol=4, loc="best")

    for i, column in enumerate(RHO_COLUMNS):
        axes[1].plot(run.t, run.df[column], label=rf"$\rho_{i + 1}$")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Unknown parameter estimate")
    axes[1].ticklabel_format(axis="y", style="sci", scilimits=(-3, 3))
    axes[1].legend(ncol=5, loc="best")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_parameters_theta_rho")


def plot_fast_states(run: RunData, out_dir: Path) -> List[Path]:
    z_angle = run.df["state_obs_0"].to_numpy(dtype=float) - run.df[
        "state_obs_1"
    ].to_numpy(dtype=float)
    z_velocity = run.df["state_obs_2"].to_numpy(dtype=float) - run.df[
        "state_obs_3"
    ].to_numpy(dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.4), sharex=True)
    axes[0].plot(run.t, z_angle, color="0.1")
    axes[0].axhline(0.0, color="0.45", linewidth=0.8)
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel(r"$z_{f,angle}$ (rad)")
    axes[1].plot(run.t, z_velocity, color="0.1")
    axes[1].axhline(0.0, color="0.45", linewidth=0.8)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel(r"$z_{f,velocity}$ (rad/s)")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_fast_states")


def plot_control_inputs(run: RunData, out_dir: Path) -> List[Path]:
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
    axes[0].plot(run.t, run.df["velocity_command_rad_s"], label="sent velocity command")
    axes[0].plot(run.t, run.df["joint_cal_rad_s"], "--", label="raw controller command")
    axes[0].set_ylabel("Velocity command (rad/s)")
    axes[0].legend(loc="best")

    axes[1].plot(run.t, run.df["state_tau"], label=r"$\tau$")
    axes[1].plot(run.t, run.df["state_tau_s"], "--", label=r"$\tau_s$")
    axes[1].plot(run.t, run.df["state_tau_f_c"], "-.", label=r"$\tau_f$")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Controller internal input")
    axes[1].legend(loc="best")
    fig.tight_layout()
    return save_figure(fig, out_dir, "fig_control_inputs")


def format_metric(value: Any, precision: int = 4) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{precision}g}"
    if isinstance(value, list):
        return ", ".join(format_metric(item, precision) for item in value)
    return str(value)


def write_summary(
    run: RunData, metrics: Dict[str, Any], figure_paths: Sequence[Path], out_dir: Path
) -> Path:
    summary_path = out_dir / "analysis_summary.md"
    assumption = (
        "Observer-state naming follows the dynamic model in controller.cpp: "
        "state_obs_0/state_obs_2 are treated as rigid-link angle/velocity "
        "estimates, and state_obs_1/state_obs_3 as motor angle/velocity "
        "estimates. The direct motor comparison therefore uses "
        "joint_angle_rad vs state_obs_1 and joint_velocity_rad_s vs state_obs_3."
    )
    lines = [
        "# flexjoint_vs Analysis Summary",
        "",
        f"- Run directory: {run.run_dir}",
        f"- Data log: {run.data_file}",
        f"- Run config: {run.config_file}",
        f"- Exit reason: {metrics['exit_reason']}",
        "",
        "## Assumption",
        "",
        assumption,
        "",
        "## Metrics",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Samples | {metrics['sample_count']} |",
        f"| Duration (s) | {format_metric(metrics['duration_s'])} |",
        f"| Image tolerance (px) | {format_metric(metrics['image_error_tolerance_px'])} |",
        f"| Initial RMS image error (px) | {format_metric(metrics['initial_rms_image_error_px'])} |",
        f"| Final RMS image error (px) | {format_metric(metrics['final_rms_image_error_px'])} |",
        f"| Final feature error norms (px) | {format_metric(metrics['final_point_error_norms_px'])} |",
        f"| Convergence time (s) | {format_metric(metrics['convergence_time_s'])} |",
        f"| Max abs velocity command (rad/s) | {format_metric(metrics['max_abs_velocity_command_rad_s'])} |",
        f"| Mean abs velocity command (rad/s) | {format_metric(metrics['mean_abs_velocity_command_rad_s'])} |",
        f"| RMS fast-state norm | {format_metric(metrics['rms_fast_state_norm'])} |",
        f"| Max fast-state norm | {format_metric(metrics['max_fast_state_norm'])} |",
        f"| Max abs motor angle estimation error (rad) | {format_metric(metrics['max_abs_motor_angle_estimation_error_rad'])} |",
        f"| Max abs motor velocity estimation error (rad/s) | {format_metric(metrics['max_abs_motor_velocity_estimation_error_rad_s'])} |",
        f"| Vision valid ratio | {format_metric(metrics['vision_valid_ratio'])} |",
        f"| Feedback valid ratio | {format_metric(metrics['feedback_valid_ratio'])} |",
        "",
        "## Figures",
        "",
    ]
    for path in figure_paths:
        lines.append(f"- {path.name}")
    lines.append("")
    summary_path.write_text("\n".join(lines), encoding="utf-8")
    return summary_path


def analyze_run(run_dir: str | Path, out: Optional[str | Path], paper_style: bool) -> Path:
    configure_style(paper_style)
    run = load_run(run_dir)
    out_dir = resolve_path(out, Path.cwd()) if out else run.run_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    figure_paths: List[Path] = []
    figure_paths.extend(plot_image_errors(run, out_dir))
    figure_paths.extend(plot_image_trajectories(run, out_dir))
    figure_paths.extend(plot_observer_states(run, out_dir))
    figure_paths.extend(plot_motor_actual_vs_estimated(run, out_dir))
    figure_paths.extend(plot_parameters(run, out_dir))
    figure_paths.extend(plot_fast_states(run, out_dir))
    figure_paths.extend(plot_control_inputs(run, out_dir))

    metrics = clean_for_json(compute_metrics(run))
    metrics_path = out_dir / "metrics.json"
    metrics_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_summary(run, metrics, figure_paths, out_dir)
    return out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate paper-style figures and metrics for one flexjoint_vs run."
    )
    parser.add_argument(
        "--run",
        default=None,
        help=(
            "Path to data/log/<timestamp>. If omitted, a folder picker opens "
            "so you can select the run directory."
        ),
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output directory. Defaults to <run>/analysis.",
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
    run_dir = args.run
    if run_dir is None:
        try:
            run_dir = select_directory_dialog(
                "Select flexjoint_vs run directory", default_run_root()
            )
        except FileDialogError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 2

    out_dir = analyze_run(run_dir, args.out, args.paper_style)
    print(f"Analysis written to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
