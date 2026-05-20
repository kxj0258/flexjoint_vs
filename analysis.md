# Experimental Data Analysis

The analysis scripts turn each `data/log/<timestamp>` run into paper-style
figures, metrics, and Markdown summaries.

## Dependencies

```bash
python3 -m pip install -r requirements.txt
```

Required Python packages are `numpy`, `pandas`, `matplotlib`, and `pyyaml`.

## Conda Environment

If you prefer to keep the analysis tools isolated from the system Python,
create a dedicated conda environment:

```bash
conda create -n flexjoint-analysis python=3.12 -y
conda activate flexjoint-analysis
python -m pip install -r requirements.txt
```

Verify the environment before running the scripts:

```bash
python - <<'PY'
import numpy
import pandas
import matplotlib
import yaml

print("numpy", numpy.__version__)
print("pandas", pandas.__version__)
print("matplotlib", matplotlib.__version__)
print("pyyaml", yaml.__version__)
PY
```

Run the analysis scripts after activating the environment:

```bash
conda activate flexjoint-analysis
python scripts/analyze_run.py --run data/log/20260520_153401 --paper-style
```

To remove the environment later:

```bash
conda deactivate
conda env remove -n flexjoint-analysis
```

## Single Run

```bash
python3 scripts/analyze_run.py --run data/log/20260520_153401 --paper-style
```

By default, outputs are written to `<run>/analysis`:

- `fig_image_errors.png/.pdf`
- `fig_image_trajectories.png/.pdf`
- `fig_observer_states.png/.pdf`
- `fig_motor_actual_vs_estimated.png/.pdf`
- `fig_parameters_theta_rho.png/.pdf`
- `fig_fast_states.png/.pdf`
- `fig_control_inputs.png/.pdf`
- `metrics.json`
- `analysis_summary.md`

The script reads `dataFile.txt` and `run_config.yaml` from the run directory.
Image errors are computed from `vision.desired_coords`, and the convergence
threshold uses `task.image_error_tolerance_px`.

## Multiple Runs

Create a manifest like `data/experiments/example_manifest.yaml`:

```yaml
output_dir: comparison
experiments:
  - name: proposed
    label: Proposed controller
    run: ../log/20260520_153401
  - name: no_fast
    label: No fast subsystem
    run: ../log/20260520_153401
  - name: baseline_pd
    label: Baseline PD
    run: ../log/20260520_153401
```

Then run:

```bash
python3 scripts/compare_runs.py data/experiments/example_manifest.yaml --paper-style
```

The default output directory is `data/experiments/comparison` unless
`output_dir` or `--out` is set.

## Signal Assumption

The observer-state naming follows the dynamic model in `controller.cpp`:
`state_obs_0/state_obs_2` are treated as rigid-link angle/velocity estimates,
and `state_obs_1/state_obs_3` as motor angle/velocity estimates. The direct
motor comparison therefore uses `joint_angle_rad` vs `state_obs_1` and
`joint_velocity_rad_s` vs `state_obs_3`.
