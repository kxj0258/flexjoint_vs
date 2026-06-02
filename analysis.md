# 实验数据分析说明

本项目的离线分析脚本会读取每次实验目录中的 `dataFile.txt`、
`run_config.yaml` 和 `run_summary.md`，生成图像误差、特征点轨迹、控制输入、
观测器状态、参数变化和汇总指标。脚本支持 2/3/4 个视觉特征点，会根据
`run_config.yaml` 中的 `vision.feature_count` 或 `vision.desired_coords`
自动确定点数。

## 环境准备

```bash
python3 -m pip install -r requirements.txt
```

需要的 Python 包包括 `numpy`、`pandas`、`matplotlib` 和 `pyyaml`。

如果希望使用独立环境，可以创建 conda 环境：

```bash
conda create -n flexjoint-analysis python=3.12 -y
conda activate flexjoint-analysis
python -m pip install -r requirements.txt
```

验证环境：

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

## 单组实验分析

单组实验目录通常位于 `data/log/<timestamp>`，目录内应包含：

```text
dataFile.txt
run_config.yaml
run_summary.md
```

运行：

```bash
python3 scripts/analyze_run.py --run data/log/<timestamp>
```

如果不传 `--run`，脚本会弹出文件夹选择框，让你手动选择某一组实验目录。

默认输出到 `<run>/analysis/`：

- `fig_image_errors.png/.pdf/.svg`
- `fig_image_trajectories.png/.pdf/.svg`
- `fig_observer_states.png/.pdf/.svg`
- `fig_motor_actual_vs_estimated.png/.pdf/.svg`
- `fig_parameters_theta_rho.png/.pdf/.svg`
- `fig_parameters_theta_rho_estimates_only.png/.pdf/.svg`
- `fig_fast_states.png/.pdf/.svg`
- `fig_control_inputs.png/.pdf/.svg`
- `metrics.json`
- `analysis_summary.md`

指定输出目录：

```bash
python3 scripts/analyze_run.py --run data/log/<timestamp> --out data/analysis/my_run
```

## 批量分析 log 文件夹

如果你已经跑了很多组实验，并且每组实验都是 `data/log/` 下的一个子文件夹，
可以直接批量分析整个 log 文件夹：

```bash
python3 scripts/analyze_log_folder.py data/log
```

不带参数运行时会弹出文件夹选择框：

```bash
python3 scripts/analyze_log_folder.py
```

脚本会扫描所选文件夹下每个包含 `dataFile.txt` 和 `run_config.yaml` 的子目录，
逐个生成单组分析，并在 log 根目录下生成批量汇总：

```text
data/log/batch_analysis/
  batch_summary.md
  batch_metrics.csv
  batch_metrics.json
```

常用选项：

```bash
python3 scripts/analyze_log_folder.py data/log --skip-existing
python3 scripts/analyze_log_folder.py data/log --recursive
python3 scripts/analyze_log_folder.py data/log --out data/log_batch_result
```

- `--skip-existing`：如果某组实验已经有 `analysis/metrics.json`，则不重新画图。
- `--recursive`：递归搜索更深层目录中的实验数据。
- `--out`：指定批量汇总输出目录。

批量汇总会从每组实验的 `run_config.yaml` 中读取并记录：

```yaml
experiment:
  controller_mode: ...
vision:
  feature_count: ...
  desired_coords: [...]
```

因此 `batch_summary.md` 中会告诉你：哪些实验对应同一个视觉任务
`desired_coords`，以及这些实验分别采用了什么控制方法 `controller_mode`。
同样的信息也会写入 `batch_metrics.csv` 和 `batch_metrics.json`，便于后续用
表格软件或 Python 再处理。

## 多组实验对比

如果你希望手动挑选几组实验进行横向对比，可以写一个 manifest 文件，例如
`data/experiments/example_manifest.yaml`：

```yaml
output_dir: comparison
experiments:
  - name: proposed
    label: Proposed controller
    run: ../log/20260520_153401

  - name: no_fast
    label: No fast subsystem
    run: ../log/20260520_160012

  - name: baseline_pd
    label: Baseline PD
    run: ../log/20260520_164233
```

运行：

```bash
python3 scripts/compare_runs.py data/experiments/example_manifest.yaml
```

如果不传 manifest 路径，脚本会弹出文件选择框。默认输出目录为 manifest
所在目录下的 `comparison/`，也可以使用 `--out` 指定：

```bash
python3 scripts/compare_runs.py data/experiments/example_manifest.yaml --out data/experiments/my_comparison
```

`compare_runs.py` 会生成：

- `fig_compare_image_error.png/.pdf/.svg`
- `fig_compare_trajectories.png/.pdf/.svg`
- `fig_compare_control_inputs.png/.pdf/.svg`
- `compare_metrics.json`
- `compare_summary.md`

## 图像误差和收敛指标

图像误差来自 `dataFile.txt` 中的 `img_uN/img_vN` 与 `run_config.yaml` 中的
`vision.desired_coords`。新日志还包含 `img_r1 ... img_rN` 半径列，分析脚本会
在存在这些列时记录半径统计。RMS 图像误差按特征点数量 N 归一化：

```text
sqrt(sum_i((u_i-u_di)^2 + (v_i-v_di)^2) / N)
```

收敛时间使用 `task.image_error_tolerance_px` 作为阈值，寻找 RMS 图像误差稳定
低于该阈值的时间。

## 信号含义假设

观测器状态命名沿用 `controller.cpp` 中的动力学模型：

- `state_obs_0/state_obs_2`：柔性/刚性侧角度和速度估计
- `state_obs_1/state_obs_3`：电机侧角度和速度估计

因此电机实际值和估计值对比图使用：

- `joint_angle_rad` 对比 `state_obs_1`
- `joint_velocity_rad_s` 对比 `state_obs_3`
