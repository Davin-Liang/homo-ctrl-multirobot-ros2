#!/usr/bin/env python3
"""Compute auditable no-feedforward sensitivity statistics from real trials."""

import argparse
import csv
import math
from pathlib import Path
import statistics

import yaml


TAIL_WINDOW_S = 10.0
METRIC_KEYS = (
    "mean_abs_distance_error_m",
    "rms_distance_error_m",
    "tail_mean_abs_distance_error_m",
    "tail_distance_error_std_m",
    "mean_relative_velocity_error_mps",
    "tail_mean_relative_velocity_error_mps",
)
REQUIRED_COLUMNS = (
    "time_s", "distance_m", "leader_vx_map_ms", "leader_vy_map_ms",
    "follower_vx_map_ms", "follower_vy_map_ms",
)
TRIALS = {
    "齐次度": {
        "-1.0": (
            "6d_artstein_no_forw_vs_forw_20260922_112130",
            "6d_artstein_no_forw_vs_forw_20260922_112637",
            "6d_artstein_no_forw_vs_forw_20260922_113126",
        ),
        "-0.9": (
            "6d_artstein_nu-0.9(no_fora)_20260922_161938",
            "6d_artstein_nu-0.9(no_fora)_20260922_162215",
            "6d_artstein_nu-0.9(no_fora)_20260922_162535",
        ),
        "-0.8": (
            "6d_artstein_nu-0.8(no_fora)_20260922_162911",
            "6d_artstein_nu-0.8(no_fora)_20260922_163147",
            "6d_artstein_nu-0.8(no_fora)_20260922_163348",
        ),
        "-0.7": (
            "6d_artstein_nu-0.7(no_fora)_20260922_163733",
            "6d_artstein_nu-0.7(no_fora)_20260922_164127",
            "6d_artstein_nu-0.7(no_fora)_20260922_164349",
        ),
        "-0.6": (
            "6d_artstein_nu-0.6(no_fora)_20260922_165110",
            "6d_artstein_nu-0.6(no_fora)_20260922_165453",
            "6d_artstein_nu-0.6(no_fora)_20260922_165759",
        ),
        "-0.5": (
            "6d_artstein_nu-0.5(no_fora)_20260922_170125",
            "6d_artstein_nu-0.5(no_fora)_20260922_170509",
            "6d_artstein_nu-0.5(no_fora)_20260922_170816",
        ),
    },
    "控制频率 (Hz)": {
        "10": (
            "6d_artstein_10hz(no_fora)_20260922_160335",
            "6d_artstein_10hz(no_fora)_20260922_160621",
            "6d_artstein_10hz(no_fora)_20260922_161008",
        ),
        "20": (
            "6d_artstein_no_forw_vs_forw_20260922_112130",
            "6d_artstein_no_forw_vs_forw_20260922_112637",
            "6d_artstein_no_forw_vs_forw_20260922_113126",
        ),
        "25": (
            "6d_artstein_25hz(no_fora)_20260922_154002",
            "6d_artstein_25hz(no_fora)_20260922_154256",
            "6d_artstein_25hz(no_fora)_20260922_154540",
        ),
        "40": (
            "6d_artstein_40hz(no_fora)_20260922_155158",
            "6d_artstein_40hz(no_fora)_20260922_155542",
            "6d_artstein_40hz(no_fora)_20260922_155808",
        ),
    },
}


def compute_trial_metrics(rows, radius_m):
    """Compute the six agreed metrics for one trajectory."""
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("ideal radius must be a positive finite number")
    if not rows or rows[-1]["time_s"] - rows[0]["time_s"] < TAIL_WINDOW_S:
        raise ValueError("trajectory duration must be at least 10 s")
    errors = [row["distance_m"] - radius_m for row in rows]
    velocity_errors = [math.hypot(row["leader_vx_map_ms"] - row["follower_vx_map_ms"],
                                  row["leader_vy_map_ms"] - row["follower_vy_map_ms"])
                       for row in rows]
    tail_start = rows[-1]["time_s"] - TAIL_WINDOW_S
    tail_errors = [error for row, error in zip(rows, errors) if row["time_s"] >= tail_start]
    tail_velocity_errors = [error for row, error in zip(rows, velocity_errors)
                            if row["time_s"] >= tail_start]
    return {
        "mean_abs_distance_error_m": statistics.mean(map(abs, errors)),
        "rms_distance_error_m": math.sqrt(statistics.mean(error ** 2 for error in errors)),
        "tail_mean_abs_distance_error_m": statistics.mean(map(abs, tail_errors)),
        "tail_distance_error_std_m": statistics.pstdev(tail_errors),
        "mean_relative_velocity_error_mps": statistics.mean(velocity_errors),
        "tail_mean_relative_velocity_error_mps": statistics.mean(tail_velocity_errors),
    }


def aggregate_trials(trials):
    """Return arithmetic mean and sample standard deviation per metric."""
    if len(trials) != 3:
        raise ValueError("each group must contain exactly three trials")
    return {key: (statistics.mean(trial[key] for trial in trials),
                  statistics.stdev(trial[key] for trial in trials))
            for key in METRIC_KEYS}


def load_trial(directory, expected_parameter, group_name):
    """Load and validate one trajectory directory."""
    try:
        metadata = yaml.safe_load((directory / "metadata.yaml").read_text(encoding="utf-8"))
        with (directory / "raw.csv").open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None or any(name not in reader.fieldnames for name in REQUIRED_COLUMNS):
                raise ValueError("raw.csv is missing required columns")
            rows = []
            for row in reader:
                values = {name: float(row[name]) for name in REQUIRED_COLUMNS}
                if not all(math.isfinite(value) for value in values.values()):
                    raise ValueError("raw.csv contains a non-finite value")
                rows.append(values)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"{directory}: {error}") from error
    if not isinstance(metadata, dict) or metadata.get("mode") != "real":
        raise ValueError(f"{directory}: metadata mode must be real")
    recording = metadata.get("recording", {})
    parameters = metadata.get("controller_parameters", {})
    if parameters.get("enable_leader_cmd_feedforward") is not False:
        raise ValueError(f"{directory}: leader command feedforward must be false")
    radius_m = recording.get("ideal_radius_m")
    if group_name == "齐次度" and parameters.get("hpc_nu_override") != float(expected_parameter):
        raise ValueError(f"{directory}: hpc_nu_override does not match {expected_parameter}")
    if group_name == "控制频率 (Hz)" and parameters.get("control_rate") != float(expected_parameter):
        raise ValueError(f"{directory}: control_rate does not match {expected_parameter}")
    return compute_trial_metrics(rows, radius_m)


def analyze_all(package_dir):
    """Return all validated per-trial and group statistics."""
    base_dir = package_dir / "robot_traj" / "real"
    result = {}
    for group_name, parameter_groups in TRIALS.items():
        group_result = {}
        for parameter, directories in parameter_groups.items():
            trial_metrics = [load_trial(base_dir / directory, parameter, group_name)
                             for directory in directories]
            group_result[parameter] = (directories, trial_metrics, aggregate_trials(trial_metrics))
        result[group_name] = group_result
    return result


def _format_header(parameter_name):
    return (f"| {parameter_name} | 试次 | 平均距离误差 (m) | RMS 距离误差 (m) | "
            "末 10 s 距离误差 (m) | 末 10 s 距离误差标准差 (m) | "
            "平均相对速度误差 (m/s) | 末 10 s 相对速度误差 (m/s) |\n"
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")


def format_markdown(results):
    """Format detailed and aggregate tables for the statistics document."""
    lines = ["## 无前馈敏感性统计结果", "",
             "指标按每次实验的全程或末 10 s 计算；组级结果为 3 次试验的均值 ± 样本标准差。", ""]
    for group_name, parameter_groups in results.items():
        lines.extend([f"### 不同{group_name}对比", "", "#### 逐次明细", "", _format_header(group_name)])
        for parameter, (directories, trials, _) in parameter_groups.items():
            for index, (directory, metrics) in enumerate(zip(directories, trials), start=1):
                values = " | ".join(f"{metrics[key]:.4f}" for key in METRIC_KEYS)
                lines.append(f"| {parameter} | {index} (`{directory}`) | {values} |")
        lines.extend(["", "#### 组级汇总（均值 ± 标准差）", "", _format_header(group_name)])
        for parameter, (_, _, summary) in parameter_groups.items():
            values = " | ".join(f"{mean:.4f} ± {std:.4f}" for mean, std in summary.values())
            lines.append(f"| {parameter} | n=3 | {values} |")
        lines.append("")
    return "\n".join(lines).rstrip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=("markdown",), default="markdown")
    parser.parse_args()
    print(format_markdown(analyze_all(Path(__file__).resolve().parents[1])))


if __name__ == "__main__":
    main()
