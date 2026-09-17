#!/usr/bin/env python3
"""Produce auditable, consistently defined metrics for the real-robot report."""

import argparse
import csv
from decimal import Decimal
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile

import yaml


IDEAL_RADIUS_M = 1.0
TAIL_WINDOW_S = 10.0
REQUIRED_COLUMNS = [
    "time_s", "leader_x_m", "leader_y_m", "leader_vx_ms", "leader_vy_ms",
    "leader_v_ms", "follower_x_m", "follower_y_m", "follower_vx_ms",
    "follower_vy_ms", "follower_v_ms", "distance_m",
]
REPORT_ASSET_FILENAMES = (
    "feedforward_trajectory.png",
    "feedforward_velocity_components.png",
    "feedforward_x_position.png",
    "feedforward_y_position.png",
    "hpc_lpc_trajectory.png",
    "hpc_lpc_velocity_components.png",
    "hpc_lpc_x_position.png",
    "hpc_lpc_y_position.png",
)
LEADER_COLOR = "#1f77b4"
FOLLOWER_COLOR = "#ff7f0e"
IDEAL_RADIUS_COLOR = "#7f7f7f"
PNG_DPI = 300


def chinese_sans_serif_font():
    """Return an installed Chinese sans-serif font name, if one is available."""
    from matplotlib import font_manager

    preferred_names = (
        "Noto Sans CJK SC", "Noto Sans CJK", "Source Han Sans SC",
        "Source Han Sans CN", "Microsoft YaHei", "SimHei", "WenQuanYi Zen Hei",
        "PingFang SC", "Heiti SC",
    )
    available = {font.name for font in font_manager.fontManager.ttflist}
    return next((name for name in preferred_names if name in available), None)


def configure_report_plotting():
    """Configure headless Matplotlib and return localized report labels."""
    import matplotlib
    matplotlib.use("Agg", force=True)
    from matplotlib import pyplot as plt

    font_name = chinese_sans_serif_font()
    if font_name:
        plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": [font_name],
                             "axes.unicode_minus": False})
        return True, plt
    print("warning: Chinese sans-serif font unavailable; using English report labels.",
          file=sys.stderr)
    plt.rcParams.update({"font.family": "DejaVu Sans", "axes.unicode_minus": False})
    return False, plt


def report_label(experiment, chinese_labels):
    """Use manifest labels when renderable, otherwise provide an ASCII equivalent."""
    if chinese_labels:
        return experiment["display_label"]
    if experiment["controller_family"] == "artstein_lpc":
        return "Artstein-LPC (lambda={:.1f})".format(experiment["initial_min_lambda"])
    if experiment["id"] == "hpc_lpc_reference":
        return "Artstein-HPC (lambda={:.1f})".format(experiment["initial_min_lambda"])
    state = "on" if experiment.get("leader_command_feedforward") else "off"
    return f"Artstein-HPC (Leader command feedforward: {state})"


def stage_experiment_ids():
    """Return the dedicated records used for the HPC/LPC stage comparison."""
    return ("hpc_lpc_reference", "lpc_lambda_25_v025")


def compute_distance_metrics(distances, ideal_radius_m):
    """Return distance and ideal-formation-radius error statistics."""
    if ideal_radius_m <= 0 or not math.isfinite(ideal_radius_m):
        raise ValueError("ideal_radius_m must be a positive finite number")
    if not distances:
        raise ValueError("distance data is empty")
    if not all(math.isfinite(distance) for distance in distances):
        raise ValueError("distance data contains a non-finite value")

    decimal_distances = [Decimal(str(distance)) for distance in distances]
    decimal_radius = Decimal(str(ideal_radius_m))
    distance_errors = [distance - decimal_radius for distance in decimal_distances]
    return {
        "sample_count": len(distances),
        "mean_distance_m": statistics.mean(distances),
        "std_distance_m": statistics.pstdev(distances),
        "mean_abs_distance_error_m": float(
            sum(abs(error) for error in distance_errors) / len(distance_errors)),
        "rms_distance_error_m": float(
            (sum(error ** 2 for error in distance_errors) / len(distance_errors)).sqrt()),
    }


def compute_tail_distance_metrics(rows, ideal_radius_m, window_s):
    """Return stability metrics over the final time window of a trajectory."""
    if window_s <= 0 or not math.isfinite(window_s):
        raise ValueError("window_s must be a positive finite number")
    if not rows:
        raise ValueError("trajectory rows are empty")
    end_time = rows[-1]["time_s"]
    tail_distances = [row["distance_m"] for row in rows
                      if row["time_s"] >= end_time - window_s]
    base_metrics = compute_distance_metrics(tail_distances, ideal_radius_m)
    errors = [distance - ideal_radius_m for distance in tail_distances]
    return {
        "tail_window_s": window_s,
        "tail_mean_abs_distance_error_m": base_metrics["mean_abs_distance_error_m"],
        "tail_distance_error_std_m": statistics.pstdev(errors),
    }


def compute_velocity_tracking_metrics(rows, window_s):
    """Return Leader--Follower velocity consistency metrics from measured velocities."""
    if window_s <= 0 or not math.isfinite(window_s):
        raise ValueError("window_s must be a positive finite number")
    if not rows:
        raise ValueError("trajectory rows are empty")
    relative_errors = [math.hypot(row["follower_vx_ms"] - row["leader_vx_ms"],
                                  row["follower_vy_ms"] - row["leader_vy_ms"])
                       for row in rows]
    end_time = rows[-1]["time_s"]
    tail_errors = [error for row, error in zip(rows, relative_errors)
                   if row["time_s"] >= end_time - window_s]
    return {
        "mean_relative_velocity_error_mps": statistics.mean(relative_errors),
        "tail_mean_relative_velocity_error_mps": statistics.mean(tail_errors),
    }


def load_yaml(path):
    try:
        with path.open(encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"{path}: cannot read YAML: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def load_csv_rows(path):
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != REQUIRED_COLUMNS:
                raise ValueError(
                    f"{path}: expected exactly columns {', '.join(REQUIRED_COLUMNS)}")
            rows = []
            for row_number, row in enumerate(reader, start=2):
                try:
                    values = {name: float(row[name]) for name in REQUIRED_COLUMNS}
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f"{path}: row {row_number} has a non-numeric value") from error
                nonfinite = next((name for name, value in values.items()
                                  if not math.isfinite(value)), None)
                if nonfinite:
                    raise ValueError(
                        f"{path}: row {row_number} column {nonfinite} is non-finite")
                rows.append(values)
    except OSError as error:
        raise ValueError(f"{path}: cannot read CSV: {error}") from error
    if not rows:
        raise ValueError(f"{path}: CSV has no data rows")
    return rows


def path_length(rows, x_name, y_name):
    return sum(math.hypot(current[x_name] - previous[x_name],
                          current[y_name] - previous[y_name])
               for previous, current in zip(rows, rows[1:]))


def analyze_experiment(repository_root, experiment):
    experiment_id = experiment.get("id")
    source_dir = experiment.get("source_dir")
    if not isinstance(experiment_id, str) or not isinstance(source_dir, str):
        raise ValueError("manifest experiment requires string id and source_dir")

    directory = repository_root / "homo_multirobot_formation_control" / source_dir
    metadata = load_yaml(directory / "metadata.yaml")
    if metadata.get("platform") != "real":
        raise ValueError(f"{directory / 'metadata.yaml'}: platform must be real")
    rows = load_csv_rows(directory / "raw.csv")
    distances = [row["distance_m"] for row in rows]
    metrics = compute_distance_metrics(distances, IDEAL_RADIUS_M)
    metrics.update(compute_tail_distance_metrics(rows, IDEAL_RADIUS_M, TAIL_WINDOW_S))
    metrics.update(compute_velocity_tracking_metrics(rows, TAIL_WINDOW_S))
    metrics.update({
        "experiment_id": experiment_id,
        "display_label": experiment.get("display_label", ""),
        "comparison": experiment.get("comparison", ""),
        "controller_family": experiment.get("controller_family", ""),
        "source_dir": source_dir,
        "trial_id": metadata.get("trial_id", ""),
        "ideal_radius_m": IDEAL_RADIUS_M,
        "duration_s": rows[-1]["time_s"] - rows[0]["time_s"],
        "leader_path_length_m": path_length(rows, "leader_x_m", "leader_y_m"),
        "follower_path_length_m": path_length(rows, "follower_x_m", "follower_y_m"),
        "mean_leader_speed_mps": statistics.mean(row["leader_v_ms"] for row in rows),
        "std_leader_speed_mps": statistics.pstdev(row["leader_v_ms"] for row in rows),
        "mean_follower_speed_mps": statistics.mean(row["follower_v_ms"] for row in rows),
        "std_follower_speed_mps": statistics.pstdev(row["follower_v_ms"] for row in rows),
    })
    return metrics


def analyze_manifest(manifest_path):
    manifest = load_yaml(manifest_path)
    experiments = manifest.get("experiments")
    if not isinstance(experiments, list) or not experiments:
        raise ValueError(f"{manifest_path}: experiments must be a non-empty list")
    repository_root = manifest_path.resolve().parents[3]
    return [analyze_experiment(repository_root, experiment) for experiment in experiments]


def load_report_series(manifest_path):
    """Load the manifest-selected raw series without modifying the source trials."""
    manifest = load_yaml(manifest_path)
    experiments = manifest.get("experiments")
    if not isinstance(experiments, list) or not experiments:
        raise ValueError(f"{manifest_path}: experiments must be a non-empty list")
    repository_root = manifest_path.resolve().parents[3]
    series = []
    for experiment in experiments:
        experiment_id = experiment.get("id")
        source_dir = experiment.get("source_dir")
        if not isinstance(experiment_id, str) or not isinstance(source_dir, str):
            raise ValueError("manifest experiment requires string id and source_dir")
        rows = load_csv_rows(
            repository_root / "homo_multirobot_formation_control" / source_dir / "raw.csv")
        series.append((experiment, rows))
    return series


def _save_figure(figure, output_path):
    figure.savefig(output_path, dpi=PNG_DPI, bbox_inches="tight")


def _draw_trajectory(axis, selected_series, chinese_labels):
    """Draw one XY trajectory comparison without an ideal-radius reference."""
    for index, (experiment, rows) in enumerate(selected_series):
        label = report_label(experiment, chinese_labels)
        line_style = ("-", "--", ":")[index]
        leader_name = "Leader" if not chinese_labels else "领航者"
        follower_name = "Follower" if not chinese_labels else "跟随者"
        leader_x = [row["leader_x_m"] for row in rows]
        leader_y = [row["leader_y_m"] for row in rows]
        follower_x = [row["follower_x_m"] for row in rows]
        follower_y = [row["follower_y_m"] for row in rows]
        axis.plot(leader_x, leader_y, color=LEADER_COLOR, linestyle=line_style, linewidth=1.3,
                  label=f"{label} — {leader_name}")
        axis.plot(follower_x, follower_y, color=FOLLOWER_COLOR, linestyle=line_style, linewidth=1.3,
                  label=f"{label} — {follower_name}")
        axis.plot(leader_x[0], leader_y[0], marker="o", markersize=3.5, color=LEADER_COLOR)
        axis.plot(follower_x[0], follower_y[0], marker="o", markersize=3.5, color=FOLLOWER_COLOR)
        axis.plot(leader_x[-1], leader_y[-1], marker="x", markersize=4.5, color=LEADER_COLOR)
        axis.plot(follower_x[-1], follower_y[-1], marker="x", markersize=4.5, color=FOLLOWER_COLOR)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=6.5, loc="best")


def _draw_velocity_components(axis, selected_series, chinese_labels):
    """Draw Vx and Vy traces for each Leader--Follower experiment pair."""
    for index, (experiment, rows) in enumerate(selected_series):
        label = report_label(experiment, chinese_labels)
        line_style = ("-", "--", ":")[index]
        time_s = [row["time_s"] for row in rows]
        axis.plot(time_s, [row["leader_vx_ms"] for row in rows], color=LEADER_COLOR,
                  linestyle=line_style, linewidth=1.0, label=f"{label} — Leader Vx")
        axis.plot(time_s, [row["follower_vx_ms"] for row in rows], color=FOLLOWER_COLOR,
                  linestyle=line_style, linewidth=1.0, label=f"{label} — Follower Vx")
        axis.plot(time_s, [row["leader_vy_ms"] for row in rows], color="#4fa3d1",
                  linestyle=line_style, linewidth=1.0, label=f"{label} — Leader Vy")
        axis.plot(time_s, [row["follower_vy_ms"] for row in rows], color="#c99a00",
                  linestyle=line_style, linewidth=1.0, label=f"{label} — Follower Vy")
    axis.set_xlabel("Time (s)" if not chinese_labels else "时间 (s)")
    axis.set_ylabel("Velocity (m/s)" if not chinese_labels else "速度 (m/s)")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=6.2, loc="best", ncol=2)


def _draw_position_component(axis, selected_series, component, chinese_labels):
    """Draw the selected map-position component against time."""
    column = f"{component.lower()}_m"
    leader_column = f"leader_{column}"
    follower_column = f"follower_{column}"
    for index, (experiment, rows) in enumerate(selected_series):
        label = report_label(experiment, chinese_labels)
        line_style = ("-", "--", ":")[index]
        time_s = [row["time_s"] for row in rows]
        axis.plot(time_s, [row[leader_column] for row in rows], color=LEADER_COLOR,
                  linestyle=line_style, linewidth=1.1, label=f"{label} — Leader")
        axis.plot(time_s, [row[follower_column] for row in rows], color=FOLLOWER_COLOR,
                  linestyle=line_style, linewidth=1.1, label=f"{label} — Follower")
    axis.set_xlabel("Time (s)" if not chinese_labels else "时间 (s)")
    axis.set_ylabel(f"{component} (m)")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=6.5, loc="best")


def _draw_control_pipeline(plt, output_path, chinese_labels):
    """Draw an aligned control-flow diagram with one consistent arrow baseline."""
    from matplotlib.patches import FancyBboxPatch

    figure, axis = plt.subplots(figsize=(15, 4.6))
    axis.set_xlim(0, 15.7)
    axis.set_ylim(0, 5.2)
    axis.axis("off")
    labels = (
        ("Motion-capture\nstate", "动捕状态"),
        ("Follower Artstein\nintegral compensation Td", "Follower Artstein\n积分补偿 Td"),
        ("Follower forward\nprediction tau", "Follower 前向预测 tau"),
        ("HPC or LPC\ncontrol law", "HPC 或 LPC\n控制律"),
        ("Velocity / wheel-speed\nconstraints", "速度/轮速约束"),
        ("Follower cmd_vel", "Follower cmd_vel"),
    )
    xs = (0.25, 2.7, 5.15, 7.6, 10.65, 13.1)
    width, height, y = 1.85, 0.92, 1.95
    line_y = y + height / 2
    for x, label_pair in zip(xs, labels):
        text = label_pair[1] if chinese_labels else label_pair[0]
        patch = FancyBboxPatch((x, y), width, height, boxstyle="round,pad=0.05,rounding_size=0.08",
                               facecolor="#eef4fb", edgecolor="#356a9a", linewidth=1.25)
        axis.add_patch(patch)
        axis.text(x + width / 2, line_y, text, ha="center", va="center", fontsize=8.4)

    sum_x, sum_y, sum_radius = 9.95, line_y, 0.18
    axis.text(sum_x, sum_y, "+", ha="center", va="center", fontsize=15, weight="bold",
              bbox={"boxstyle": "circle,pad=0.14", "fc": "white", "ec": "#333333"})
    flow_segments = (
        (xs[0] + width, xs[1]),
        (xs[1] + width, xs[2]),
        (xs[2] + width, xs[3]),
        (xs[3] + width, sum_x - sum_radius),
        (sum_x + sum_radius, xs[4]),
        (xs[4] + width, xs[5]),
    )
    for start_x, end_x in flow_segments:
        axis.annotate("", xy=(end_x, line_y), xytext=(start_x, line_y),
                      arrowprops={"arrowstyle": "->", "lw": 1.45, "color": "#333333"})
    optional = ("Optional: Leader cmd_vel velocity-increment\nfeedforward (Experiment 1 on/off)"
                if not chinese_labels else "可选：Leader cmd_vel 速度增量前馈\n（实验一开/关）")
    optional_y = 4.0
    optional_width, optional_height = 4.1, 0.8
    optional_x = sum_x - optional_width / 2
    patch = FancyBboxPatch((optional_x, optional_y), optional_width, optional_height,
                           boxstyle="round,pad=0.05,rounding_size=0.08", facecolor="#fff7e6",
                           edgecolor="#b36b00", linewidth=1.25)
    axis.add_patch(patch)
    axis.text(sum_x, optional_y + optional_height / 2, optional, ha="center", va="center", fontsize=8)
    axis.annotate("", xy=(sum_x, sum_y + sum_radius), xytext=(sum_x, optional_y),
                  arrowprops={"arrowstyle": "->", "lw": 1.25, "ls": "--", "color": "#b36b00"})
    _save_figure(figure, output_path)
    plt.close(figure)


def generate_report_assets(manifest_path, assets_dir):
    """Generate separate 300 dpi report figures from manifest-selected raw series."""
    chinese_labels, plt = configure_report_plotting()
    series_by_id = {experiment["id"]: (experiment, rows)
                    for experiment, rows in load_report_series(manifest_path)}
    required_ids = ("hpc_feedforward_on", "hpc_feedforward_off", *stage_experiment_ids())
    missing = [experiment_id for experiment_id in required_ids if experiment_id not in series_by_id]
    if missing:
        raise ValueError("manifest missing required report experiments: " + ", ".join(missing))
    assets_dir.mkdir(parents=True, exist_ok=True)
    ff_series = [series_by_id[experiment_id] for experiment_id in required_ids[:2]]
    stage_series = [series_by_id[experiment_id] for experiment_id in stage_experiment_ids()]

    for prefix, selected_series in (("feedforward", ff_series), ("hpc_lpc", stage_series)):
        figure, axis = plt.subplots(figsize=(7.4, 6.2), constrained_layout=True)
        _draw_trajectory(axis, selected_series, chinese_labels)
        _save_figure(figure, assets_dir / f"{prefix}_trajectory.png")
        plt.close(figure)

        figure, axis = plt.subplots(figsize=(10.8, 5.4), constrained_layout=True)
        _draw_velocity_components(axis, selected_series, chinese_labels)
        _save_figure(figure, assets_dir / f"{prefix}_velocity_components.png")
        plt.close(figure)

        for component, suffix in (("X", "x_position"), ("Y", "y_position")):
            figure, axis = plt.subplots(figsize=(10.8, 5.0), constrained_layout=True)
            _draw_position_component(axis, selected_series, component, chinese_labels)
            _save_figure(figure, assets_dir / f"{prefix}_{suffix}.png")
            plt.close(figure)

    for obsolete_name in (
            "feedforward_comparison.png", "feedforward_distance_error.png",
            "hpc_lpc_distance_error.png", "control_pipeline.png"):
        (assets_dir / obsolete_name).unlink(missing_ok=True)
    return [assets_dir / filename for filename in REPORT_ASSET_FILENAMES]


def write_metrics(rows, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "metrics.csv"
    fieldnames = list(rows[0])
    with tempfile.NamedTemporaryFile(
            "w", newline="", encoding="utf-8", dir=output_dir, delete=False) as handle:
        temporary_path = Path(handle.name)
        try:
            writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
    os.replace(temporary_path, output_path)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        rows = analyze_manifest(args.manifest)
        output_path = write_metrics(rows, args.output)
        assets = generate_report_assets(args.manifest, args.output / "assets")
    except ValueError as error:
        print(f"analysis failed: {error}", file=sys.stderr)
        return 2
    print(f"wrote {len(rows)} experiment rows to {output_path}")
    print(f"wrote {len(assets)} report PNG assets to {args.output / 'assets'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
