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
REQUIRED_COLUMNS = [
    "time_s", "leader_x_m", "leader_y_m", "leader_vx_ms", "leader_vy_ms",
    "leader_v_ms", "follower_x_m", "follower_y_m", "follower_vx_ms",
    "follower_vy_ms", "follower_v_ms", "distance_m",
]


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
        "final_distance_error_m": float(abs(distance_errors[-1])),
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


def write_metrics(rows, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "metrics.csv"
    fieldnames = list(rows[0])
    with tempfile.NamedTemporaryFile(
            "w", newline="", encoding="utf-8", dir=output_dir, delete=False) as handle:
        temporary_path = Path(handle.name)
        try:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
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
    except ValueError as error:
        print(f"analysis failed: {error}", file=sys.stderr)
        return 2
    print(f"wrote {len(rows)} experiment rows to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
