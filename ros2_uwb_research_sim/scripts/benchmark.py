#!/usr/bin/env python3

# Copyright 2026 Anand Bobba
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# UWB Noise Model Benchmark
# -------------------------
# Parametric sweep over noise configurations to compare localization error.
# Runs multiple experiment configs and produces a comparison summary.
#
# Usage:
#   ros2 run ros2_uwb_research_sim benchmark.py
#   ros2 run ros2_uwb_research_sim benchmark.py --duration 30 --output /tmp/bench

"""Parametric benchmark sweep over UWB noise model configurations."""

import argparse
import csv
import math
import os
import subprocess
import sys
from datetime import datetime


# ---------------------------------------------------------------------------
# Benchmark configurations: (name, params.yaml overrides as dict)
# ---------------------------------------------------------------------------
CONFIGS = [
    {
        "name": "ideal",
        "description": "No noise (ideal channel)",
        "gaussian_sigma": 0.0,
        "nlos_prob": 0.0,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "low_noise",
        "description": "Low Gaussian noise (sigma=0.05m)",
        "gaussian_sigma": 0.05,
        "nlos_prob": 0.0,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "medium_noise",
        "description": "Medium Gaussian noise (sigma=0.15m)",
        "gaussian_sigma": 0.15,
        "nlos_prob": 0.0,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "high_noise",
        "description": "High Gaussian noise (sigma=0.30m)",
        "gaussian_sigma": 0.30,
        "nlos_prob": 0.0,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "nlos_10pct",
        "description": "Gaussian + 10% NLOS",
        "gaussian_sigma": 0.10,
        "nlos_prob": 0.10,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "nlos_30pct",
        "description": "Gaussian + 30% NLOS",
        "gaussian_sigma": 0.10,
        "nlos_prob": 0.30,
        "multipath_sigma": 0.0,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "multipath",
        "description": "Gaussian + multipath (alpha=0.5)",
        "gaussian_sigma": 0.10,
        "nlos_prob": 0.0,
        "multipath_sigma": 0.08,
        "clock_drift_sigma": 0.0,
    },
    {
        "name": "full_channel",
        "description": "All noise sources active",
        "gaussian_sigma": 0.10,
        "nlos_prob": 0.15,
        "multipath_sigma": 0.05,
        "clock_drift_sigma": 0.002,
    },
]


def write_experiment_yaml(config: dict, path: str) -> None:
    """Write a temporary experiment YAML for runner.py."""
    with open(path, "w") as f:
        f.write(f"name: {config['name']}\n")
        f.write(f"duration: {config.get('duration', 20)}\n")
        f.write("uwb:\n")
        f.write(f"  gaussian_sigma: {config['gaussian_sigma']}\n")
        f.write(f"  nlos_prob: {config['nlos_prob']}\n")
        f.write("  nlos_lambda: 2.0\n")
        f.write("  multipath_alpha: 0.5\n")
        f.write(f"  multipath_sigma: {config['multipath_sigma']}\n")
        f.write(f"  clock_drift_sigma: {config['clock_drift_sigma']}\n")


def compute_metrics(csv_path: str) -> dict:
    """Parse a researcher_node CSV and return RMSE, MAE, bias, std."""
    errors = []
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                gt = float(row["ground_truth"])
                meas = float(row["measured"])
                errors.append(meas - gt)
    except (FileNotFoundError, KeyError, ValueError):
        return {"rmse": float("nan"), "mae": float("nan"),
                "bias": float("nan"), "std": float("nan"), "n": 0}

    if not errors:
        return {"rmse": float("nan"), "mae": float("nan"),
                "bias": float("nan"), "std": float("nan"), "n": 0}

    n = len(errors)
    mean_err = sum(errors) / n
    rmse = math.sqrt(sum(e ** 2 for e in errors) / n)
    mae = sum(abs(e) for e in errors) / n
    variance = sum((e - mean_err) ** 2 for e in errors) / n

    return {
        "rmse": rmse,
        "mae": mae,
        "bias": mean_err,
        "std": math.sqrt(variance),
        "n": n,
    }


def run_config(config: dict, output_dir: str, duration: int) -> dict:
    """Run a single benchmark configuration and return its metrics."""
    config = dict(config)
    config["duration"] = duration

    yaml_path = os.path.join(output_dir, f"{config['name']}.yaml")
    csv_path = os.path.join(output_dir, f"{config['name']}.csv")

    write_experiment_yaml(config, yaml_path)

    print(f"  Running '{config['name']}': {config['description']} ({duration}s)...")

    try:
        result = subprocess.run(
            ["ros2", "run", "ros2_uwb_research_sim", "runner.py",
             "--config", yaml_path, "--output", csv_path],
            timeout=duration + 30,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"    [WARN] runner.py exited with code {result.returncode}")
    except subprocess.TimeoutExpired:
        print(f"    [WARN] '{config['name']}' timed out")
    except FileNotFoundError:
        # runner.py not available — generate synthetic data for demo
        print("    [INFO] runner.py not found, generating synthetic metrics")
        _write_synthetic_csv(config, csv_path, n=500)

    metrics = compute_metrics(csv_path)
    metrics["name"] = config["name"]
    metrics["description"] = config["description"]
    return metrics


def _write_synthetic_csv(config: dict, csv_path: str, n: int = 500) -> None:
    """Generate synthetic range measurements for offline benchmarking."""
    import random
    rng = random.Random(42)
    sigma = config["gaussian_sigma"]
    nlos_prob = config["nlos_prob"]
    nlos_lambda = 2.0
    mp_sigma = config["multipath_sigma"]
    drift_sigma = config["clock_drift_sigma"]

    mp_state = 0.0
    drift = 0.0
    true_dist = 5.0  # fixed for benchmark

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "anchor_id", "ground_truth", "measured",
                         "gaussian_noise", "nlos_bias", "multipath_error", "clock_drift"])
        for i in range(n):
            g_noise = rng.gauss(0, sigma)
            nlos = rng.expovariate(nlos_lambda) if rng.random() < nlos_prob else 0.0
            mp_state = 0.5 * mp_state + rng.gauss(0, mp_sigma)
            drift += rng.gauss(0, drift_sigma)
            measured = true_dist + g_noise + nlos + mp_state + drift
            writer.writerow([i * 0.05, "A0", true_dist, measured,
                             g_noise, nlos, mp_state, drift])


def print_table(results: list) -> None:
    """Print a formatted comparison table."""
    header = (
        f"{'Config':<20} {'Description':<40}"
        f" {'RMSE(m)':>9} {'MAE(m)':>8} {'Bias(m)':>9} {'Std(m)':>8} {'N':>6}"
    )
    sep = "-" * len(header)
    print("\n" + sep)
    print(header)
    print(sep)
    for r in results:
        print(f"{r['name']:<20} {r['description']:<40} "
              f"{r['rmse']:>9.4f} {r['mae']:>8.4f} "
              f"{r['bias']:>9.4f} {r['std']:>8.4f} {r['n']:>6}")
    print(sep + "\n")


def save_summary(results: list, output_dir: str) -> None:
    """Write benchmark summary CSV."""
    path = os.path.join(output_dir, "benchmark_summary.csv")
    with open(path, "w", newline="") as f:
        fields = ["name", "description", "rmse", "mae", "bias", "std", "n"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    print(f"Summary saved to: {path}")


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Benchmark UWB noise models across configurations")
    parser.add_argument("--duration", type=int, default=20,
                        help="Duration (seconds) per configuration (default: 20)")
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output directory for results"
             " (default: ~/ros2_uwb_benchmark_<timestamp>)")
    parser.add_argument("--configs", nargs="+", default=None,
                        help="Subset of config names to run (default: all)")
    return parser.parse_args()


def main():
    """Run the benchmark sweep and print results."""
    args = parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output or os.path.expanduser(f"~/ros2_uwb_benchmark_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)

    configs = CONFIGS
    if args.configs:
        configs = [c for c in CONFIGS if c["name"] in args.configs]
        if not configs:
            print(f"No matching configs. Available: {[c['name'] for c in CONFIGS]}")
            sys.exit(1)

    print("\nUWB Noise Model Benchmark")
    print(f"  Configurations : {len(configs)}")
    print(f"  Duration each  : {args.duration}s")
    print(f"  Output dir     : {output_dir}\n")

    results = []
    for config in configs:
        metrics = run_config(config, output_dir, args.duration)
        results.append(metrics)
        print(f"    RMSE={metrics['rmse']:.4f}m  MAE={metrics['mae']:.4f}m  "
              f"Bias={metrics['bias']:.4f}m  Std={metrics['std']:.4f}m  N={metrics['n']}")

    print_table(results)
    save_summary(results, output_dir)

    # Best config by RMSE
    valid = [r for r in results if not math.isnan(r["rmse"]) and r["n"] > 0]
    if valid:
        best = min(valid, key=lambda r: r["rmse"])
        print(f"Best configuration by RMSE: '{best['name']}' (RMSE={best['rmse']:.4f}m)")


if __name__ == "__main__":
    main()
