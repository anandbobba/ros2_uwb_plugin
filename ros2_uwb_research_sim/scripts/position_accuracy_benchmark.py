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
# Position Accuracy Benchmark — Pure Python, no ROS runtime required
# ------------------------------------------------------------------
# Sweeps anchor count × Gaussian noise level, runs 1000 Gauss-Newton
# solves per configuration, and reports RMSE, 90th-percentile error,
# and convergence rate.
#
# Usage:
#   python3 position_accuracy_benchmark.py
#   python3 position_accuracy_benchmark.py --iters 500 --output /tmp/bench.csv

"""Offline position-accuracy benchmark: sweep anchor count × noise level."""

import argparse
import csv
from datetime import datetime
import math
import os
import random
import sys

# ---------------------------------------------------------------------------
# Gauss-Newton trilateration solver (mirrors TrilaterationNode logic)
# ---------------------------------------------------------------------------


class Anchor:
    __slots__ = ("position", "range_m")

    def __init__(self, x, y, z, range_m):
        self.position = (x, y, z)
        self.range_m = range_m


def _norm3(a, b):
    """Euclidean distance between two 3-tuples."""
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _gauss_newton(anchors, x0=(0.0, 0.0, 0.0), max_iter=50, tol=1e-7):
    """
    Iterate Gauss-Newton position solver.

    Returns (estimate_3tuple, converged: bool).
    """
    x = list(x0)
    n = len(anchors)

    for _ in range(max_iter):
        # Build Jacobian J (n×3) and residual r (n,)
        J = []
        r = []
        for a in anchors:
            diff = [x[k] - a.position[k] for k in range(3)]
            d = max(math.sqrt(sum(v ** 2 for v in diff)), 1e-6)
            J.append([diff[k] / d for k in range(3)])
            r.append(a.range_m - d)

        # Normal equations: dx = (J^T J)^{-1} J^T r  (3x3 system)
        JtJ = [[sum(J[i][p] * J[i][q] for i in range(n)) for q in range(3)] for p in range(3)]
        Jtr = [sum(J[i][p] * r[i] for i in range(n)) for p in range(3)]

        # Cholesky-like solve for small 3x3 — use Cramer's rule via numpy-free Gaussian elim
        dx = _solve3(JtJ, Jtr)
        if dx is None:
            return tuple(x), False

        x = [x[k] + dx[k] for k in range(3)]
        if math.sqrt(sum(v ** 2 for v in dx)) < tol:
            return tuple(x), True

    return tuple(x), False


def _solve3(A, b):
    """Solve 3x3 linear system Ax=b via Gaussian elimination. Returns None if singular."""
    aug = [list(A[i]) + [b[i]] for i in range(3)]

    for col in range(3):
        # Partial pivot
        max_row = max(range(col, 3), key=lambda r: abs(aug[r][col]))
        aug[col], aug[max_row] = aug[max_row], aug[col]
        if abs(aug[col][col]) < 1e-12:
            return None
        pivot = aug[col][col]
        for row in range(col + 1, 3):
            factor = aug[row][col] / pivot
            for j in range(col, 4):
                aug[row][j] -= factor * aug[col][j]

    x = [0.0, 0.0, 0.0]
    for i in range(2, -1, -1):
        x[i] = aug[i][3]
        for j in range(i + 1, 3):
            x[i] -= aug[i][j] * x[j]
        x[i] /= aug[i][i]
    return x


# ---------------------------------------------------------------------------
# Benchmark configurations
# ---------------------------------------------------------------------------

def generate_anchors_geometry(n_anchors, radius=8.0, height=2.5):
    """Place n_anchors evenly on a circle of given radius at a given height."""
    positions = []
    for i in range(n_anchors):
        angle = 2 * math.pi * i / n_anchors
        positions.append((radius * math.cos(angle), radius * math.sin(angle), height))
    return positions


def make_noisy_anchors(positions, tag_pos, sigma, rng):
    """Create Anchor objects with Gaussian-noisy ranges."""
    anchors = []
    for pos in positions:
        true_range = _norm3(pos, tag_pos)
        noisy_range = true_range + rng.gauss(0.0, sigma)
        anchors.append(Anchor(*pos, noisy_range))
    return anchors


def run_single_config(n_anchors, sigma, n_iters, rng):
    """Run n_iters Gauss-Newton solves for a given config. Return metrics dict."""
    positions = generate_anchors_geometry(n_anchors)
    errors = []
    converged = 0

    for _ in range(n_iters):
        # Random tag position inside the anchor ring
        tag_x = rng.uniform(-5.0, 5.0)
        tag_y = rng.uniform(-5.0, 5.0)
        tag_z = 0.0
        tag_pos = (tag_x, tag_y, tag_z)

        anchors = make_noisy_anchors(positions, tag_pos, sigma, rng)
        est, ok = _gauss_newton(anchors)

        err = _norm3(est, tag_pos)
        errors.append(err)
        if ok:
            converged += 1

    errors.sort()
    n = len(errors)
    rmse = math.sqrt(sum(e ** 2 for e in errors) / n)
    mae = sum(errors) / n
    p90 = errors[int(0.90 * n)]
    conv_rate = converged / n_iters

    return {
        'n_anchors': n_anchors,
        'sigma_m': sigma,
        'n_iters': n_iters,
        'rmse_m': rmse,
        'mae_m': mae,
        'p90_m': p90,
        'convergence_pct': conv_rate * 100.0,
    }


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def print_table(results):
    """Print a formatted ASCII comparison table."""
    hdr = (f"{'Anchors':>7}  {'σ(m)':>6}  "
           f"{'RMSE(m)':>9}  {'MAE(m)':>8}  "
           f"{'P90(m)':>8}  {'Conv%':>6}")
    sep = "-" * len(hdr)
    print("\n" + sep)
    print(hdr)
    print(sep)
    for r in results:
        print(f"{r['n_anchors']:>7}  {r['sigma_m']:>6.3f}  "
              f"{r['rmse_m']:>9.4f}  {r['mae_m']:>8.4f}  "
              f"{r['p90_m']:>8.4f}  {r['convergence_pct']:>6.1f}")
    print(sep + "\n")


def save_csv(results, path):
    """Write results to a CSV file."""
    fields = ["n_anchors", "sigma_m", "n_iters", "rmse_m", "mae_m",
              "p90_m", "convergence_pct"]
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    print(f'Results saved to: {path}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

ANCHOR_COUNTS = [3, 4, 6, 8]
NOISE_LEVELS = [0.0, 0.05, 0.10, 0.20]   # metres


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Offline UWB position accuracy benchmark (no ROS needed)')
    parser.add_argument('--iters', type=int, default=1000,
                        help='Gauss-Newton solves per configuration (default: 1000)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output CSV path (default: ~/position_accuracy_<timestamp>.csv)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed for reproducibility (default: 42)')
    return parser.parse_args()


def main():
    """Entry point."""
    args = parse_args()
    rng = random.Random(args.seed)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = args.output or os.path.expanduser(
        f'~/position_accuracy_{timestamp}.csv')

    print("\nUWB Position Accuracy Benchmark")
    print(f"  Anchor counts  : {ANCHOR_COUNTS}")
    print(f"  Noise levels   : {NOISE_LEVELS} m")
    print(f"  Iters per cfg  : {args.iters}")
    print(f"  Random seed    : {args.seed}\n")

    results = []
    total = len(ANCHOR_COUNTS) * len(NOISE_LEVELS)
    done = 0
    for n_anchors in ANCHOR_COUNTS:
        for sigma in NOISE_LEVELS:
            done += 1
            sys.stdout.write(f"\r  Running [{done}/{total}]  "
                             f"{n_anchors} anchors, σ={sigma:.3f} m ...    ")
            sys.stdout.flush()
            r = run_single_config(n_anchors, sigma, args.iters, rng)
            results.append(r)

    print("")
    print_table(results)
    save_csv(results, csv_path)

    # Summary: best config by RMSE (excluding ideal σ=0)
    noisy = [r for r in results if r["sigma_m"] > 0]
    if noisy:
        best = min(noisy, key=lambda r: r["rmse_m"])
        print(f"Best noisy config → {best['n_anchors']} anchors, "
              f"σ={best['sigma_m']} m  "
              f"(RMSE={best['rmse_m']:.4f} m)")


if __name__ == '__main__':
    main()
