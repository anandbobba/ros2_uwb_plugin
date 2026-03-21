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

"""Analyze and plot UWB range error metrics from a researcher_node CSV file."""

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def analyze_and_plot(csv_path, output_dir):
    """Compute error metrics and save plots from a researcher_node CSV."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    df = pd.read_csv(csv_path)

    # Compute Metrics
    errors = df['error']
    metrics = {
        'RMSE': np.sqrt(np.mean(errors**2)),
        'MAE': np.mean(np.abs(errors)),
        'Bias': np.mean(errors),
        'Variance': np.var(errors),
        'Max Error': np.max(np.abs(errors))
    }

    # Save Metrics
    with open(os.path.join(output_dir, 'metrics.txt'), 'w') as f:
        for k, v in metrics.items():
            f.write(f'{k}: {v:.5f}\n')

    # Plots
    plt.figure(figsize=(15, 10))

    # 1. Time Series Error
    plt.subplot(2, 2, 1)
    time_rel = (df['timestamp'] - df['timestamp'].iloc[0]).values
    plt.plot(time_rel, df['error'].values, label='Error')
    plt.axhline(metrics['Bias'], color='r', linestyle='--', label='Bias')
    plt.title('Error over Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Error [m]')
    plt.legend()
    plt.grid(True)

    # 2. CDF of Error
    plt.subplot(2, 2, 2)
    sorted_error = np.sort(np.abs(df['error'].values))
    y = np.arange(len(sorted_error)) / float(len(sorted_error))
    plt.plot(sorted_error, y)
    plt.title('CDF of Absolute Error')
    plt.xlabel('Error [m]')
    plt.ylabel('Probability')
    plt.grid(True)

    # 3. Noise Component contribution
    plt.subplot(2, 2, 3)
    components = ['gaussian_noise', 'nlos_bias', 'multipath_error', 'clock_drift']
    comp_sums = [np.sum(np.abs(df[c].values)) for c in components]
    plt.pie(comp_sums, labels=components, autopct='%1.1f%%')
    plt.title('Contribution to Total Error Magnitude')

    # 4. Correlation Plot (AR(1) check)
    plt.subplot(2, 2, 4)
    # Convert to array to avoid pandas/matplotlib indexing conflict
    err_values = df['error'].values
    plt.scatter(err_values[:-1], err_values[1:], alpha=0.5)
    plt.title('Lag Plot (Temporal Correlation)')
    plt.xlabel('E(t)')
    plt.ylabel('E(t+1)')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'analysis.png'))
    plt.close()

    print(f'Analysis complete. Results saved in {output_dir}')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: python3 plotter.py <csv_path> <output_dir>')
    else:
        analyze_and_plot(sys.argv[1], sys.argv[2])
