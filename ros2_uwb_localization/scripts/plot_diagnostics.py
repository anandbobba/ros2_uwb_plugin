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

import os
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

matplotlib.use('Agg')  # Non-interactive backend — saves PNG without needing a display


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 plot_diagnostics.py <benchmark_csv>')
        return

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f'File not found: {csv_path}')
        return

    df = pd.read_csv(csv_path)

    # 1. Error Attribution Pie Chart
    # Use absolute values for contribution
    sources = ['gaussian', 'nlos', 'multipath', 'drift']
    # Filter columns that exist
    sources = [s for s in sources if s in df.columns]

    if not sources:
        print('No diagnostic columns found in CSV. Use standard plot_results.py instead.')
        return

    total_errs = df[sources].abs().sum()

    plt.figure(figsize=(15, 10))

    # Grid: 2x2
    # 1. Pie Chart
    plt.subplot(2, 2, 1)
    plt.pie(
        total_errs, labels=total_errs.index, autopct='%1.1f%%', startangle=140,
        colors=['#66b3ff', '#99ff99', '#ffcc99', '#ff9999']
    )
    plt.title('Error Source Attribution (%)')

    # 2. Stacked Error over Time
    plt.subplot(2, 2, 2)
    time = df['timestamp'] - df['timestamp'].iloc[0]
    plt.stackplot(time, df[sources].abs().T, labels=sources, alpha=0.8)
    plt.xlabel('Time (s)')
    plt.ylabel('Error Contribution (m)')
    plt.title('Absolute Error Contribution Over Time')
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)

    # 3. Error CDF
    plt.subplot(2, 2, 3)
    errors = np.sort(df['error_3d'].values)
    cdf = np.arange(len(errors)) / float(len(errors))
    plt.plot(errors, cdf, label='Empirical CDF', linewidth=2)
    plt.axvline(np.percentile(errors, 95), color='r', linestyle='--', label='P95')
    plt.xlabel('Error (m)')
    plt.ylabel('Probability')
    plt.title('Localization Error CDF')
    plt.legend()
    plt.grid(True)

    # 4. Multipath Lag Correlation
    plt.subplot(2, 2, 4)
    if 'multipath' in df.columns:
        mp = df['multipath'].values
        lags = range(1, 41)
        corrs = [np.corrcoef(mp[:-lag], mp[lag:])[0, 1] for lag in lags]
        plt.stem(lags, corrs)
        plt.xlabel('Lag (samples)')
        plt.ylabel('Autocorrelation')
        plt.title('Multipath Error Autocorrelation')
        plt.grid(True)
    else:
        plt.text(0.5, 0.5, 'Multipath data not found', ha='center')

    plt.tight_layout()
    output_png = csv_path.replace('.csv', '_diagnostics.png')
    plt.savefig(output_png)
    print(f'Diagnostic plots saved to: {output_png}')
    # plt.show()  # Disabled: use 'xdg-open <file>.png' to view


if __name__ == '__main__':
    main()
