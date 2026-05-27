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
        print('Usage: python3 plot_results.py <benchmark_csv>')
        return

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f'File not found: {csv_path}')
        return

    df = pd.read_csv(csv_path)

    # Accuracy Stats
    errors = df['error_3d'].values
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(errors)
    p95 = np.percentile(errors, 95)

    print(f'--- Statistics for {os.path.basename(csv_path)} ---')
    print(f'Samples: {len(errors)}')
    print(f'RMSE:    {rmse:.4f} m')
    print(f'MAE:     {mae:.4f} m')
    print(f'P95:     {p95:.4f} m')

    # Plot Distribution
    plt.figure(figsize=(10, 5))

    # 1. Error over time
    plt.subplot(1, 2, 1)
    plt.plot(df['timestamp'] - df['timestamp'].iloc[0], df['error_3d'], label='3D Error')
    plt.axhline(y=mae, color='r', linestyle='--', label='MAE')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Localization Error over Time')
    plt.legend()
    plt.grid(True)

    # 2. Histogram (Distribution)
    plt.subplot(1, 2, 2)
    plt.hist(errors, bins=30, density=True, alpha=0.7, color='g')
    plt.xlabel('Error (m)')
    plt.ylabel('Density')
    plt.title('Error Distribution')
    plt.grid(True)

    plt.tight_layout()
    output_png = csv_path.replace('.csv', '_plot.png')
    plt.savefig(output_png)
    print(f'Plot saved to: {output_png}')
    # plt.show()  # Disabled: use a viewer like 'eog' or 'xdg-open' to view the saved PNG


if __name__ == '__main__':
    main()
