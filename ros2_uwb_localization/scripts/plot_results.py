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

    has_yaw = 'error_yaw' in df.columns
    if has_yaw:
        yaw_errors = df['error_yaw'].values * 180.0 / np.pi
        yaw_rmse = np.sqrt(np.mean(yaw_errors**2))
        yaw_mae = np.mean(np.abs(yaw_errors))

    print(f'--- Statistics for {os.path.basename(csv_path)} ---')
    print(f'Samples: {len(errors)}')
    print(f'Pos RMSE:    {rmse:.4f} m')
    print(f'Pos MAE:     {mae:.4f} m')
    print(f'Pos P95:     {p95:.4f} m')
    if has_yaw:
        print(f'Yaw RMSE:    {yaw_rmse:.4f} deg')
        print(f'Yaw MAE:     {yaw_mae:.4f} deg')

    # Plot Distribution
    num_cols = 3 if has_yaw else 2
    plt.figure(figsize=(5 * num_cols, 10 if has_yaw else 5))

    # 1. Error over time
    plt.subplot(2 if has_yaw else 1, num_cols, 1)
    plt.plot(df['timestamp'] - df['timestamp'].iloc[0], df['error_3d'], label='3D Error')
    plt.axhline(y=mae, color='r', linestyle='--', label='MAE')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Position Error over Time')
    plt.legend()
    plt.grid(True)

    # 2. Histogram (Distribution)
    plt.subplot(2 if has_yaw else 1, num_cols, 2)
    plt.hist(errors, bins=30, density=True, alpha=0.7, color='g')
    plt.xlabel('Error (m)')
    plt.ylabel('Density')
    plt.title('Position Error Distribution')
    plt.grid(True)

    if has_yaw:
        # 3. Yaw Error over time
        plt.subplot(2, num_cols, 3)
        plt.plot(
            df['timestamp'] - df['timestamp'].iloc[0],
            yaw_errors,
            label='Yaw Error',
            color='orange'
        )
        plt.axhline(y=yaw_mae, color='r', linestyle='--', label='MAE')
        plt.axhline(y=-yaw_mae, color='r', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (deg)')
        plt.title('Yaw Error over Time')
        plt.legend()
        plt.grid(True)

        # 4. Yaw Covariance
        plt.subplot(2, num_cols, 4)
        plt.plot(
            df['timestamp'] - df['timestamp'].iloc[0],
            df['covariance_yaw'],
            label='Yaw Covariance',
            color='purple'
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Covariance (rad^2)')
        plt.title('Yaw Covariance over Time')
        plt.yscale('log')
        plt.legend()
        plt.grid(True)

        # 5. Yaw Confidence
        # Approximate confidence from covariance: cov = 0.01 + (1.0 - conf) * 100.0
        # => conf = 1.0 - (cov - 0.01)/100.0
        confidence = np.clip(1.0 - (df['covariance_yaw'] - 0.01) / 100.0, 0.0, 1.0)
        plt.subplot(2, num_cols, 5)
        plt.plot(
            df['timestamp'] - df['timestamp'].iloc[0],
            confidence,
            label='Confidence',
            color='green'
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Confidence [0, 1]')
        plt.title('Dual-Tag Yaw Confidence')
        plt.ylim(-0.1, 1.1)
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    output_png = csv_path.replace('.csv', '_plot.png')
    plt.savefig(output_png)
    print(f'Plot saved to: {output_png}')


if __name__ == '__main__':
    main()
