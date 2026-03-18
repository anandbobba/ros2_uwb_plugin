#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — saves PNG without needing a display
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_results.py <benchmark_csv>")
        return

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        return

    df = pd.read_csv(csv_path)
    
    # Accuracy Stats
    errors = df['error_3d'].values
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(errors)
    p95 = np.percentile(errors, 95)

    print(f"--- Statistics for {os.path.basename(csv_path)} ---")
    print(f"Samples: {len(errors)}")
    print(f"RMSE:    {rmse:.4f} m")
    print(f"MAE:     {mae:.4f} m")
    print(f"P95:     {p95:.4f} m")

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
    print(f"Plot saved to: {output_png}")
    # plt.show()  # Disabled: use a viewer like 'eog' or 'xdg-open' to view the saved PNG

if __name__ == '__main__':
    main()
