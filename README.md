# ROS2 UWB Localization Framework

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-orange.svg)](https://en.cppreference.com/w/cpp/17)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

A modular, production-grade UWB (Ultra-Wideband) localization ecosystem for ROS2. This framework provides high-fidelity simulation, robust position estimation, and research-grade diagnostic tools for analyzing signal propagation errors (NLOS, Multipath, Drift).

---

## 🛠 Features

- **Realistic Simulation**: Gazebo (Ignition) plugin with modular noise models:
  - **Gaussian Noise**: Thermal and quantization error.
  - **NLOS Bias**: Exponentially distributed bias for obstructed paths.
  - **Multipath AR(1)**: Time-correlated noise from signal reflections.
  - **Clock Drift**: Random walk oscillator instability.
- **Robust Localization**: 
  - Iterative Weighted Gauss-Newton Trilateration (3D and 2D modes).
  - Multi-sensor fusion via `robot_localization` EKF.
- **Research Analytics**:
  - Real-time error attribution (What % of error is NLOS vs. Noise?).
  - Automated CSV dataset logging and ROS bag integration.
  - Python scripts for RMSE, MAE, and P95 precision analysis.
- **Ease of Use**: Unified launch system with automatic robot autopilot.
- **Dependency Installer**: One-script setup for all system and ROS2 dependencies.

---

## 🎥 Demo

[▶️ Watch the ROS2 UWB Framework in Action](https://drive.google.com/drive/folders/156ZFzQYNlvvF9lHqF-TpCHE53OlKgoRr?usp=drive_link)

---

## 📐 Architecture

```text
       [ GAZEBO SIMULATION ]             [ LOCALIZATION PIPELINE ]
      +---------------------+           +--------------------------+
      |  UWB Plugin (Tag)   |           |  UWB Range Preprocessor  |
      |  (Noise + NLOS)     |           |  (Outlier Filtering)     |
      +----------+----------+           +------------+-------------+
                 | /uwb/range                        | /uwb/ranges_filtered
                 v                                   v
      +----------+----------+           +------------+-------------+
      |  Anchor Manager     |           |  Trilateration Solver    |
      |  (TF Broadcast)     |           |  (Gauss-Newton Engine)   |
      +----------+----------+           +------------+-------------+
                 | /tf                               | /uwb/pose
                 v                                   v
      +----------+----------+           +------------+-------------+
      |  RViz Visualization | <--------- |  robot_localization EKF  |
      |  (Live Tracking)    |            |  (Odometry Fusion)       |
      +---------------------+           +--------------------------+
```

---

## 🚀 Quick Start

### 1. Set Up the Environment
```bash
mkdir -p ~/ros2_uwb_ws/src && cd ~/ros2_uwb_ws/src
git clone https://github.com/anandbobba/ros2_uwb_plugin.git .
cd ..

# Install all binary and system dependencies automatically
chmod +x src/install_dependencies.sh
./src/install_dependencies.sh

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Run the Full Demo
Launch the simulator, localization pipeline, and automatic robot motion:
```bash
ros2 launch ros2_uwb_research_sim demo.launch.py
```
*(Inside Gazebo, click the "Play" button. The robot will start circular motion immediately.)*

### 3. Performance Mode (Recommended)
If your system experiences lag or "Jump in time" warnings, run in **Headless Mode** (Server only + RViz). This is 10x more stable for long-term research data collection:
```bash
ros2 launch ros2_uwb_research_sim demo.launch.py gazebo_gui:=false
```
---

## 📊 Research Workflow

### Recording Datasets
Capture raw UWB ranges, fused poses, and error diagnostics for offline analysis:
```bash
ros2 launch ros2_uwb_localization record_dataset.launch.py
```

### Accuracy Analysis
Once recorded, generate performance plots and error attribution reports:
```bash
# General performance (RMSE/Trajectory)
python3 ros2_uwb_localization/scripts/plot_results.py <dataset>.csv

# Research diagnostics (NLOS/Correlation)
python3 ros2_uwb_localization/scripts/plot_diagnostics.py <dataset>.csv
```

### Dynamic Noise Profiles
Modify the environment complexity at runtime:
```bash
ros2 param set /uwb_plugin_uwb_anchor_0 nlos_prob 0.8
```

---

## 📁 Repository Structure

- `ros2_uwb_msgs`: Custom interfaces for range aggregation and diagnostics.
- `ros2_uwb_localization`: Core positioning engine, preprocessor, and benchmark nodes.
- `ros2_uwb_research_sim`: Ignition Gazebo world, robot URDF, and research plugin.

---

## 📡 Example Output (Topics)

| Topic | Description |
|---|---|
| `/uwb/range` | Raw range measurements from simulator/hardware. |
| `/uwb/pose` | Absolute pose from the Trilateration Solver. |
| `/odometry/filtered_uwb` | Fused EKF output (UWB + Wheel Odometry). |
| `/uwb/error_diagnostics` | Breakdown of specific error sources (physical truth). |

---

## 🤝 Contributing & Support

We welcome contributions to UWB error modeling and filtering algorithms.
1. Fork the repo.
2. Create your feature branch (`git checkout -b feature/nlos-rejection`).
3. Commit changes (`git commit -m 'Add NLOS rejection'`).
4. Push to the branch (`git push origin feature/nlos-rejection`).
5. Open a Pull Request.

**License**: Apache 2.0
