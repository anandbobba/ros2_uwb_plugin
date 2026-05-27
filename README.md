# ROS2 UWB Localization Framework

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange)](https://gazebosim.org/home)
[![Build Status](https://github.com/anandbobba/ros2_uwb_plugin/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/anandbobba/ros2_uwb_plugin/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

A modular, research-grade UWB localization framework for ROS2 and Gazebo.

---

## 🎥 Demo

[▶️ Watch Demo](https://drive.google.com/file/d/1kJ0pV0zVk8k41X8jCc-OAbcPqFgLc1oQ/view?usp=drive_link)

---

## 🛠 Features

- **High-Fidelity Simulation**: Gazebo plugin with modular error models (Gaussian, NLOS Bias, Multipath, Clock Drift).
- **Position Estimation**: Iterative Weighted Gauss-Newton Trilateration supporting 2D and 3D modes.
- **Sensor Fusion**: Multi-sensor fusion via `robot_localization` EKF (UWB + Odometry).
- **Advanced Diagnostics**: Real-time error attribution and signal propagation analysis.
- **Hardware Agnostic**: Generic serial interface for real UWB sensors with built-in Mock Mode.

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

### 1. Installation
```bash
# Create workspace
mkdir -p ~/ros2_uwb_ws/src && cd ~/ros2_uwb_ws/src
git clone https://github.com/anandbobba/ros2_uwb_plugin.git .
cd ..

# Install dependencies using rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Optional: Automated system setup
# chmod +x src/install_dependencies.sh && ./src/install_dependencies.sh

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Launch Simulation
Launch the simulator, localization pipeline, and automated research autopilot:
```bash
# Recommended for stability (CycloneDDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ros2_uwb_research_sim demo.launch.py
```
*Note: In Gazebo, click the "Play" button to start the simulation.*

---

## 📊 Research Workflow

### Dataset Recording
Capture ranges, filtered poses, and diagnostic metrics for offline evaluation:
```bash
ros2 launch ros2_uwb_localization record_dataset.launch.py
```

### Performance Analytics
Generate error distribution plots and attribution reports from recorded CSV data:
```bash
# Trajectory and RMSE analysis
python3 ros2_uwb_localization/scripts/plot_results.py <dataset>.csv

# Signal error attribution (NLOS vs. Noise)
python3 ros2_uwb_localization/scripts/plot_diagnostics.py <dataset>.csv
```

### Runtime Configuration
Dynamically modify environment complexity via ROS2 parameters:
```bash
ros2 param set /uwb_plugin_uwb_anchor_0 nlos_prob 0.8
```

---

## 🔌 Real Hardware Usage

The framework includes a generic serial driver to bridge physical UWB tags (Decawave, LinkTrack, etc.) into the pipeline.

1.  **Configure**: Update port settings in `ros2_uwb_drivers/config/uwb_driver.yaml`.
2.  **Launch Driver**:
    ```bash
    ros2 launch ros2_uwb_drivers uwb_driver.launch.py
    ```
3.  **Run Pipeline**:
    ```bash
    ros2 launch ros2_uwb_localization localization.launch.py
    ```

---

## 📁 Repository Structure

- `ros2_uwb_msgs`: Custom interfaces for range aggregation and error diagnostics.
- `ros2_uwb_localization`: Core positioning engine, preprocessor, and benchmark nodes.
- `ros2_uwb_research_sim`: Gazebo simulator, robot models, and research plugin.
- `ros2_uwb_drivers`: Generic serial bridge for hardware integration.

---

## 📡 Topics Output

| Topic | Description |
|---|---|
| `/uwb/range` | Incoming range measurements (Sim or Hardware). |
| `/uwb/pose` | Pure UWB absolute position estimate. |
| `/odometry/filtered_uwb` | Fused EKF output (UWB + Odometry). |
| `/uwb/error_diagnostics` | Real-time breakdown of noise sources (Truth vs Measured). |

---

## Dual Tag Yaw Estimation

The `ros2_uwb_plugin` now supports estimating the robot's yaw (heading) using two UWB tags mounted on the front and rear of the robot. This is crucial for environments where IMU or odometry-based heading is unreliable.

### Why Yaw Matters
Trilateration with a single UWB tag provides `(x, y, z)` position but no orientation. Without orientation, EKF fusion heavily relies on the IMU or odometry to maintain heading, which can drift over time. Dual UWB tags provide a drift-free absolute heading measurement.

### Mathematical Derivation
The yaw is computed simply using the `atan2` function on the relative positions of the front and rear tags:
```cpp
double dx = front.x - rear.x;
double dy = front.y - rear.y;
double yaw = atan2(dy, dx);
```
An Exponential Moving Average (EMA) filter is applied to the sine and cosine components of the raw yaw to smooth the output while avoiding wrap-around issues.

### Integration Guide
1. **Physical Setup**: Mount two UWB tags on the robot, aligned with the longitudinal axis (front-to-back).
2. **Configuration**: Measure the distance between the tags and update the `baseline_expected` parameter in `config/localizer.yaml`.
3. **Launch**: Use the `dual_tag:=true` argument to launch the dual-tag pipeline.

### Example Launch Commands
**Single Tag Mode (Default)**
```bash
ros2 launch ros2_uwb_localization localization.launch.py
```

**Dual Tag Mode**
```bash
ros2 launch ros2_uwb_localization localization.launch.py dual_tag:=true
```

## Architecture Diagram (Dual Tag)

```text
[Front Ranges] -> Preprocessor -> Solver -> /uwb/front/pose \
                                                             -> Yaw Estimator -> /uwb/pose (x,y,yaw,cov) -> EKF
[Rear Ranges]  -> Preprocessor -> Solver -> /uwb/rear/pose  /        ^
                                                                     |
[IMU (/imu/data)] ---------------------------------------------------+
```

### Advanced Yaw Features (NEXT STAGE)
1. **Dynamic Confidence & Covariance**: 
   The framework dynamically scales the yaw covariance based on the distortion of the measured baseline against the `baseline_expected`. If tags drop out, the node holds the last known yaw and gradually inflates the covariance.
2. **IMU Complementary Fusion**:
   If `use_imu_fusion` is enabled, the estimator uses a complementary filter to pull high-frequency IMU yaw toward the drift-free UWB yaw absolute reference, preventing angular drift.
3. **Advanced RViz Visualization**:
   - Color-coded confidence arrows (Green=Stable, Yellow=Uncertain, Red=Poor)
   - Dynamic covariance ellipses
   - Real-time front-to-rear baseline rendering
4. **Hardware Driver Extension**:
   To pass dual hardware tags on a single serial port, the driver supports a 4-element CSV output: `TAG_ID,ANCHOR_ID,RANGE,RSSI`. Configure `front_tag_id` and `rear_tag_id` in `uwb_driver.yaml`.

---

## 🛠 Troubleshooting

### Topic Connectivity & Missing Transforms
If you see "No transform" errors in RViz or empty topics, clear the Shared Memory locks and switch to CycloneDDS:
```bash
rm -rf /dev/shm/fastrtps*
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Gazebo Stability
If simulation crashes on startup, ensure you have ran a clean build:
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

**License**: Apache 2.0
