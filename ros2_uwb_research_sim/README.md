# ros2_uwb_research_sim

**High-fidelity UWB sensor simulation for Gazebo / Ignition.**

This package provides a professional-grade Ignition Gazebo plugin for modeling UWB range sensors with complex physical error characteristics. It is designed for researchers and engineers developing robust localization algorithms.

## 🚀 Features

- **Standardized Output** — Publishes `ros2_uwb_msgs/UWBRange` messages for plug-and-play use with the localization stack.
- **Physical Error Models**:
  - **Gaussian Noise**: Static standard deviation model.
  - **NLOS Bias**: Exponentially distributed bias for non-line-of-sight conditions.
  - **Multipath AR(1)**: Correlated temporal noise.
  - **Clock Drift**: Random walk drift modeling for unsynchronized anchors.
- **Research Data Node**: Publishes ground-truth vs. measured data on a dedicated topic for analysis.

## 🚥 Quick Start

### Basic Simulation
```bash
ros2 launch ros2_uwb_research_sim sim.launch.py
```

## ⚙️ Plugin Configuration

Add the plugin to your robot's model (URDF or SDF):

```xml
<plugin name="ros2_uwb_research_sim::UWBPlugin" filename="libUWBPlugin.so">
  <target_entity>uwb_anchor_0</target_entity>
  <update_rate>20.0</update_rate>
  <gaussian_std>0.05</gaussian_std>
  <nlos_prob>0.1</nlos_prob>
  <nlos_lambda>2.0</nlos_lambda>
</plugin>
```

## 📂 Package Structure

- **`include/ros2_uwb_research_sim/`**: Modular error model headers.
- **`src/`**: Implementation of models and the Gazebo plugin.
- **`scripts/`**: Research tools for plotting and offline benchmarking.
- **`worlds/`**: Demo worlds with predefined anchor layouts.
