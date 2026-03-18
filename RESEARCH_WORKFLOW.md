# UWB Research Workflow: Step-by-Step Guide

This guide provides a sequential workflow for running the full localization pipeline, performing live noise injection, and generating research-grade diagnostic reports.

---

## 1. Workspace Preparation (Foolproof Build)
To avoid "Is a directory" conflicts and dependency-file race conditions, always use this clean build sequence:

```bash
cd ~/ros2_uwb_plugin
# 1. Purge all artifacts
rm -rf build/ install/ log/

# 2. Build everything (tests disabled to avoid race conditions in parallel builds)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# 3. Source the environment
source install/setup.bash
```

> [!NOTE]
> The UWB Plugin now uses a **threaded ROS executor**. This means you can change noise profiles via `ros2 param set` even when Gazebo is **paused**.

---

## 2. Launch Simulation & Pipeline (Terminal A)
This launches Gazebo, the Anchor Manager, Range Preprocessor, Trilateration Solver, EKF Fusion, and RViz2.

```bash
source ~/ros2_uwb_plugin/install/setup.bash
ros2 launch ros2_uwb_research_sim demo.launch.py
```

---

## 3. Record Research Dataset (Terminal B)
Start recording ground truth, raw UWB diagnostics, and estimated poses. This also starts the benchmark node.

```bash
source ~/ros2_uwb_plugin/install/setup.bash
ros2 launch ros2_uwb_localization record_dataset.launch.py
```

---

## 4. Runtime Noise Injection (Terminal C)
While the simulation is running, you can dynamically switch environments.

```bash
# Switch anchor 0 to high-noise warehouse profile
ros2 param set /uwb_plugin_uwb_anchor_0 noise_profile warehouse

# Switch anchor 1 to ideal (zero-noise) profile
ros2 param set /uwb_plugin_uwb_anchor_1 noise_profile ideal

# Back to research defaults
ros2 param set /uwb_plugin_uwb_anchor_0 noise_profile research
```

---

## 5. Post-Process Analysis (After Ctrl+C)
Once you stop the recording (Terminal B), generate the research plots.

```bash
cd ~/ros2_uwb_plugin/ros2_uwb_localization/scripts

# 1. General Accuracy Report (RMSE, MAE, P95)
python3 plot_results.py ../../uwb_benchmark.csv

# 2. Advanced Error Attribution & Correlation
python3 plot_diagnostics.py ../../uwb_benchmark.csv
```

---

## 6. ROS2 Bag Playback & Offline Analysis

The framework supports offline analysis using recorded data.

### Option A: One-Shot Synthetic Demo (No Gazebo Required)
If you want to test the full pipeline without running the simulator:
```bash
source ~/ros2_uwb_plugin/install/setup.bash
ros2 launch ros2_uwb_localization demo_bag.launch.py
```
*This generates a path, records a bag, and immediately replays it through RViz.*

### Option B: Replaying Your Research Datasets
To visualize or re-process a dataset recorded in Step 3:

1. **Start the Visualization & Pipeline**:
   ```bash
   source ~/ros2_uwb_plugin/install/setup.bash
   # Launch RViz and the preprocessor/solver nodes
   ros2 launch ros2_uwb_localization localization.launch.py rviz:=true use_sim_time:=true
   ```

2. **Play the Bag**:
   ```bash
   source ~/ros2_uwb_plugin/install/setup.bash
   # Use --clock to ensure simulated time is used
   ros2 bag play <your_recorded_bag_directory> --clock
   ```

---

## Workflow Summary
1. **Prepare**: Build and source.
2. **Execute**: Launch `demo.launch.py`.
3. **Capture**: Launch `record_dataset.launch.py`.
4. **Perturb**: Use `ros2 param set` to change noise profiles live.
5. **Replay**: Use `ros2 bag play --clock` for offline validation.
6. **Analyze**: Use `plot_diagnostics.py` to generate the final report.
