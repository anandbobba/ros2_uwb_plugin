# ros2_uwb_plugin — Command Reference

Quick reference for building, testing, running, and benchmarking the plugin.
Source ROS2 and the workspace before running any command:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_uwb_plugin/install/setup.bash
```

---

## Build

```bash
cd ~/ros2_uwb_plugin

# Full build (both packages)
colcon build --symlink-install

# Single package (faster iteration)
colcon build --symlink-install --packages-select ros2_uwb_localization
colcon build --symlink-install --packages-select ros2_uwb_research_sim
```

---

## Unit Tests

```bash
cd ~/ros2_uwb_plugin

# Build + run all tests
colcon test --packages-select ros2_uwb_localization ros2_uwb_research_sim \
            --return-code-on-test-failure

# Show results (verbose pass/fail per test)
colcon test-result --verbose

# Run a single test binary directly
./build/ros2_uwb_localization/test_trilateration
./build/ros2_uwb_localization/test_anchor_manager
./build/ros2_uwb_localization/test_covariance
./build/ros2_uwb_research_sim/test_noise_models
./build/ros2_uwb_research_sim/test_channel_edge_cases

# Run with a filter (example: only covariance tests)
./build/ros2_uwb_localization/test_covariance --gtest_filter='Covariance.*'
```

**Expected:** 39 tests pass (0 failures)
| Binary | Tests |
|---|---|
| test_trilateration | 9 |
| test_anchor_manager | 10 |
| test_covariance | 9 |
| test_noise_models | 17 |
| test_channel_edge_cases | 11 |

---

## Simulation (Ignition Gazebo)

```bash
# Basic GUI simulation (4 anchors + robot)
ros2 launch ros2_uwb_research_sim basic_sim.launch.py

# Headless simulation (CI / SSH — no GUI)
ros2 launch ros2_uwb_research_sim headless_sim.launch.py

# Verify range data
ros2 topic echo /uwb/range_0
ros2 topic echo /uwb/research_data        # ground truth + all noise components

# Drive the robot (in a second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Full Demo (Gazebo + Localization + RViz)

```bash
ros2 launch ros2_uwb_localization demo.launch.py
```

---

## ROS Bag — Record

```bash
# Record a bag while the full demo is running (Gazebo required)
ros2 launch ros2_uwb_localization record_bag.launch.py

# Custom output path
ros2 launch ros2_uwb_localization record_bag.launch.py \
    bag_path:=~/my_uwb_run
```

Topics recorded: `/odom`, `/uwb/range_0..3`, `/uwb/pose`, `/uwb/research_data`,
`/uwb/markers`, `/odometry/filtered`, `/tf`, `/tf_static`, `/clock`.

---

## ROS Bag — Play Back

```bash
# Play a previously recorded bag
ros2 launch ros2_uwb_localization play_bag.launch.py \
    bag_path:=~/ros2_uwb_bags/run_<timestamp>

# Half-speed playback
ros2 launch ros2_uwb_localization play_bag.launch.py \
    bag_path:=~/ros2_uwb_bags/run_<timestamp> rate:=0.5

# List available bags
ls ~/ros2_uwb_bags/
```

---

## ROS Bag — Synthetic Demo (no Gazebo)

```bash
# Step 1: Generate a 60 s synthetic demo bag (circular path)
ros2 run ros2_uwb_localization generate_demo_bag.py

# Custom output / duration
ros2 run ros2_uwb_localization generate_demo_bag.py \
    --output ~/my_demo_bag --duration 120 --radius 4.0

# Step 2: Play the bag through localization + RViz
ros2 launch ros2_uwb_localization play_bag.launch.py \
    bag_path:=~/ros2_uwb_demo_bag

# One-shot: generate + play automatically
ros2 launch ros2_uwb_localization demo_bag.launch.py
ros2 launch ros2_uwb_localization demo_bag.launch.py rviz:=false  # headless
```

---

## Benchmarks

```bash
# Noise model parametric benchmark (8 configurations)
ros2 run ros2_uwb_research_sim benchmark.py
ros2 run ros2_uwb_research_sim benchmark.py --duration 30 --output /tmp/bench

# Position accuracy offline benchmark — no ROS runtime required!
python3 ~/ros2_uwb_plugin/ros2_uwb_research_sim/scripts/position_accuracy_benchmark.py

# Or after install:
ros2 run ros2_uwb_research_sim position_accuracy_benchmark.py

# Custom sweep
python3 position_accuracy_benchmark.py --iters 2000 --seed 123 \
    --output ~/accuracy_results.csv
```

---

## Research / Data Collection

```bash
# Start the researcher node (logs all noise components to CSV)
ros2 run ros2_uwb_research_sim researcher_node.py

# Launch full experiment pipeline (requires gazebo)
ros2 run ros2_uwb_research_sim runner.py experiments/full_mode.yaml

# Plot results from a CSV
ros2 run ros2_uwb_research_sim plotter.py /path/to/results.csv /tmp/plots/
```

---

## Verify Build is Clean

```bash
cd ~/ros2_uwb_plugin
colcon build --symlink-install 2>&1 | grep -E "warning:|error:" | grep -v "^$" | wc -l
# Expected output: 0
```
