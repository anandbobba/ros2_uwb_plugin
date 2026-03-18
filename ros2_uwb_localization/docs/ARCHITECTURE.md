# System Architecture

## Overview

`ros2_uwb_localization` is a modular ROS2 package implementing a complete UWB
indoor localization pipeline. The system is designed around the principle of
**separation of concerns**: each node handles one responsibility and
communicates through standard ROS2 interfaces.

## Node Graph

```
                                  ┌─────────────────┐
                                  │  anchors.yaml   │
                                  └────────┬────────┘
                                           │ (parameters)
                    ┌──────────────────────┬┼──────────────────────┐
                    ▼                      ▼▼                      ▼
          ┌─────────────────┐   ┌──────────────────┐   ┌───────────────────┐
          │ Anchor Manager  │   │  Trilateration   │   │  Visualization    │
          │                 │   │     Node         │   │     Node          │
          │ • Load anchors  │   │ • Gauss-Newton   │   │ • Anchor markers  │
          │ • Broadcast TF  │   │ • NLOS rejection │   │ • Range circles   │
          └─────────────────┘   │ • Covariance     │   │ • Labels          │
                                └────────┬─────────┘   └───────────────────┘
                                         │
                                    /uwb/pose
                                         │
                                         ▼
                              ┌─────────────────────┐
                              │    EKF Fusion Node   │
                              │  (robot_localization)│
                              │                      │
                              │  /uwb/pose ────►     │
                              │  /odom ────────►     │──► /odometry/filtered
                              │  /imu/data ───►      │
                              └─────────────────────┘
```

## Data Flow

### Range Measurement Pipeline

1. **Source**: Gazebo UWB Plugin (`ros2_uwb_research_sim`) or real UWB hardware
2. **Transport**: `sensor_msgs/msg/Range` on `/uwb/range_N`
3. **Processing**: `trilateration_node` collects all ranges and solves for position
4. **Output**: `geometry_msgs/msg/PoseWithCovarianceStamped` on `/uwb/pose`

### Sensor Fusion Pipeline

1. **Inputs**: `/uwb/pose` (position), `/odom` (velocity), `/imu/data` (orientation)
2. **Filter**: Extended Kalman Filter via `robot_localization`
3. **Output**: `nav_msgs/msg/Odometry` on `/odometry/filtered`

## Trilateration Algorithm

The solver uses iterative Gauss-Newton optimization:

**Objective**: Minimize the sum of squared range residuals:

$$\min_x \sum_{i=1}^{N} (r_i - \|x - a_i\|)^2$$

where $r_i$ is the measured range to anchor $a_i$ and $x$ is the estimated
position.

**Jacobian**: For each anchor $i$:

$$J_i = \frac{(x - a_i)^T}{\|x - a_i\|}$$

**Update step**:

$$\Delta x = (J^T J)^{-1} J^T r$$

**Covariance propagation**:

$$\Sigma_{pos} = \sigma_r^2 (J^T J)^{-1}$$

### NLOS Rejection

After each solver iteration, residuals are checked:

$$|r_i - \|x_{est} - a_i\|| > \tau_{outlier}$$

Anchors exceeding the threshold are flagged as outliers and excluded from
subsequent iterations. This handles the positive bias introduced by NLOS
propagation paths.

## Frame Conventions

```
map
 └── uwb_world (identity transform)
 └── uwb_anchor_0 (from anchor_manager)
 └── uwb_anchor_1
 └── uwb_anchor_2
 └── uwb_anchor_3
 └── odom (from EKF or odometry)
      └── base_link (from robot URDF)
```

## Parameter Sharing

All nodes that need anchor positions load them from the same parameter
structure. The `anchors.yaml` config file is passed to each node at launch
time, ensuring consistency without inter-node communication overhead.

## Design Decisions

1. **No custom messages** — Uses only standard ROS2 message types for maximum
   interoperability. Any UWB hardware that publishes `sensor_msgs/Range` works.

2. **Parameter-based anchor loading** — Anchors are ROS2 parameters rather than
   a service-based discovery protocol. This is simpler, deterministic, and
   works with launch file composition.

3. **Separate anchor manager** — Even though trilateration loads anchors
   directly from parameters, the anchor manager provides TF frames that other
   tools (RViz, navigation) can use.

4. **robot_localization for EKF** — Rather than implementing a custom filter,
   we delegate to the battle-tested `robot_localization` package, which handles
   15-state EKF with proper time handling and sensor rejection.
