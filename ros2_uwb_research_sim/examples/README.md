# Simulation Examples

This directory contains self-contained launch files for trying out the UWB
Gazebo plugin quickly.

---

## `basic_sim.launch.py` — Interactive GUI Simulation

Launches Ignition Gazebo with a full GUI window, 4 UWB anchors, and a
differential-drive robot.  Use this for interactive development and demos.

```bash
ros2 launch ros2_uwb_research_sim basic_sim.launch.py
```

Verify range data is flowing:

```bash
ros2 topic echo /uwb/range_0        # Raw range (m) to anchor 0
ros2 topic echo /uwb/research_data  # Ground truth + all noise components
```

Drive the robot (requires `teleop_twist_keyboard`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## `headless_sim.launch.py` — Server-Only (No GUI)

Runs the Gazebo server with `-s` (no renderer).  Ideal for CI pipelines, SSH
sessions, or Docker containers without a display.

```bash
ros2 launch ros2_uwb_research_sim headless_sim.launch.py
```

Same topic verification commands as above apply.

---

## `example_world.sdf`

Ignition Gazebo SDF world used by both launch files. Contains:
- 4 UWB anchor models with the plugin attached
- A differential-drive robot model
- Default Gaussian noise `σ = 0.05 m`, 10% NLOS probability

Customise noise parameters directly in this file or override via launch
arguments if using substitutions.

---

## Noise Parameter Reference

| SDF Parameter       | Description                              | Default |
|---------------------|------------------------------------------|---------|
| `gaussian_std`      | Gaussian noise std dev (m)               | 0.05    |
| `nlos_prob`         | Probability of NLOS event (0–1)          | 0.10    |
| `nlos_lambda`       | NLOS exponential bias rate (1/m)         | 2.0     |
| `multipath_alpha`   | AR(1) multipath coefficient (0–1)        | 0.80    |
| `multipath_std`     | Multipath process std dev (m)            | 0.02    |
| `clock_drift_std`   | Clock drift std dev per update (m/step)  | 0.001   |
