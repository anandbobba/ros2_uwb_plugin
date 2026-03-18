# 🏁 Final Release Checklist

Pre-flight verification for the UWB Localization Framework.

## 1. 🏗️ Build Integrity
- [ ] Clean build passes: `rm -rf build install log && colcon build`
- [ ] All 3 packages (`msgs`, `localization`, `research_sim`) are present.
- [ ] `plot_results.py` has executable permissions.

## 2. 📡 Runtime Verification
- [ ] **One-Command Demo**: `ros2 launch ros2_uwb_research_sim demo.launch.py` works.
- [ ] **TF Tree**: `map -> odom -> base_footprint` is connected.
- [ ] **Anchor TF**: `map -> anchor_n` frames are visible in RViz.
- [ ] **Noise Profiles**: `ideal` profile results in < 2cm error.

## 3. 📊 Dataset & Analytics
- [ ] `record_dataset.launch.py` generates both a bag and a CSV.
- [ ] CSV contains `timestamp, error_x, error_y, error_z, error_3d`.
- [ ] `plot_results.py` generates a valid `.png` distribution plot.

## 4. 📖 Documentation
- [ ] Root README contains Mermaid architecture diagram.
- [ ] Quick Start section is accurate.
- [ ] `anchors.yaml` documentation matches code logic.

---
Validated by: Antigravity Engineer
Date: March 2026
