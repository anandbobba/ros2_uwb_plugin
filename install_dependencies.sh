#!/bin/bash
# ROS2 UWB Framework - Dependency Installer
# Copyright 2026 Anand Bobba

set -e

echo "Updating system packages..."
sudo apt update

echo "Installing essential build tools..."
sudo apt install -y python3-rosdep \
  python3-colcon-common-extensions \
  ros-humble-rmw-cyclonedds-cpp

echo "Initializing and updating rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

echo "Installing all framework dependencies via rosdep..."
# This now pulls in ros_gz_sim, ros_gz_bridge, and all msg/localization dependencies
rosdep install --from-paths src --ignore-src -r -y

echo "Installation complete! You can now build with 'colcon build'."
