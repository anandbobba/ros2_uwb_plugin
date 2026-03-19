#!/bin/bash
# ROS2 UWB Framework - Dependency Installer
# Copyright 2026 Anand Bobba

set -e

echo "Updating system packages..."
sudo apt update

echo "Installing ROS2 Humble, Gazebo, and Python dependencies..."
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  libignition-gazebo6-dev \
  python3-matplotlib \
  python3-pandas \
  python3-numpy

echo "Updating rosdep..."
rosdep update

echo "Installing package-specific dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "Installation complete! You can now build the workspace with 'colcon build'."
