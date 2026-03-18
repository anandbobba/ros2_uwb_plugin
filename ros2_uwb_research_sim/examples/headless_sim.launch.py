# Copyright 2026 Anand Bobba
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Headless Simulation Example
# ---------------------------
# Identical to basic_sim.launch.py but runs Ignition Gazebo without a GUI.
# Useful for CI pipelines, SSH sessions, or container environments.
#
# Usage:
#   ros2 launch ros2_uwb_research_sim headless_sim.launch.py
#   ros2 launch ros2_uwb_research_sim headless_sim.launch.py gaussian_sigma:=0.10
#
# Verify with:
#   ros2 topic echo /uwb/range_0
"""Headless UWB simulation: Gazebo server only (no GUI), suitable for CI / SSH."""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the headless simulation LaunchDescription."""
    pkg_sim = get_package_share_directory('ros2_uwb_research_sim')
    pkg_prefix = get_package_prefix('ros2_uwb_research_sim')

    plugin_path = os.path.join(pkg_prefix, 'lib', 'ros2_uwb_research_sim')
    world_file = os.path.join(pkg_sim, 'examples', 'example_world.sdf')
    urdf_file = os.path.join(pkg_sim, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''), ':', plugin_path],
    )
    set_ld_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[os.environ.get('LD_LIBRARY_PATH', ''), ':', plugin_path],
    )

    # -s: server only (no GUI renderer)
    # -r: start simulation immediately (don't pause on launch)
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        LogInfo(msg=[
            '\n',
            '=== UWB Plugin Headless Simulation ===\n',
            'Running Gazebo server without GUI (suitable for CI / SSH).\n',
            'Verify range data with:\n',
            '  ros2 topic echo /uwb/range_0\n',
            '  ros2 topic echo /uwb/research_data\n',
            'Stop with Ctrl-C.\n',
        ]),
        set_plugin_path,
        set_ld_path,
        gazebo_headless,
        robot_state_publisher,
        ros_gz_bridge,
    ])
