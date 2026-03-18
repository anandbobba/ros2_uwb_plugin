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
# Demo Bag Launch
# ---------------
# One-shot launch: generates a synthetic UWB bag (no Gazebo), then
# immediately plays it back through the full localization pipeline + RViz.
#
# Useful for contributors who don't have Ignition Gazebo installed.
#
# Usage:
#   ros2 launch ros2_uwb_localization demo_bag.launch.py
#   ros2 launch ros2_uwb_localization demo_bag.launch.py rviz:=false
#   ros2 launch ros2_uwb_localization demo_bag.launch.py bag_path:=/tmp/demo_bag

"""Generate a synthetic UWB bag then play it through the localization pipeline."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the demo-bag LaunchDescription."""
    pkg_localization = get_package_share_directory('ros2_uwb_localization')
    pkg_localization_prefix = get_package_prefix('ros2_uwb_localization')
    pkg_sim = get_package_share_directory('ros2_uwb_research_sim')

    urdf_file = os.path.join(pkg_sim, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_localization, 'rviz', 'uwb_localization.rviz')
    gen_bag_script = os.path.join(
        pkg_localization_prefix, 'lib', 'ros2_uwb_localization', 'generate_demo_bag.py')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_bag = os.path.expanduser(f'~/ros2_uwb_demo_bag_{timestamp}')

    bag_path = LaunchConfiguration('bag_path')
    launch_rviz = LaunchConfiguration('rviz')
    duration = LaunchConfiguration('duration')

    # --- Step 1: Generate the synthetic bag ---
    generate_bag = ExecuteProcess(
        cmd=[
            'python3', gen_bag_script,
            '--output', bag_path,
            '--duration', duration,
        ],
        output='screen',
    )

    # --- Step 2 (delayed): robot_state_publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    # --- Step 3 (delayed): localization pipeline ---
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # --- Step 4 (delayed): RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    # Play bag after a short delay to allow nodes to start
    bag_play = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag_path,
                    '--rate', '1.0',
                    '--clock',
                    '--topics',
                    '/odom',
                    '/uwb/range',  # Standardized UWBRange topic
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path', default_value=default_bag,
            description='Output directory for the generated demo bag'),
        DeclareLaunchArgument(
            'duration', default_value='60.0',
            description='Synthetic bag duration in seconds (default: 60)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz visualization'),
        LogInfo(msg=[
            '\n',
            '=== UWB Demo Bag Launch ===\n',
            'Step 1: Generating synthetic bag (robot on circular path)...\n',
            'Step 2: Starting localization pipeline + RViz...\n',
            'Step 3: Playing bag (5 s delay for nodes to initialise).\n',
        ]),
        generate_bag,
        robot_state_publisher,
        joint_state_publisher,
        localization_launch,
        rviz_node,
        bag_play,
    ])
