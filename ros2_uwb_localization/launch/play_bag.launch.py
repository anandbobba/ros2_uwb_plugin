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
# ROS Bag Playback Launch
# -----------------------
# Plays back a recorded bag alongside the localization pipeline and RViz.
# No simulator needed — bag provides all sensor data.
#
# Usage:
#   ros2 launch ros2_uwb_localization play_bag.launch.py bag_path:=/path/to/bag
#   ros2 launch ros2_uwb_localization play_bag.launch.py \
#     bag_path:=~/ros2_uwb_bags/run_20260311 rate:=0.5
#
# Tip: list available bags with:
#   ls ~/ros2_uwb_bags/
"""Play back a recorded ROS bag alongside the localization pipeline and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_bag_play(context, *args, **kwargs):
    bag_path = os.path.expanduser(LaunchConfiguration('bag_path').perform(context))
    rate = LaunchConfiguration('rate').perform(context)
    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', bag_path,
                '--rate', rate,
                '--clock',
                # Exclude /tf and /tf_static: these were recorded from Gazebo with
                # sim-time header stamps (0–41s) that conflict with the clock published
                # by --clock (based on wall-clock recording timestamps).  The live EKF
                # and robot_state_publisher regenerate all necessary TF at runtime.
                '--topics',
                '/odom',
                '/uwb/range_0', '/uwb/range_1', '/uwb/range_2', '/uwb/range_3',
                '/uwb/research_data',
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    """Build and return the bag playback LaunchDescription."""
    pkg_localization = get_package_share_directory('ros2_uwb_localization')
    pkg_sim = get_package_share_directory('ros2_uwb_research_sim')

    urdf_file = os.path.join(pkg_sim, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_localization, 'rviz', 'uwb_localization.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    launch_rviz = LaunchConfiguration('rviz')

    bag_play = OpaqueFunction(function=_make_bag_play)

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

    # Publish default (zero) joint states — /joint_states was not recorded in the
    # bag, so robot_state_publisher would otherwise produce no base_footprint→base_link TF.
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

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

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

    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
                              description='Path to the ROS bag directory to play back'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Playback speed multiplier (e.g. 0.5 = half speed)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz visualization'),
        robot_state_publisher,
        joint_state_publisher,
        localization_launch,
        rviz_node,
        bag_play,
    ])
