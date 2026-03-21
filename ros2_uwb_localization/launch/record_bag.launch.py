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
# ROS Bag Recording Launch
# ------------------------
# Runs the full demo and simultaneously records all relevant topics to a bag.
#
# Usage:
#   ros2 launch ros2_uwb_localization record_bag.launch.py
#   ros2 launch ros2_uwb_localization record_bag.launch.py bag_path:=/tmp/my_run
#
# Topics recorded:
#   /odom               Wheel odometry
#   /uwb/range_*        Raw UWB range measurements (4 anchors)
#   /uwb/pose           Trilateration position estimate
#   /odometry/filtered  EKF fused pose
#   /tf /tf_static      All transforms
#   /uwb/markers        RViz visualization markers
#   /clock              Simulation clock
"""Run the full demo and record all localization topics to a ROS bag."""

from datetime import datetime
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the bag-recording demo LaunchDescription."""
    pkg_localization = get_package_share_directory('ros2_uwb_localization')
    pkg_sim = get_package_share_directory('ros2_uwb_research_sim')
    pkg_sim_prefix = get_package_prefix('ros2_uwb_research_sim')

    plugin_path = os.path.join(pkg_sim_prefix, 'lib', 'ros2_uwb_research_sim')
    default_world = os.path.join(pkg_sim, 'examples', 'example_world.sdf')
    urdf_file = os.path.join(pkg_sim, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_localization, 'rviz', 'uwb_localization.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_bag_path = os.path.expanduser(f'~/ros2_uwb_bags/run_{timestamp}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')
    launch_rviz = LaunchConfiguration('rviz')

    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''), ':', plugin_path],
    )
    set_ld_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[os.environ.get('LD_LIBRARY_PATH', ''), ':', plugin_path],
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', default_world],
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

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    # Record all localization-relevant topics
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_path,
            '/odom',
            '/joint_states',
            '/uwb/range_0',
            '/uwb/range_1',
            '/uwb/range_2',
            '/uwb/range_3',
            '/uwb/pose',
            '/uwb/research_data',
            '/uwb/markers',
            '/odometry/filtered',
            '/tf',
            '/tf_static',
            '/clock',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('bag_path', default_value=default_bag_path,
                              description='Output path for the ROS bag'),
        set_plugin_path,
        set_ld_path,
        gazebo,
        robot_state_publisher,
        ros_gz_bridge,
        localization_launch,
        rviz_node,
        bag_record,
    ])
