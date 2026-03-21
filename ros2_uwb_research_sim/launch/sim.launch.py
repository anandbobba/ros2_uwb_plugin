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

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the UWB simulation LaunchDescription."""
    # Detect appropriate Gazebo command based on ROS2 distribution
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        gz_executable = 'ign'
        gz_subcommand = 'gazebo'
    else:
        # For Jazzy and beyond, use the modern 'gz sim' convention
        gz_executable = 'gz'
        gz_subcommand = 'sim'

    pkg_uwb_sim = get_package_share_directory('ros2_uwb_research_sim')
    pkg_prefix = get_package_prefix('ros2_uwb_research_sim')

    # Path to the library where libUWBPlugin.so is installed
    plugin_path = os.path.join(pkg_prefix, 'lib', 'ros2_uwb_research_sim')

    world_file = os.path.join(pkg_uwb_sim, 'examples', 'example_world.sdf')
    urdf_file = os.path.join(pkg_uwb_sim, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to the SDF world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Set GZ/IGN Gazebo plugin path so it can find libUWBPlugin.so
        # We set both for maximum cross-version compatibility
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[plugin_path]
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value=[plugin_path]
        ),

        # Set LD_LIBRARY_PATH so the system can find libuwb_models.so
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[os.environ.get('LD_LIBRARY_PATH', ''), ':', plugin_path]
        ),

        # Launch Gazebo Sim (Runtime CLI detection)
        ExecuteProcess(
            cmd=[gz_executable, gz_subcommand, '-r', LaunchConfiguration('world')],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Joint State Publisher (fallback for wheel TFs before Gazebo is ready)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        # Dedicated /clock Bridge (Prevents clock lag by isolating it)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/world/uwb_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            remappings=[('/world/uwb_world/clock', '/clock')],
            output='screen'
        ),

        # General ROS - GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='general_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/tag_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/tag_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[
                ('/model/tag_robot/tf', '/tf'),
                ('/model/tag_robot/joint_state', '/joint_states'),
                ('/ground_truth/odom', '/ground_truth/pose'),
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
