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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_localization = FindPackageShare('ros2_uwb_localization')
    pkg_research_sim = FindPackageShare('ros2_uwb_research_sim')

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Run RViz visualization if true')

    declare_dual_tag = DeclareLaunchArgument(
        'dual_tag',
        default_value='false',
        description='Enable dual UWB tag yaw estimation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('launch_rviz')
    dual_tag = LaunchConfiguration('dual_tag')

    # 1. Simulation (Gazebo)
    # This includes the robot, world, and UWB plugin
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_research_sim, 'launch', 'sim.launch.py'])
        ]),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 2. Localization Pipeline (Modular)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_localization, 'launch', 'localization.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'dual_tag': dual_tag
        }.items()
    )

    # 5. Benchmarking
    benchmark = Node(
        package='ros2_uwb_localization',
        executable='uwb_benchmark_node',
        name='uwb_benchmark',
        parameters=[
            {'gt_topic': '/ground_truth/pose'},
            {'est_topic': '/uwb/pose'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Note: RViz is now handled inside localization.launch.py, so we don't need to launch it here
    # However, if run_rviz is true, it will be launched by localization.launch.py (which we should probably support there)
    # Actually, localization.launch.py always launches RViz right now.

    # 7. Research Autopilot (Circular Motion)
    autopilot = Node(
        package='ros2_uwb_research_sim',
        executable='research_autopilot.py',
        name='research_autopilot',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_dual_tag)
    ld.add_action(LogInfo(msg=["Launch Argument 'launch_rviz' is: ", run_rviz]))
    ld.add_action(sim_launch)
    ld.add_action(localization_launch)
    ld.add_action(benchmark)
    ld.add_action(autopilot)

    return ld
