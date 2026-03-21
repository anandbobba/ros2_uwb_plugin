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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_loc = FindPackageShare('ros2_uwb_localization')

    # Arguments
    rviz = LaunchConfiguration('rviz', default='true')

    # 1. Standard Simulation + Localization Launch
    # We leverage the existing modular launch files but override key params here
    main_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_loc, 'launch', 'demo.launch.py'])
        ]),
        launch_arguments={
            'launch_rviz': rviz,
            'use_sim_time': 'true'
        }.items()
    )

    # Note: We can add specific overrides for noise profiles here or in the world file
    # For this demo, we assume the world file has the desired UWB plugins.

    ld.add_action(main_demo)

    return ld
