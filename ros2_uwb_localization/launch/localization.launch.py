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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_localization = FindPackageShare('ros2_uwb_localization')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Config file paths
    localizer_config = PathJoinSubstitution([pkg_localization, 'config', 'localizer.yaml'])
    anchors_config = PathJoinSubstitution([pkg_localization, 'config', 'anchors.yaml'])
    ekf_config = PathJoinSubstitution([pkg_localization, 'config', 'ekf.yaml'])

    # 1. Anchor Manager
    anchor_manager = Node(
        package='ros2_uwb_localization',
        executable='anchor_manager_node',
        name='anchor_manager',
        parameters=[anchors_config, {'use_sim_time': use_sim_time}]
    )

    # 2. Modular Localization Pipeline
    preprocessor = Node(
        package='ros2_uwb_localization',
        executable='uwb_range_preprocessor',
        name='uwb_range_preprocessor',
        parameters=[localizer_config, {'use_sim_time': use_sim_time}]
    )

    solver = Node(
        package='ros2_uwb_localization',
        executable='uwb_trilateration_solver',
        name='uwb_trilateration_solver',
        parameters=[localizer_config, anchors_config, {'use_sim_time': use_sim_time}]
    )

    # 3. EKF Fusion (robot_localization)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered_uwb')]
    )

    # 4. Visualization
    visualization = Node(
        package='ros2_uwb_localization',
        executable='uwb_visualization_node',
        name='uwb_visualization',
        parameters=[
            anchors_config,
            PathJoinSubstitution([pkg_localization, 'config', 'visualization.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    ld.add_action(anchor_manager)
    ld.add_action(preprocessor)
    ld.add_action(solver)
    ld.add_action(ekf)
    ld.add_action(visualization)

    return ld
