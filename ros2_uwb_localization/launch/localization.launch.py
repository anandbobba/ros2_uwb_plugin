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

from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    pkg_localization = FindPackageShare('ros2_uwb_localization')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    dual_tag = LaunchConfiguration('dual_tag', default='false')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'dual_tag',
        default_value='false',
        description='Enable dual UWB tag yaw estimation'
    ))

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

    # 2. Single Tag Pipeline
    preprocessor_single = Node(
        package='ros2_uwb_localization',
        executable='uwb_range_preprocessor',
        name='uwb_range_preprocessor',
        parameters=[localizer_config, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(dual_tag)
    )

    solver_single = Node(
        package='ros2_uwb_localization',
        executable='uwb_trilateration_solver',
        name='uwb_trilateration_solver',
        parameters=[localizer_config, anchors_config, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(dual_tag)
    )

    # 3. Dual Tag Pipeline
    preprocessor_front = Node(
        package='ros2_uwb_localization',
        executable='uwb_range_preprocessor',
        name='uwb_range_preprocessor_front',
        parameters=[localizer_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/uwb/range', '/uwb/front/range'),
            ('/uwb/ranges_filtered', '/uwb/front/ranges_filtered')
        ],
        condition=IfCondition(dual_tag)
    )

    solver_front = Node(
        package='ros2_uwb_localization',
        executable='uwb_trilateration_solver',
        name='uwb_trilateration_solver_front',
        parameters=[localizer_config, anchors_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/uwb/ranges_filtered', '/uwb/front/ranges_filtered'),
            ('/uwb/pose', '/uwb/front/pose')
        ],
        condition=IfCondition(dual_tag)
    )

    preprocessor_rear = Node(
        package='ros2_uwb_localization',
        executable='uwb_range_preprocessor',
        name='uwb_range_preprocessor_rear',
        parameters=[localizer_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/uwb/range', '/uwb/rear/range'),
            ('/uwb/ranges_filtered', '/uwb/rear/ranges_filtered')
        ],
        condition=IfCondition(dual_tag)
    )

    solver_rear = Node(
        package='ros2_uwb_localization',
        executable='uwb_trilateration_solver',
        name='uwb_trilateration_solver_rear',
        parameters=[localizer_config, anchors_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/uwb/ranges_filtered', '/uwb/rear/ranges_filtered'),
            ('/uwb/pose', '/uwb/rear/pose')
        ],
        condition=IfCondition(dual_tag)
    )

    yaw_estimator = Node(
        package='ros2_uwb_localization',
        executable='uwb_yaw_estimator',
        name='uwb_yaw_estimator',
        parameters=[localizer_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(dual_tag)
    )

    # 4. EKF Fusion (robot_localization)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered_uwb')]
    )

    # 5. Visualization
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
    ld.add_action(preprocessor_single)
    ld.add_action(solver_single)
    ld.add_action(preprocessor_front)
    ld.add_action(solver_front)
    ld.add_action(preprocessor_rear)
    ld.add_action(solver_rear)
    ld.add_action(yaw_estimator)
    ld.add_action(ekf)
    ld.add_action(visualization)

    return ld
