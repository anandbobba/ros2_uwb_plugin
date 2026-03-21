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

from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Create a unique bag name based on timestamp
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    default_bag_name = f'uwb_dataset_{timestamp}'

    bag_name = LaunchConfiguration('bag_name', default=default_bag_name)

    # Topics to record - Standardized list for UWB research
    topics = [
        '/uwb/range',
        '/uwb/ranges_filtered',
        '/uwb/pose',
        '/odometry/filtered_uwb',
        '/odom',
        '/tf',
        '/tf_static',
        '/ground_truth/pose',
        '/uwb/markers',
        '/uwb/error_diagnostics'
    ]

    # 1. Bag Recording Process
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name] + topics,
        output='screen'
    )

    # 2. Benchmark Node (for real-time CSV logging and stats)
    benchmark_node = Node(
        package='ros2_uwb_localization',
        executable='uwb_benchmark_node',
        name='uwb_benchmark_recorder',
        parameters=[
            {'gt_topic': '/ground_truth/pose'},
            {'est_topic': '/uwb/pose'},
            {'log_file': [bag_name, '.csv']},
            {'use_sim_time': False}  # Wall time to avoid /clock deadlock
        ]
    )

    ld.add_action(record_process)
    ld.add_action(benchmark_node)

    return ld
