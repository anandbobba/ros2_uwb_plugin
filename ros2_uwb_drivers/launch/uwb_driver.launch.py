import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_uwb_drivers')
    default_config_path = os.path.join(pkg_share, 'config', 'uwb_driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Path to the driver configuration file'
        ),

        Node(
            package='ros2_uwb_drivers',
            executable='uwb_serial_driver',
            name='uwb_serial_driver',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
