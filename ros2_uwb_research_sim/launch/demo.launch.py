import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    pkg_loc = FindPackageShare('ros2_uwb_localization')
    pkg_sim = FindPackageShare('ros2_uwb_research_sim')

    # Arguments
    noise_profile = LaunchConfiguration('noise_profile', default='indoor')
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
