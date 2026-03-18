from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    use_sim_time = LaunchConfiguration('use_sim_time') # This will now refer to the declared argument
    run_rviz = LaunchConfiguration('rviz', default='true')

    # 1. Simulation (Gazebo)
    # This includes the robot, world, and UWB plugin
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_research_sim, 'launch', 'sim.launch.py'])
        ]),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 2. Anchor Manager
    anchor_manager = Node(
        package='ros2_uwb_localization',
        executable='anchor_manager_node',
        name='anchor_manager',
        parameters=[
            PathJoinSubstitution([pkg_localization, 'config', 'anchors.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Modular Localization Pipeline
    localizer_config = PathJoinSubstitution([pkg_localization, 'config', 'localizer.yaml'])
    anchors_config = PathJoinSubstitution([pkg_localization, 'config', 'anchors.yaml'])

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

    # 4. EKF Fusion (robot_localization)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_localization, 'config', 'ekf.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/odometry/filtered', '/odometry/filtered_uwb')]
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

    # 6. Visualization (RViz)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_localization, 'rviz', 'uwb_localization.rviz'])],
        condition=IfCondition(run_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7. Research Autopilot (Circular Motion)
    autopilot = Node(
        package='ros2_uwb_research_sim',
        executable='research_autopilot.py',
        name='research_autopilot',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(sim_launch)
    ld.add_action(anchor_manager)
    ld.add_action(preprocessor)
    ld.add_action(solver)
    ld.add_action(ekf)
    ld.add_action(benchmark)
    ld.add_action(rviz)
    ld.add_action(autopilot)

    return ld
