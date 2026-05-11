"""Simulation bringup using Nav2 and optional SLAM."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('inspection_robot')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    world_file = os.path.join(pkg_share, 'worlds', 'inspection.world')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    camera_calibration_file = os.path.join(pkg_share, 'config', 'camera_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    map_file = LaunchConfiguration('map', default='')

    existing_gmp = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_gmp = os.pathsep.join([
        os.path.join(pkg_share, 'models'),
        os.path.join(tb3_gazebo_share, 'models'),
        existing_gmp,
    ])

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle_pi',
            '-file', os.path.join(tb3_gazebo_share, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0',
        ],
        output='screen',
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params}.items(),
        condition=IfCondition(use_slam),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map_file, 'params_file': nav2_params}.items(),
        condition=UnlessCondition(use_slam),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params}.items(),
    )

    aruco = Node(
        package='inspection_robot',
        executable='aruco_detector',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'camera_calibration_file': camera_calibration_file,
        }],
        output='screen',
    )
    nav2_waypoints = Node(
        package='inspection_robot',
        executable='nav2_waypoint_follower',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'waypoints_file': waypoints_file,
            'global_frame': 'map',
            'autostart': True,
        }],
        output='screen',
    )
    logger = Node(
        package='inspection_robot',
        executable='report_logger',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_gmp),
        gzserver, gzclient, robot_state_publisher, spawn_entity,
        slam, localization, navigation,
        aruco, nav2_waypoints, logger,
    ])
