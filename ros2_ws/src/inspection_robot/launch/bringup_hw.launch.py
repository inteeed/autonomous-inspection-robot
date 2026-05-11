"""Hardware bringup path for TurtleBot3 plus inspection nodes."""
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
    tb3_bringup_share = get_package_share_directory('turtlebot3_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    localization_params = os.path.join(pkg_share, 'config', 'localization.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    camera_calibration_file = os.path.join(pkg_share, 'config', 'camera_hw.yaml')

    use_slam = LaunchConfiguration('use_slam', default='false')
    map_file = LaunchConfiguration('map', default='')
    camera_package = LaunchConfiguration('camera_package', default='v4l2_camera')
    camera_executable = LaunchConfiguration('camera_executable', default='v4l2_camera_node')

    tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_bringup_share, 'launch', 'robot.launch.py')),
    )
    camera = Node(
        package=camera_package,
        executable=camera_executable,
        output='screen',
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'slam_params_file': slam_params}.items(),
        condition=IfCondition(use_slam),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'map': map_file, 'params_file': localization_params}.items(),
        condition=UnlessCondition(use_slam),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params}.items(),
    )

    aruco = Node(
        package='inspection_robot',
        executable='aruco_detector',
        parameters=[{
            'use_sim_time': False,
            'camera_calibration_file': camera_calibration_file,
            'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info',
        }],
        output='screen',
    )
    nav2_waypoints = Node(
        package='inspection_robot',
        executable='nav2_waypoint_follower',
        parameters=[{
            'use_sim_time': False,
            'waypoints_file': waypoints_file,
            'global_frame': 'map',
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
        }],
        output='screen',
    )
    logger = Node(
        package='inspection_robot',
        executable='report_logger',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('camera_package', default_value='v4l2_camera'),
        DeclareLaunchArgument('camera_executable', default_value='v4l2_camera_node'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        tb3, camera, slam, localization, navigation,
        aruco, nav2_waypoints, logger,
    ])
