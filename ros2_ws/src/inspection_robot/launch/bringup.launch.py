"""Bring up the full inspection demo:

  1. Set TURTLEBOT3_MODEL=waffle_pi (camera-equipped variant).
  2. Extend GAZEBO_MODEL_PATH so Gazebo finds our marker models.
  3. Launch gzserver+gzclient with our custom inspection.world.
  4. Spawn turtlebot3 robot_state_publisher + spawn_entity (turtlebot3_gazebo
     already provides robot_state_publisher.launch.py and a spawner).
  5. Start the three inspection nodes: aruco_detector, waypoint_follower,
     report_logger.
  6. Optionally start RViz with the /inspection/annotated image view.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('inspection_robot')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_share, 'worlds', 'inspection.world')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    camera_calibration_file = os.path.join(pkg_share, 'config', 'camera_sim.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'inspection.rviz')

    # Prepend our models dir to GAZEBO_MODEL_PATH so <include><uri>model://aruco_marker_N</uri>
    # in inspection.world resolves correctly.
    existing_gmp = os.environ.get('GAZEBO_MODEL_PATH', '')
    our_models = os.path.join(pkg_share, 'models')
    tb3_models = os.path.join(tb3_gazebo_share, 'models')
    new_gmp = os.pathsep.join([p for p in (our_models, tb3_models, existing_gmp) if p])

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    set_gmp = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_gmp)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    publish_annotated = LaunchConfiguration('publish_annotated', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # gazebo_ros offers gzserver.launch.py and gzclient.launch.py — use them.
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
    )

    # Reuse turtlebot3 robot_state_publisher (publishes /tf for the URDF).
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn the robot at the origin via gazebo_ros's spawn_entity.py service.
    spawn_pose = ['-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0']
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle_pi',
                   '-file', os.path.join(tb3_gazebo_share, 'models',
                                         'turtlebot3_waffle_pi', 'model.sdf'),
                   *spawn_pose],
        output='screen',
    )

    aruco = Node(
        package='inspection_robot', executable='aruco_detector', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'camera_calibration_file': camera_calibration_file,
            'publish_annotated': ParameterValue(publish_annotated, value_type=bool),
            'min_detection_confidence': 0.6,
            'required_consecutive_detections': 2,
        }],
    )
    follower = Node(
        package='inspection_robot', executable='waypoint_follower', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'waypoints_file': waypoints_file,
        }],
    )
    logger = Node(
        package='inspection_robot', executable='report_logger', output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_annotated', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        set_model, set_gmp,
        gzserver, gzclient,
        robot_state_publisher, spawn_entity,
        aruco, follower, logger, rviz,
    ])
