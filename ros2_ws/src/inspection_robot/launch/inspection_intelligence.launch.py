"""Optional inspection intelligence nodes."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('inspection_robot')
    anomaly_params = os.path.join(pkg_share, 'config', 'anomaly.yaml')
    scheduler_params = os.path.join(pkg_share, 'config', 'scheduler.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='inspection_robot',
            executable='anomaly_detector',
            parameters=[anomaly_params, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            output='screen',
        ),
        Node(
            package='inspection_robot',
            executable='inspection_scheduler',
            parameters=[scheduler_params, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            output='screen',
        ),
        Node(
            package='inspection_robot',
            executable='dashboard_server',
            output='screen',
        ),
    ])
