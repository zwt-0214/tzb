import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('arm2nav')
    default_params = os.path.join(package_share, 'config', 'arm2nav.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='arm2nav',
            executable='arm2nav',
            name='arm2nav',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
