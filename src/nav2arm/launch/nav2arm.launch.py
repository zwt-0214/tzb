import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('nav2arm')
    default_params = os.path.join(package_share, 'config', 'nav2arm.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='nav2arm coordinator parameter file',
        ),
        DeclareLaunchArgument(
            'wait_for_nav_go_finish',
            default_value='true',
            description='true waits for nav_go_finish; false starts the arm task immediately',
        ),
        Node(
            package='nav2arm',
            executable='coordinator',
            name='nav2arm',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'wait_for_nav_go_finish':
                    ParameterValue(
                        LaunchConfiguration('wait_for_nav_go_finish'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
