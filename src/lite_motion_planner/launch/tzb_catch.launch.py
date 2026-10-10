from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


SUPPORTED_ROBOT_MODELS = {
    'tzb_description': 'tzb_planner.yaml',
    'engineer_0520_description': 'engineer_0520_planner.yaml',
}


def _build_model_nodes(context, pkg_share):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    if robot_model not in SUPPORTED_ROBOT_MODELS:
        supported = ', '.join(SUPPORTED_ROBOT_MODELS)
        raise RuntimeError(
            f"Unsupported robot_model '{robot_model}'. Expected one of: {supported}")

    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if not params_file:
        params_file = os.path.join(
            pkg_share, 'config', SUPPORTED_ROBOT_MODELS[robot_model])
    use_serial_driver = (
        LaunchConfiguration('use_serial_driver').perform(context).lower() == 'true')
    catch_parameters = [params_file]
    catch_parameters.append({
        'tzb.autostart': (
            LaunchConfiguration('autostart').perform(context).lower() == 'true'),
    })
    listen_navigation_goal_status = LaunchConfiguration(
        'listen_navigation_goal_status').perform(context).strip().lower()
    if listen_navigation_goal_status:
        catch_parameters.append({
            'tzb.listen_navigation_goal_status': listen_navigation_goal_status == 'true',
        })
    if not use_serial_driver:
        catch_parameters.append({
            'tzb.arm_feedback_enabled': False,
            'tzb.require_arm_feedback_for_completion': False,
        })

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'planning_node.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_robot_state_publisher': LaunchConfiguration('use_robot_state_publisher'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    tzb_catch_node = Node(
        package='lite_motion_planner',
        executable='tzb_catch',
        name='tzb_catch',
        output='screen',
        parameters=catch_parameters,
    )

    return [
        LogInfo(msg=(
            f"lite_motion_planner robot_model={robot_model}; "
            f"loading planner parameters from {params_file}")),
        planner_launch,
        tzb_catch_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('lite_motion_planner')
    serial_share = get_package_share_directory('rm_serial_driver')
    default_serial_params = os.path.join(serial_share, 'config', 'serial_params.yaml')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='tzb_description',
        description='Robot model: tzb_description (default) or engineer_0520_description',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional planner YAML override; empty selects it from robot_model',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Start immediately; false waits for /navigation_goal_status=1',
    )
    listen_navigation_goal_status_arg = DeclareLaunchArgument(
        'listen_navigation_goal_status',
        default_value='',
        description='Optional override: listen for /navigation_goal_status before starting the arm task',
    )
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rsp_arg = DeclareLaunchArgument('use_robot_state_publisher', default_value='true')
    use_serial_driver_arg = DeclareLaunchArgument('use_serial_driver', default_value='true')
    serial_params_file_arg = DeclareLaunchArgument(
        'serial_params_file', default_value=default_serial_params)
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'config', 'lite_motion_debug_safe.rviz'))

    serial_node = Node(
        package='rm_serial_driver',
        executable='node',
        name='serial_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_serial_driver')),
        parameters=[LaunchConfiguration('serial_params_file')],
    )

    return LaunchDescription([
        robot_model_arg,
        params_file_arg,
        autostart_arg,
        listen_navigation_goal_status_arg,
        use_rviz_arg,
        use_rsp_arg,
        use_serial_driver_arg,
        serial_params_file_arg,
        rviz_config_arg,
        serial_node,
        OpaqueFunction(
            function=lambda context: _build_model_nodes(context, pkg_share)),
    ])
