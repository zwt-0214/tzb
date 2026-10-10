#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _include(package_name, launch_name, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', launch_name)
        ),
        launch_arguments=arguments.items(),
    )


def _launch_selected_camera(context):
    config_path = LaunchConfiguration('camera_config_file').perform(context)
    with open(config_path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    camera = config.get('camera', config)
    driver = str(camera.get('driver', 'orbbec_gemini336l')).strip().lower()
    camera_name = str(camera.get('camera_name', 'camera')).strip('/') or 'camera'
    width = int(camera.get('width', 640))
    height = int(camera.get('height', 480))
    fps = int(camera.get('fps', 30))

    if driver in ('d435i', 'realsense', 'realsense_d435i'):
        camera_launch = _include(
            'realsense2_camera',
            'rs_launch.py',
            {
                'camera_name': camera_name,
                # Keep the node at /<camera_name>. Setting both namespace and
                # name to "camera" would publish /camera/camera/... topics.
                'camera_namespace': '',
                'device_type': 'd435i',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_accel': 'false',
                'enable_gyro': 'false',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'pointcloud.ordered_pc': 'true',
                'depth_module.depth_profile': f'{width},{height},{fps}',
                'rgb_camera.color_profile': f'{width},{height},{fps}',
            },
        )
        return [
            LogInfo(msg='Camera switch: starting Intel RealSense D435i.'),
            LogInfo(msg=(
                f'Camera topics: /{camera_name}/color/image_raw, '
                f'/{camera_name}/depth/image_rect_raw, '
                f'/{camera_name}/depth/color/points.')),
            camera_launch,
        ]

    if driver in ('orbbec_gemini336l', 'gemini336l', 'gemini_336l', 'orbbec'):
        camera_launch = _include(
            'orbbec_camera',
            'gemini_330_series.launch.py',
            {
                'camera_name': camera_name,
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_left_ir': 'false',
                'enable_right_ir': 'false',
                'enable_accel': 'false',
                'enable_gyro': 'false',
                'depth_registration': 'true',
                'align_target_stream': 'COLOR',
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'true',
                'ordered_pc': 'true',
                'color_width': str(width),
                'color_height': str(height),
                'color_fps': str(fps),
                'depth_width': str(width),
                'depth_height': str(height),
                'depth_fps': str(fps),
            },
        )
        prefix = f'/{camera_name}'
        return [
            LogInfo(msg='Camera switch: starting Orbbec Gemini 336L.'),
            LogInfo(msg=(
                f'Camera topics: /{camera_name}/color/image_raw, '
                f'/{camera_name}/depth/image_rect_raw, '
                f'/{camera_name}/depth/color/points.')),
            GroupAction(actions=[
                SetRemap(
                    src=f'{prefix}/depth/image_raw',
                    dst=f'{prefix}/depth/image_rect_raw'),
                SetRemap(
                    src=f'{prefix}/depth_registered/points',
                    dst=f'{prefix}/depth/color/points'),
                camera_launch,
            ]),
        ]

    raise RuntimeError(
        f"Unsupported camera.driver '{driver}' in {config_path}; "
        "expected 'orbbec_gemini336l' or 'd435i'.")


def generate_launch_description():
    package_share = get_package_share_directory('pose_estimator')
    default_config = os.path.join(package_share, 'config', 'camera_switch.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=default_config,
            description='YAML selecting Orbbec Gemini 336L by default (D435i also supported).'),
        OpaqueFunction(function=_launch_selected_camera),
    ])
