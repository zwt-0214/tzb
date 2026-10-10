#!/usr/bin/env python3
"""
一体化 bringup: 可切换深度相机 + YOLO 检测 + pose_estimator 位姿估计。

用法:
  ros2 launch pose_estimator perception_bringup.launch.py

可选参数:
  launch_camera:=false           # 相机已在别处单独启动
  camera_config_file:=/path/to/camera_switch.yaml
  params_file:=/path/to/params.yaml

注意:
  - pose_estimator 内部也有 launch_camera 开关，这里统一由本文件启动相机，
    因此强制传 launch_camera:=false 给 pose_estimator，避免重复启动。
  - camera_switch.yaml 默认使用 Orbbec Gemini 336L，也可切换到 D435i。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pose_estimator_pkg = get_package_share_directory('pose_estimator')
    yolo_node_pkg = get_package_share_directory('yolo_node')

    default_params = os.path.join(pose_estimator_pkg, 'config', 'params.yaml')
    default_camera_config = os.path.join(
        pose_estimator_pkg, 'config', 'camera_switch.yaml')

    launch_camera = LaunchConfiguration('launch_camera')
    params_file = LaunchConfiguration('params_file')

    declared_args = [
        DeclareLaunchArgument('launch_camera', default_value='true',
                              description='是否按 camera_switch.yaml 启动相机'),
        DeclareLaunchArgument('camera_config_file', default_value=default_camera_config,
                              description='默认 Orbbec Gemini 336L，可切换 D435i'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='pose_estimator 参数文件路径'),
    ]

    # ---- 1. YAML-selected camera ----
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pose_estimator_pkg, 'launch', 'camera_switch.launch.py')
        ]),
        launch_arguments={
            'camera_config_file': LaunchConfiguration('camera_config_file'),
        }.items(),
        condition=IfCondition(launch_camera),
    )

    # ---- 2. YOLO 检测节点 ----
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(yolo_node_pkg, 'launch', 'yolo_node.launch.py')
        ]),
        launch_arguments={
            'image_topic': '/camera/color/image_raw',
            'detection_topic': '/yolo/detections',
        }.items(),
    )

    # ---- 3. pose_estimator (launch_camera=false 避免重复启动相机) ----
    pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pose_estimator_pkg, 'launch', 'pose_estimator.launch.py')
        ]),
        launch_arguments={
            'launch_camera': 'false',
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription(declared_args + [
        camera,
        yolo,
        pose,
        LogInfo(msg='perception bringup 已启动: 相机 + YOLO + pose_estimator'),
    ])
