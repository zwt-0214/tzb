# ROS 2 话题接口

本文档列出当前工程主要节点的默认 ROS 2 话题。话题名均可通过节点参数或 YAML 配置修改。

## 数据流

```text
Orbbec 相机
  ├─ RGB 图像 ─> yolo_node ─> 2D 检测框
  └─ 点云 ──────────────────> pose_estimator ─> 目标 6DoF 位姿
                                                   │
                                                   v
                                              tzb_catch
                                                   │
                                                   v
                                               串口驱动
```

## `yolo_node`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/camera/color/image_raw` | `sensor_msgs/Image` | 相机 RGB 原始图像 |
| 发布 | `/yolo/detections` | `yolo_node/DetectionArray` | YOLO 检测结果 |
| 发布 | `/yolo/annotated_image` | `sensor_msgs/Image` | 绘制检测框、类别和置信度的图像 |

`DetectionArray.detections[]` 中单个目标的字段：

| 字段 | 含义 |
| --- | --- |
| `x`, `y` | 检测框左上角像素坐标 |
| `width`, `height` | 检测框宽高，单位为像素 |
| `confidence` | 检测置信度，范围为 `0.0~1.0` |
| `class_id` | 类别编号 |
| `class_name` | 类别名称 |

检测结果只有在置信度达到发布门限时才发布；标注图像仅在存在订阅者时生成。配置见 [yolo_detector.py](src/yolo_node/yolo_node/yolo_detector.py#L132)。

## `pose_estimator`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 与彩色图对齐的有序点云 |
| 订阅 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| 订阅 | `/yolo/detections` | `yolo_node/DetectionArray` | YOLO 二维检测框 |
| 发布 | `/pose_estimator/object_pose` | `geometry_msgs/PoseStamped` | 目标最终 6DoF 位姿 |
| 发布 | `/pose_estimation_node/roi_cloud` | `sensor_msgs/PointCloud2` | 检测框内降采样后的目标点云 |
| 发布 | `/pose_estimation_node/coarse_pose` | `geometry_msgs/PoseStamped` | RANSAC 粗配准位姿 |
| 发布 | `/pose_estimation_node/coarse_model_marker` | `visualization_msgs/Marker` | 粗配准模型，RViz 中为蓝色 |
| 发布 | `/pose_estimation_node/model_marker` | `visualization_msgs/Marker` | ICP 最终配准模型，RViz 中为绿色 |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | 父坐标系到 `object_pose` 的动态变换 |

`object_pose` 字段含义：

- `header.frame_id`：位姿参考坐标系，通常来自相机点云。
- `position`：目标中心位置，单位为米。
- `orientation`：目标姿态四元数；当前 `pose_yaw: 0.0` 会覆盖输出 yaw。

配置见 [params.yaml](src/pose_estimator/config/params.yaml#L1)。

## `tzb_catch`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/pose_estimator/object_pose` | `geometry_msgs/PoseStamped` | 视觉目标位姿 |
| 订阅 | `/serial/arm_joint_state` | `sensor_msgs/JointState` | 机械臂实际关节反馈 |
| 订阅 | `/navigation_goal_status` | `std_msgs/Int8` | `0`：等待或重新使能；`1`：触发抓取 |
| 订阅 | `/tzb_catch/start` | `std_msgs/Int8` | 值为 `1` 时启动抓取任务 |
| 发布 | `/tzb_catch/arm_joint_command` | `sensor_msgs/JointState` | 六个机械臂关节目标角，单位为 rad |
| 发布 | `/tzb_catch/gripper_command` | `std_msgs/Int8` | `1`：闭合；`-1`：张开 |
| 发布 | `/tzb_catch/result` | `std_msgs/Int8` | `1`：任务成功；`-1`：任务失败 |
| 发布 | `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 规划后的关节轨迹 |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | RViz 模型关节状态 |
| 发布 | `/tzb_catch/e3_marker` | `visualization_msgs/Marker` | 目标物体模型 |
| 发布 | `/tzb_catch/reference_frames` | `visualization_msgs/MarkerArray` | 参考坐标系和 TCP 轨迹可视化 |

默认配置见 [tzb_planner.yaml](src/lite_motion_planner/config/tzb_planner.yaml#L41)。

## `rm_serial_driver`

| 方向 | 话题 | 消息类型 | 含义 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |
| 订阅 | `/tzb_catch/arm_joint_command` | `sensor_msgs/JointState` | 机械臂关节命令 |
| 订阅 | `/tzb_catch/gripper_command` | `std_msgs/Int8` | 夹爪开合命令 |
| 发布 | `/serial/arm_joint_state` | `sensor_msgs/JointState` | 下位机返回的实际关节角 |

## 可选通信节点

- `arm2nav`：通过 `/arm2nav`、`/arm2nav/command`、`/arm2nav/from_arm` 交换目标停车、导航建议位置、抓取触发和返航状态。
- `nav2arm`：旧版接口，通过 `tzb_state_test`、`/tzb_catch/start` 和 `/tzb_catch/result` 协调导航与机械臂。
- 默认总启动文件中两者均未启用，分别需要设置 `use_arm2nav:=true` 或 `use_nav2arm:=true`。


2026.10.10
新增了视觉给导航二维地图发送终点和yaw值（相对于此时的自身位置，请注意坐标系转换）的能力
