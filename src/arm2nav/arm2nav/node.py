#!/usr/bin/env python3
"""10 Hz arm/navigation state bridge with optional sentry coarse localization."""

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import tf2_ros

from arm2nav_msgs.msg import Arm2Nav
from yolo_node.msg import DetectionArray


class Arm2NavNode(Node):
    def __init__(self):
        super().__init__('arm2nav')

        self.state_topic = self.declare_parameter('state_topic', '/arm2nav').value
        self.nav_command_topic = self.declare_parameter(
            'nav_command_topic', '/arm2nav/command').value
        self.arm_state_topic = self.declare_parameter(
            'arm_state_topic', '/arm2nav/from_arm').value
        self.detection_topic = self.declare_parameter(
            'detection_topic', '/yolo/detections').value
        self.pointcloud_topic = self.declare_parameter(
            'pointcloud_topic', '/camera/depth/color/points').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/color/image_raw').value
        self.target_frame = self.declare_parameter(
            'target_frame', 'camera_link').value
        self.target_class = self.declare_parameter(
            'target_class', 'blue_block').value
        self.confidence_threshold = float(self.declare_parameter(
            'confidence_threshold', 0.6).value)
        self.min_consecutive_detections = max(3, int(self.declare_parameter(
            'min_consecutive_detections', 3).value))
        self.roi_side_m = max(0.001, float(self.declare_parameter(
            'roi_side_m', 0.05).value))
        self.trigger_pulse_sec = max(0.1, float(self.declare_parameter(
            'trigger_pulse_sec', 0.2).value))
        self.publish_hz = max(1.0, float(self.declare_parameter(
            'publish_hz', 10.0).value))
        self.max_cloud_age_sec = max(0.0, float(self.declare_parameter(
            'max_cloud_age_sec', 0.5).value))
        self.require_arm_ready = bool(self.declare_parameter(
            'require_arm_ready', True).value)

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.state_pub = self.create_publisher(Arm2Nav, self.state_topic, state_qos)
        self.create_subscription(
            Arm2Nav, self.nav_command_topic, self._on_nav_command, event_qos)
        self.create_subscription(
            Arm2Nav, self.arm_state_topic, self._on_arm_state, event_qos)
        self.create_subscription(
            DetectionArray, self.detection_topic, self._on_detection, event_qos)
        self.create_subscription(
            PointCloud2, self.pointcloud_topic, self._on_pointcloud, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(
            Image, self.image_topic, self._on_image, sensor_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._target_need_stop = False
        self._suggest_x = 0.0
        self._suggest_y = 0.0
        self._suggest_yaw = 0.0
        self._nav_arm_trigger = False
        self._return_home = False
        self._nav_trigger_until = 0.0
        self._detection_streak = 0
        self._target_reported = False
        self._last_detection_stamp = None
        self._arm_ready = not self.require_arm_ready
        self._latest_cloud: Optional[PointCloud2] = None
        self._latest_cloud_mono = 0.0
        self._camera_info: Optional[CameraInfo] = None
        self._image_size: Optional[Tuple[int, int]] = None

        self.timer = self.create_timer(1.0 / self.publish_hz, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            'arm2nav ready: state=%s command=%s arm_state=%s publish_hz=%.1f' % (
                self.state_topic, self.nav_command_topic, self.arm_state_topic, self.publish_hz))

    def _on_image(self, msg: Image) -> None:
        self._image_size = (int(msg.width), int(msg.height))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self._latest_cloud_mono = time.monotonic()

    def _on_nav_command(self, msg: Arm2Nav) -> None:
        if msg.arm_trigger:
            self._nav_arm_trigger = True
            self.get_logger().info('Received arm_trigger=1 from navigation.')
        else:
            self._nav_arm_trigger = False

    def _on_arm_state(self, msg: Arm2Nav) -> None:
        self._arm_ready = True
        if msg.return_home:
            self._return_home = True
        if msg.nav_trigger:
            self._nav_trigger_until = max(
                self._nav_trigger_until,
                time.monotonic() + self.trigger_pulse_sec)
        if not msg.target_need_stop and not msg.arm_trigger:
            self._target_need_stop = False
            self._target_reported = False
            self._detection_streak = 0
        if not msg.arm_trigger:
            self._nav_arm_trigger = False

    def _on_detection(self, msg: DetectionArray) -> None:
        if not self._arm_ready:
            return
        best = None
        best_conf = -1.0
        for detection in msg.detections:
            if self.target_class and detection.class_name != self.target_class:
                continue
            if float(detection.confidence) < self.confidence_threshold:
                continue
            if float(detection.confidence) > best_conf:
                best = detection
                best_conf = float(detection.confidence)

        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp == self._last_detection_stamp:
            return
        self._last_detection_stamp = stamp

        if best is None:
            self._detection_streak = 0
            return
        self._detection_streak += 1
        if self._detection_streak < self.min_consecutive_detections or self._target_reported:
            return

        suggestion = self._compute_suggestion(best)
        if suggestion is None:
            return
        self._suggest_x, self._suggest_y, self._suggest_yaw = suggestion
        self._target_need_stop = True
        self._target_reported = True
        self._nav_trigger_until = time.monotonic() + self.trigger_pulse_sec
        self.get_logger().info(
            'Sentry target accepted after %d detections: x=%.3f y=%.3f yaw=%.3f rad; '
            'nav_trigger pulse.' % (
                self._detection_streak, self._suggest_x, self._suggest_y, self._suggest_yaw))

    @staticmethod
    def _field_offsets(msg: PointCloud2):
        return {field.name: int(field.offset) for field in msg.fields}

    def _cloud_xyz(self, msg: PointCloud2) -> Optional[np.ndarray]:
        offsets = self._field_offsets(msg)
        if not all(name in offsets for name in ('x', 'y', 'z')):
            return None
        count = int(msg.width) * int(msg.height)
        if count <= 0 or msg.point_step <= 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = count * int(msg.point_step)
        if raw.size < expected:
            return None
        raw = raw[:expected].reshape(count, int(msg.point_step))
        xyz = np.empty((count, 3), dtype=np.float32)
        value_dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        for index, name in enumerate(('x', 'y', 'z')):
            start = offsets[name]
            values = raw[:, start:start + 4].copy().view(value_dtype).reshape(-1)
            xyz[:, index] = values
        return xyz.reshape(int(msg.height), int(msg.width), 3)

    @staticmethod
    def _rotate_translate(points: np.ndarray, transform) -> np.ndarray:
        q = transform.transform.rotation
        t = transform.transform.translation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-12:
            return points
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        return points @ rotation.T + np.array([t.x, t.y, t.z])

    def _compute_suggestion(self, detection) -> Optional[Tuple[float, float, float]]:
        cloud = self._latest_cloud
        if cloud is None or time.monotonic() - self._latest_cloud_mono > self.max_cloud_age_sec:
            self.get_logger().warn('Cannot compute sentry suggestion: point cloud is unavailable or stale.')
            return None
        if self._camera_info is None or self._image_size is None:
            self.get_logger().warn('Cannot compute sentry suggestion: waiting for camera info/image size.')
            return None

        xyz = self._cloud_xyz(cloud)
        if xyz is None:
            self.get_logger().warn('Cannot compute sentry suggestion: point cloud has no x/y/z fields.')
            return None
        image_width, image_height = self._image_size
        if image_width <= 0 or image_height <= 0 or cloud.width <= 0 or cloud.height <= 0:
            return None
        center_u = (float(detection.x) + 0.5 * float(detection.width)) * cloud.width / image_width
        center_v = (float(detection.y) + 0.5 * float(detection.height)) * cloud.height / image_height
        u = int(round(center_u))
        v = int(round(center_v))
        u = max(0, min(int(cloud.width) - 1, u))
        v = max(0, min(int(cloud.height) - 1, v))

        local = xyz[max(0, v - 5):min(int(cloud.height), v + 6),
                    max(0, u - 5):min(int(cloud.width), u + 6)].reshape(-1, 3)
        local = local[np.isfinite(local).all(axis=1)]
        local = local[local[:, 2] > 0.05]
        if len(local) == 0:
            return None
        depth = float(np.median(local[:, 2]))
        fx, fy = float(self._camera_info.k[0]), float(self._camera_info.k[4])
        if fx <= 0.0 or fy <= 0.0 or depth <= 0.0:
            return None
        half_u = max(1, int(round(fx * self.roi_side_m * 0.5 / depth)))
        half_v = max(1, int(round(fy * self.roi_side_m * 0.5 / depth)))
        roi = xyz[max(0, v - half_v):min(int(cloud.height), v + half_v + 1),
                  max(0, u - half_u):min(int(cloud.width), u + half_u + 1)].reshape(-1, 3)
        roi = roi[np.isfinite(roi).all(axis=1)]
        roi = roi[roi[:, 2] > 0.05]
        if len(roi) < 3:
            return None

        try:
            stamp = Time.from_msg(cloud.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, cloud.header.frame_id, stamp,
                timeout=Duration(seconds=0.05))
            roi = self._rotate_translate(roi, transform)
        except Exception as exc:
            self.get_logger().warn(
                'Cannot transform sentry ROI from %s to %s: %s' % (
                    cloud.header.frame_id, self.target_frame, exc))
            return None

        point = np.median(roi, axis=0)
        suggest_x = float(point[0])
        d = float(point[2])
        if d <= 0.0:
            return None
        radicand = d * d - suggest_x * suggest_x
        if radicand <= 0.0:
            self.get_logger().warn(
                'Cannot compute sentry suggestion: |x|=%.3f is not smaller than depth z=%.3f.'
                % (abs(suggest_x), d))
            return None
        suggest_y = math.sqrt(radicand)
        suggest_yaw = math.atan2(suggest_x, suggest_y)
        return suggest_x, suggest_y, suggest_yaw

    def _publish_state(self) -> None:
        msg = Arm2Nav()
        msg.target_need_stop = bool(self._target_need_stop)
        msg.suggest_x = float(self._suggest_x)
        msg.suggest_y = float(self._suggest_y)
        msg.suggest_yaw = float(self._suggest_yaw)
        msg.nav_trigger = time.monotonic() < self._nav_trigger_until
        msg.arm_trigger = bool(self._nav_arm_trigger)
        msg.return_home = bool(self._return_home)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Arm2NavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
