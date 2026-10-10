import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8

try:
    from tzb_state_test.msg import TzbState
except ImportError:  # pragma: no cover - reported clearly when the node starts.
    TzbState = None


class Nav2ArmCoordinator(Node):
    def __init__(self):
        super().__init__('nav2arm')

        if TzbState is None:
            raise RuntimeError(
                'tzb_state_test.msg.TzbState is not available. '
                'Source /home/tzb/nav_ws/install/setup.bash before starting nav2arm.'
            )

        self.state_topic = self.declare_parameter(
            'state_topic', 'tzb_state_test').value
        self.catch_start_topic = self.declare_parameter(
            'catch_start_topic', '/tzb_catch/start').value
        self.catch_result_topic = self.declare_parameter(
            'catch_result_topic', '/tzb_catch/result').value
        self.wait_for_nav_go_finish = self.declare_parameter(
            'wait_for_nav_go_finish', True).value
        self.trigger_hold_sec = max(
            0.0, self.declare_parameter('trigger_hold_sec', 0.2).value)

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_pub = self.create_publisher(TzbState, self.state_topic, state_qos)
        self.catch_start_pub = self.create_publisher(
            Int8, self.catch_start_topic, event_qos)

        self.create_subscription(
            TzbState, self.state_topic, self._on_state, event_qos)
        self.create_subscription(
            Int8, self.catch_result_topic, self._on_catch_result, event_qos)

        self._nav_go_finish = False
        self._arm_finish = False
        self._nav_true_since = None
        self._task_active = False
        self._waiting_for_nav_reset = False
        self._direct_start_pending = not self.wait_for_nav_go_finish

        self.create_timer(0.05, self._tick)
        self._publish_state(nav_go_finish=False, arm_finish=False)

        self.get_logger().info(
            'nav2arm ready: state=%s, catch_start=%s, catch_result=%s, wait_for_nav_go_finish=%s'
            % (self.state_topic,
               self.catch_start_topic, self.catch_result_topic,
               self.wait_for_nav_go_finish))

    def _publish_state(self, nav_go_finish=None, arm_finish=None):
        if nav_go_finish is not None:
            self._nav_go_finish = bool(nav_go_finish)
        if arm_finish is not None:
            self._arm_finish = bool(arm_finish)

        msg = TzbState()
        msg.nav_go_finish = self._nav_go_finish
        msg.arm_finish = self._arm_finish
        self.state_pub.publish(msg)

    def _publish_start(self):
        msg = Int8()
        msg.data = 1
        self.catch_start_pub.publish(msg)

    def _on_state(self, msg):
        self._nav_go_finish = bool(msg.nav_go_finish)
        self._arm_finish = bool(msg.arm_finish)

        if self.wait_for_nav_go_finish and not self._nav_go_finish:
            self._nav_true_since = None
            self._waiting_for_nav_reset = False
            if self._arm_finish and not self._task_active:
                self._publish_state(arm_finish=False)
        elif not self.wait_for_nav_go_finish:
            self._nav_true_since = None

    def _tick(self):
        now = time.monotonic()

        if self._task_active or self._waiting_for_nav_reset:
            return

        if not self.wait_for_nav_go_finish:
            if self._direct_start_pending:
                self._direct_start_pending = False
                self._begin_pose_or_arm_task(now, triggered_by_nav=False)
            return

        if not self._nav_go_finish:
            self._nav_true_since = None
            return

        if self._nav_true_since is None:
            self._nav_true_since = now
            return

        if now - self._nav_true_since < self.trigger_hold_sec:
            return

        self._begin_pose_or_arm_task(now, triggered_by_nav=True)

    def _begin_pose_or_arm_task(self, now, triggered_by_nav):
        del now
        self._nav_true_since = None
        self._publish_state(
            nav_go_finish=True if triggered_by_nav else None,
            arm_finish=False)
        if triggered_by_nav:
            self.get_logger().info(
                'nav_go_finish=true; starting arm task.')
        else:
            self.get_logger().info(
                'wait_for_nav_go_finish=false; starting arm task immediately.')
        self._start_arm_task()

    def _start_arm_task(self):
        self._task_active = True
        self._publish_start()
        self.get_logger().info(
            'Published arm task start on [%s].' % self.catch_start_topic)

    def _on_catch_result(self, msg):
        if not self._task_active:
            return

        self._task_active = False
        success = msg.data > 0
        self._publish_state(nav_go_finish=self._nav_go_finish, arm_finish=success)
        self._waiting_for_nav_reset = self.wait_for_nav_go_finish

        if success:
            self.get_logger().info(
                'Arm task completed; published arm_finish=true on [%s].'
                % self.state_topic)
        else:
            self.get_logger().error(
                'Arm task failed; published arm_finish=false on [%s].'
                % self.state_topic)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ArmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
