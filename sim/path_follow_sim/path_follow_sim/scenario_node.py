"""场景发布节点：生成测试路径并发布 nav_msgs/Path，可通过服务重发。

重发会让 MPC 重新初始化任务；先调用 sim_robot 的 ~/reset 再重发即可重复测试。
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_srvs.srv import Empty

from . import scenarios
from .sim_node import yaw_to_quaternion


class ScenarioPublisher(Node):
    def __init__(self):
        super().__init__('scenario_publisher')
        d = self.declare_parameter
        self.kind = d('scenario', 'line').value
        self.frame = d('frame', 'odom').value
        topic = d('topic', 'plan').value
        self.spacing = d('spacing', 0.05).value
        self.start_delay = d('start_delay', 2.0).value
        keep = d('keep_heading', False).value
        self.keep_heading = d('heading', 0.0).value if keep else None
        self.params = {k: d(f'params.{k}', v).value for k, v in scenarios.DEFAULTS.items()}
        self.points = scenarios.generate(self.kind, self.spacing, self.keep_heading, **self.params)
        self.pub = self.create_publisher(Path, topic, 10)
        self.create_service(Empty, '~/publish', self._on_publish)
        self.timer = self.create_timer(max(self.start_delay, 0.1), self._first)
        self.get_logger().info(f'scenario={self.kind} points={len(self.points)} -> {topic}')

    def _first(self):
        self.timer.cancel()
        self.publish()

    def _on_publish(self, _req, resp):
        self.publish()
        return resp

    def publish(self):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.points:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation = yaw_to_quaternion(yaw)
            msg.poses.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f'published path: {len(msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
