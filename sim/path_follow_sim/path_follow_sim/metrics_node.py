"""指标记录节点：记录轨迹与跟踪误差，结束时写 CSV + summary.json 并按阈值判定通过/失败。

结束条件：goal_topic 收到 true，或 timeout 超时。exit_on_finish=true 时自动退出；
单独运行 metrics_recorder 时进程返回码 0=通过、1=失败。通过 launch 运行时请读取
output_dir/summary.json 的 passed 字段判定（launch 不传递子进程返回码）。
"""
import csv
import json
import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool

from . import scenarios


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def summarize(rows, path, limits):
    """rows: [(t,x,y,yaw,vx,vy,w,cte,he)]，返回指标字典。纯函数，便于测试。"""
    if not rows:
        return {'passed': False, 'reason': 'no odometry recorded'}
    cte = [abs(r[7]) for r in rows]
    he = [abs(r[8]) for r in rows]
    last = rows[-1]
    end = path[-1]
    final_pos = math.hypot(last[1] - end[0], last[2] - end[1])
    result = {
        'samples': len(rows), 'duration_s': rows[-1][0] - rows[0][0],
        'max_cross_track_m': max(cte),
        'rms_cross_track_m': math.sqrt(sum(c * c for c in cte) / len(cte)),
        'max_heading_error_rad': max(he),
        'final_position_error_m': final_pos,
        'max_speed_mps': max(math.hypot(r[4], r[5]) for r in rows),
        'max_yaw_rate_rps': max(abs(r[6]) for r in rows),
    }
    failures = []
    if result['max_cross_track_m'] > limits['max_cross_track']:
        failures.append('max_cross_track')
    if final_pos > limits['max_final_error']:
        failures.append('final_error')
    result['passed'] = not failures
    result['failed_checks'] = failures
    return result


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        d = self.declare_parameter
        self.odom_topic = d('topics.odometry', 'odom').value
        self.path_topic = d('topics.path', 'plan').value
        goal_topic = d('topics.goal_reached', 'mpc/goal_reached').value
        self.out_dir = d('output_dir', '/tmp/path_follow_sim').value
        self.timeout = d('timeout', 60.0).value
        self.exit_on_finish = d('exit_on_finish', True).value
        self.limits = {'max_cross_track': d('limits.max_cross_track', 0.3).value,
                       'max_final_error': d('limits.max_final_error', 0.15).value}
        self.settle = d('settle_time', 0.5).value
        self.path = None
        self.rows = []
        self.t0 = None
        self.goal_time = None
        self.finished = False
        self.exit_code = 0
        self.create_subscription(Path, self.path_topic, self._on_path, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        if goal_topic:
            self.create_subscription(Bool, goal_topic, self._on_goal, 10)
        self.create_timer(0.1, self._check)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y, quaternion_yaw(p.pose.orientation))
               for p in msg.poses]
        if len(pts) >= 2:
            self.path = pts
            self.rows = []
            self.t0 = self._now()
            self.goal_time = None
            self.get_logger().info(f'new path ({len(pts)} points), recording restarted')

    def _on_odom(self, msg):
        if self.path is None or self.finished:
            return
        p = msg.pose.pose
        yaw = quaternion_yaw(p.orientation)
        cte, he, _ = scenarios.tracking_error(self.path, p.position.x, p.position.y, yaw)
        t = self._now() - self.t0
        self.rows.append((t, p.position.x, p.position.y, yaw, msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y, msg.twist.twist.angular.z, cte, he))

    def _on_goal(self, msg):
        if msg.data and self.path is not None and self.goal_time is None:
            self.goal_time = self._now()

    def _check(self):
        if self.finished or self.path is None:
            return
        done = self.goal_time is not None and self._now() - self.goal_time >= self.settle
        timed_out = self._now() - self.t0 > self.timeout
        if done or timed_out:
            self._finish('goal_reached' if done else 'timeout')

    def _finish(self, reason):
        self.finished = True
        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, 'trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'vy', 'w', 'cross_track', 'heading_error'])
            w.writerows(self.rows)
        with open(os.path.join(self.out_dir, 'path.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'yaw'])
            w.writerows(self.path)
        result = summarize(self.rows, self.path, self.limits)
        result['finish_reason'] = reason
        if reason == 'timeout':
            result['passed'] = False
            result.setdefault('failed_checks', []).append('timeout')
        with open(os.path.join(self.out_dir, 'summary.json'), 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info('RESULT ' + json.dumps(result))
        self.exit_code = 0 if result['passed'] else 1


def main(args=None):
    rclpy.init(args=args)
    node = MetricsRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.exit_on_finish:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    code = node.exit_code
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
