"""运动学底盘仿真节点：订阅速度/转角指令，发布 Odometry、TF。

与被测算法完全解耦：只认话题名与消息类型，由参数配置。
指令类型 cmd_type：
  twist                 geometry_msgs/Twist（linear.x/y 与 angular.z 直接作为机体速度）
  twist_stamped         geometry_msgs/TwistStamped
  ackermann             ackermann_msgs/AckermannDriveStamped（speed + steering_angle）
  four_wheel_steering   path_follow_mpc/FourWheelSteeringCommand（速度 + 前/后轴转角）
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from . import kinematics as kin

CHASSIS = ('ackermann', 'differential', 'omnidirectional', 'four_wheel_steering')
CMD_TYPES = ('twist', 'twist_stamped', 'ackermann', 'four_wheel_steering')


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        d = self.declare_parameter
        self.chassis = d('chassis', 'ackermann').value
        self.cmd_type = d('cmd_type', 'twist').value
        cmd_topic = d('topics.command', 'cmd_vel').value
        odom_topic = d('topics.odometry', 'odom').value
        self.odom_frame = d('frames.odom', 'odom').value
        self.base_frame = d('frames.base', 'base_link').value
        self.publish_tf = d('publish_tf', True).value
        rate = d('rate', 50.0).value
        self.cmd_timeout = d('cmd_timeout', 0.5).value
        self.tau = d('actuator_time_constant', 0.05).value
        self.noise_xy = d('odom_noise.position', 0.0).value
        self.noise_yaw = d('odom_noise.yaw', 0.0).value
        self.wheelbase = d('wheelbase', 0.65).value
        self.lf = d('front_axle_distance', 0.325).value
        self.lr = d('rear_axle_distance', 0.325).value
        self.max_speed = d('max_speed', 5.0).value
        self.initial = kin.Pose2D(d('initial_pose.x', 0.0).value, d('initial_pose.y', 0.0).value,
                                  d('initial_pose.yaw', 0.0).value)

        if self.chassis not in CHASSIS:
            raise ValueError(f'chassis must be one of {CHASSIS}')
        if self.cmd_type not in CMD_TYPES:
            raise ValueError(f'cmd_type must be one of {CMD_TYPES}')
        if rate <= 0.0 or self.wheelbase <= 0.0 or self.lf <= 0.0 or self.lr <= 0.0:
            raise ValueError('rate, wheelbase, front/rear_axle_distance must be > 0')

        self.pose = kin.Pose2D(self.initial.x, self.initial.y, self.initial.yaw)
        self.body = kin.Body()
        self.target = kin.Body()
        self.last_cmd_time = None
        self.last_step = self.get_clock().now()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(*self._cmd_subscription(cmd_topic), qos)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, qos)
        self.tf = TransformBroadcaster(self)
        self.create_service(Empty, '~/reset', self._reset)
        self.create_timer(1.0 / rate, self._step)
        self.get_logger().info(
            f'sim_robot: chassis={self.chassis} cmd={cmd_topic}({self.cmd_type}) odom={odom_topic}')

    def _cmd_subscription(self, topic):
        if self.cmd_type == 'twist':
            return Twist, topic, lambda m: self._set(kin.Body(m.linear.x, m.linear.y, m.angular.z))
        if self.cmd_type == 'twist_stamped':
            return (TwistStamped, topic, lambda m: self._set(
                kin.Body(m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)))
        if self.cmd_type == 'ackermann':
            from ackermann_msgs.msg import AckermannDriveStamped
            return (AckermannDriveStamped, topic, lambda m: self._set(
                kin.ackermann_to_body(m.drive.speed, m.drive.steering_angle, self.wheelbase)))
        from path_follow_mpc.msg import FourWheelSteeringCommand
        return (FourWheelSteeringCommand, topic, lambda m: self._set(kin.four_ws_to_body(
            m.longitudinal_velocity, m.front_steering_angle, m.rear_steering_angle,
            self.lf, self.lr)))

    def _set(self, body):
        values = (body.vx, body.vy, body.w)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('ignored non-finite command')
            return
        # 非全向底盘没有独立横向速度
        if self.chassis != 'omnidirectional' and self.chassis != 'four_wheel_steering':
            body.vy = 0.0
        body.vx = kin.clamp(body.vx, -self.max_speed, self.max_speed)
        self.target = body
        self.last_cmd_time = self.get_clock().now()

    def _reset(self, _req, resp):
        self.pose = kin.Pose2D(self.initial.x, self.initial.y, self.initial.yaw)
        self.body = kin.Body()
        self.target = kin.Body()
        self.last_cmd_time = None
        self.get_logger().info('reset to initial pose')
        return resp

    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0 or dt > 1.0:
            return
        stale = (self.last_cmd_time is None or
                 (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
        goal = kin.Body() if stale else self.target
        self.body = kin.Body(kin.first_order(self.body.vx, goal.vx, dt, self.tau),
                             kin.first_order(self.body.vy, goal.vy, dt, self.tau),
                             kin.first_order(self.body.w, goal.w, dt, self.tau))
        self.pose = kin.integrate(self.pose, self.body, dt)
        self._publish(now.to_msg())

    def _publish(self, stamp):
        import random
        x = self.pose.x + random.gauss(0.0, self.noise_xy) if self.noise_xy > 0.0 else self.pose.x
        y = self.pose.y + random.gauss(0.0, self.noise_xy) if self.noise_xy > 0.0 else self.pose.y
        yaw = self.pose.yaw + random.gauss(0.0, self.noise_yaw) if self.noise_yaw > 0.0 else self.pose.yaw
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.twist.twist.linear.x = self.body.vx
        odom.twist.twist.linear.y = self.body.vy
        odom.twist.twist.angular.z = self.body.w
        self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
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
