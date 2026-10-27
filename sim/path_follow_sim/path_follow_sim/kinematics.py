"""纯 Python 运动学（不依赖 ROS，可单独测试）。

约定与 path_follow_mpc 一致：
  * 机体系 x 向前、y 向左，角度逆时针为正。
  * 阿克曼：参考点在后轴中心，  w = vx * tan(delta) / wheelbase
  * 四轮转向：参考点在前后轴之间（到前轴 lf、到后轴 lr）：
        w  = vx * (tan(df) - tan(dr)) / (lf + lr)
        vy = vx * (lr * tan(df) + lf * tan(dr)) / (lf + lr)
"""
import math
from dataclasses import dataclass


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class Body:
    """机体系速度。"""
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def ackermann_to_body(speed, steer, wheelbase):
    return Body(speed, 0.0, speed * math.tan(steer) / wheelbase)


def four_ws_to_body(speed, front, rear, lf, lr):
    length = lf + lr
    return Body(speed,
                speed * (lr * math.tan(front) + lf * math.tan(rear)) / length,
                speed * (math.tan(front) - math.tan(rear)) / length)


def integrate(pose, body, dt):
    """中点法积分，圆弧运动下比欧拉法精度高。"""
    mid = pose.yaw + 0.5 * body.w * dt
    c, s = math.cos(mid), math.sin(mid)
    return Pose2D(pose.x + (body.vx * c - body.vy * s) * dt,
                  pose.y + (body.vx * s + body.vy * c) * dt,
                  normalize_angle(pose.yaw + body.w * dt))


def first_order(current, target, dt, tau):
    """一阶滞后，模拟执行机构响应；tau<=0 表示理想响应。"""
    if tau <= 1e-9:
        return target
    return current + (target - current) * (1.0 - math.exp(-dt / tau))


def rate_limit(current, target, max_rate, dt):
    """变化率限幅；max_rate<=0 表示不限制。"""
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + clamp(target - current, -step, step)
