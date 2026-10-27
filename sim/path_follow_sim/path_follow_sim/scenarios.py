"""路径场景生成与跟踪误差计算（纯 Python，不依赖 ROS）。

新增场景：在 SCENARIOS 字典注册一个返回 [(x, y, yaw), ...] 的函数即可。
"""
import math

from .kinematics import normalize_angle


def _resample(points, spacing):
    """沿折线按弧长等间距重采样，并用切线方向计算 yaw。"""
    out = [points[0]]
    carry = 0.0
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        if seg < 1e-9:
            continue
        d = spacing - carry
        while d <= seg:
            t = d / seg
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            d += spacing
        carry = seg - (d - spacing)
    if math.dist(out[-1], points[-1]) > 1e-6:
        out.append(points[-1])
    return out


def _with_yaw(xy, keep_heading=None):
    res = []
    for i, p in enumerate(xy):
        if keep_heading is not None:
            yaw = keep_heading
        else:
            q = xy[min(i + 1, len(xy) - 1)]
            r = xy[max(i - 1, 0)]
            yaw = math.atan2(q[1] - r[1], q[0] - r[0])
        res.append((p[0], p[1], yaw))
    return res


def line(p):
    n = 400
    xy = [(p['x0'] + p['length'] * i / n, p['y0']) for i in range(n + 1)]
    return xy


def arc(p):
    """从 (x0,y0) 朝 +x 出发的圆弧，radius>0 左转，<0 右转。"""
    r, span = p['radius'], p['arc_angle']
    n = 400
    return [(p['x0'] + abs(r) * math.sin(span * i / n),
             p['y0'] + r * (1 - math.cos(span * i / n))) for i in range(n + 1)]


def s_curve(p):
    n = 600
    return [(p['x0'] + p['length'] * i / n,
             p['y0'] + p['amplitude'] * math.sin(2 * math.pi * i / n)) for i in range(n + 1)]


def circle(p):
    r = abs(p['radius'])
    n = 800
    return [(p['x0'] + r * math.sin(2 * math.pi * i / n),
             p['y0'] + r * (1 - math.cos(2 * math.pi * i / n))) for i in range(n + 1)]


def figure8(p):
    r = abs(p['radius'])
    n = 1200
    return [(p['x0'] + r * math.sin(2 * math.pi * i / n),
             p['y0'] + r * math.sin(4 * math.pi * i / n) / 2) for i in range(n + 1)]


def lane_change(p):
    """平滑变道：cos 插值，横向偏移 amplitude，纵向长度 length。"""
    n = 400
    return [(p['x0'] + p['length'] * i / n,
             p['y0'] + p['amplitude'] * 0.5 * (1 - math.cos(math.pi * i / n))) for i in range(n + 1)]


SCENARIOS = {'line': line, 'arc': arc, 's_curve': s_curve, 'circle': circle,
             'figure8': figure8, 'lane_change': lane_change}

DEFAULTS = {'x0': 0.0, 'y0': 0.0, 'length': 8.0, 'radius': 3.0,
            'arc_angle': math.pi / 2, 'amplitude': 1.0}


def generate(kind, spacing=0.05, keep_heading=None, **params):
    """返回 [(x, y, yaw)]。keep_heading 不为 None 时全程固定朝向（四轮转向蟹行/全向）。"""
    if kind not in SCENARIOS:
        raise ValueError(f'unknown scenario {kind!r}, choose from {sorted(SCENARIOS)}')
    if spacing <= 0.0:
        raise ValueError('spacing must be > 0')
    merged = {**DEFAULTS, **params}
    xy = _resample(SCENARIOS[kind](merged), spacing)
    return _with_yaw(xy, keep_heading)


def tracking_error(path, x, y, yaw):
    """点到折线的最近点误差。返回 (cross_track 有符号[左正], 相对路径点 yaw 的航向误差, 弧长进度)。"""
    best = (float('inf'), 0.0, 0.0, 0.0)
    s_acc = 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg2 = dx * dx + dy * dy
        seg = math.sqrt(seg2)
        t = 0.0 if seg2 < 1e-12 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / seg2))
        px, py = a[0] + t * dx, a[1] + t * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            sign = 1.0 if dx * (y - a[1]) - dy * (x - a[0]) >= 0.0 else -1.0
            best = (d, sign * d, a[2], s_acc + t * seg)
        s_acc += seg
    return best[1], normalize_angle(yaw - best[2]), best[3]
