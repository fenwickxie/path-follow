"""从 Gazebo SDF 模型提取底盘几何参数，生成 MPC / 仿真器参数建议（不修改原模型）。

用法：
    ros2 run path_follow_sim sdf_to_params ~/.gazebo/models/cart_front_steer/model.sdf

识别规则（启发式，结果务必人工核对）：
  * 名称含 "wheel" 的 link 视为车轮；
  * 车轮 link 的 <pose> 给出位置（仅支持相对 model 的简单 6 元素 pose）；
  * 轴距 = 车轮 x 的最大-最小，轮距 = 车轮 y 的最大-最小；
  * 轮半径取 collision 中 cylinder/sphere 的 radius；
  * 前轮关节为 universal/revolute 且带转向限位时判断为前轮转向。
"""
import argparse
import xml.etree.ElementTree as ET


def _pose(elem):
    node = elem.find('pose')
    if node is None or not (node.text or '').strip():
        return [0.0] * 6
    vals = [float(v) for v in node.text.split()]
    if len(vals) != 6:
        raise ValueError('only 6-element <pose> is supported')
    return vals


def parse_sdf(path):
    root = ET.parse(path).getroot()
    model = root.find('model')
    if model is None:
        raise ValueError(f'{path}: no <model> element')
    wheels = []
    for link in model.findall('link'):
        name = link.get('name', '')
        if 'wheel' not in name.lower():
            continue
        radius = None
        for shape in ('cylinder', 'sphere'):
            node = link.find(f'.//collision/geometry/{shape}/radius')
            if node is not None:
                radius = float(node.text)
                break
        pose = _pose(link)
        wheels.append({'name': name, 'x': pose[0], 'y': pose[1], 'radius': radius})
    steer_joints = []
    for joint in model.findall('joint'):
        if joint.get('type') in ('universal', 'revolute') and 'steer' in joint.get('name', '').lower():
            steer_joints.append(joint.get('name'))
    return {'model': model.get('name'), 'wheels': wheels, 'steer_joints': steer_joints}


def estimate(info):
    wheels = info['wheels']
    if len(wheels) < 2:
        raise ValueError('need at least 2 wheels to estimate geometry')
    xs = [w['x'] for w in wheels]
    ys = [w['y'] for w in wheels]
    radii = [w['radius'] for w in wheels if w['radius']]
    front = [n for n in info['steer_joints'] if 'front' in n.lower()]
    rear = [n for n in info['steer_joints'] if 'rear' in n.lower()]
    if front and rear:
        chassis = 'four_wheel_steering'
    elif front:
        chassis = 'ackermann'
    else:
        chassis = 'differential'
    wheelbase = max(xs) - min(xs)
    return {
        'model': info['model'], 'chassis': chassis,
        'wheelbase': wheelbase, 'track': max(ys) - min(ys),
        'wheel_radius': sum(radii) / len(radii) if radii else None,
        'lf': wheelbase / 2.0, 'lr': wheelbase / 2.0,
    }


def to_yaml(e):
    wb = e['wheelbase']
    lines = [
        f"# 由 {e['model']} 的 SDF 估算，请人工核对",
        '# --- MPC (mpc.yaml) ---',
        f"chassis: {e['chassis']}",
        'limits:',
        f'  wheelbase: {wb:.3f}',
        f"  front_axle_distance: {e['lf']:.3f}",
        f"  rear_axle_distance: {e['lr']:.3f}",
        '# --- 仿真器 (sim.yaml) ---',
        f"# chassis: {e['chassis']}  wheelbase: {wb:.3f}  "
        f"front_axle_distance: {e['lf']:.3f}  rear_axle_distance: {e['lr']:.3f}",
        f"# track={e['track']:.3f} wheel_radius={e['wheel_radius']}",
    ]
    if e['chassis'] == 'ackermann':
        lines.append('# 注意：阿克曼控制参考点在后轴中心；若 base_link 在车体中心，需做 TF 偏移。')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sdf')
    args = ap.parse_args(argv)
    print(to_yaml(estimate(parse_sdf(args.sdf))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
