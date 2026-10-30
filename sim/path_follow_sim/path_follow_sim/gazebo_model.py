"""Generate a portable, joint-driven SDF robot without downloading external models.

Ackermann uses a rear-axle reference; 4WS/omni use an axle-interior reference.
Omni is implemented as four independently driven/steered swerve modules, not
an ideal lateral force or a visual-only mecanum wheel.
"""
import math
import xml.etree.ElementTree as ET

CHASSIS = ('ackermann', 'differential', 'omnidirectional', 'four_wheel_steering')
DRIVE_DEFAULTS = {
    'command_timeout': 0.5, 'wheel_torque': 8.0, 'steering_torque': 10.0,
    'steering_rate': 2.0, 'steering_limit': 0.8,
    'max_speed': 2.0, 'max_yaw_rate': 2.0, 'publish_rate': 50.0,
}


def element(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = str(value)
    return child


def robot_sdf(chassis, command_type, wheelbase=0.65, lf=0.325, lr=0.325,
              track=0.5, radius=0.1, command_topic='cmd_vel', odometry_topic='odom',
              odom_frame='odom', base_frame='base_link', drive=None):
    if chassis not in CHASSIS:
        raise ValueError(f'unknown chassis: {chassis}')
    for v in (wheelbase, lf, lr, track, radius):
        if not math.isfinite(v) or v <= 0:
            raise ValueError('geometry must be finite and positive')
    if track <= 0.12:
        raise ValueError('track must be > 0.12 m for the example chassis geometry')
    settings = dict(DRIVE_DEFAULTS)
    if drive is not None:
        unknown = set(drive) - settings.keys()
        if unknown:
            raise ValueError(f'unknown gazebo drive settings: {sorted(unknown)}')
        settings.update(drive)
    if any(not math.isfinite(v) or v <= 0 for v in settings.values()):
        raise ValueError('Gazebo drive settings must be finite and positive')
    if chassis == 'ackermann':
        lf, lr = wheelbase, 0.0
    root = ET.Element('sdf', version='1.6')
    model = element(root, 'model', name='path_follow_robot')
    center = (lf - lr) / 2
    length = max(lf + lr, 0.4)

    def link(name, x, y, z, mass, shape, size, color, friction=1.0, rotation='0 0 0'):
        node = element(model, 'link', name=name)
        element(node, 'pose', f'{x} {y} {z} {rotation}')
        inertial = element(node, 'inertial')
        element(inertial, 'mass', mass)
        inertia = element(inertial, 'inertia')
        # Inertia is expressed in each link's local frame (cylinder axis is z).
        if shape == 'box':
            a, b, c = size
            moments = (mass * (b*b+c*c)/12, mass * (a*a+c*c)/12, mass * (a*a+b*b)/12)
        elif shape == 'sphere':
            moments = (2 * mass * size[0]**2 / 5,) * 3
        else:
            r, width = size
            moments = (mass * (3*r*r+width*width)/12,
                       mass * (3*r*r+width*width)/12, mass*r*r/2)
        for key, value in zip(('ixx', 'iyy', 'izz'), moments):
            element(inertia, key, value)
        for key in ('ixy', 'ixz', 'iyz'):
            element(inertia, key, 0)
        for kind in ('collision', 'visual'):
            part = element(node, kind, name=kind)
            geometry = element(part, 'geometry')
            geo = element(geometry, shape)
            if shape == 'box':
                element(geo, 'size', ' '.join(map(str, size)))
            else:
                element(geo, 'radius', size[0])
                if shape == 'cylinder':
                    element(geo, 'length', size[1])
            if kind == 'visual':
                material = element(part, 'material')
                element(material, 'ambient', color)
                element(material, 'diffuse', color)
            else:
                surface = element(part, 'surface')
                ode = element(element(surface, 'friction'), 'ode')
                element(ode, 'mu', friction)
                element(ode, 'mu2', friction)
                contact = element(element(surface, 'contact'), 'ode')
                element(contact, 'kp', 1000000)
                element(contact, 'kd', 100)
                element(contact, 'min_depth', 0.001)
        return node

    link('base_link', center, 0, radius + 0.075, 12.0, 'box',
         (length, track - 0.12, 0.12), '0.12 0.45 0.8 1')
    for i, name in enumerate(('front_left', 'front_right', 'rear_left', 'rear_right')):
        if chassis == 'differential' and i >= 2:
            continue
        x, y = (lf if i < 2 else -lr), (track/2 if i % 2 == 0 else -track/2)
        if chassis == 'differential':
            x = 0.0
        steering = chassis in ('four_wheel_steering', 'omnidirectional') or (
            chassis == 'ackermann' and i < 2)
        parent = 'base_link'
        if steering:
            # Small knuckle: no collision geometry, so it cannot strike the ground.
            knuckle = element(model, 'link', name=name + '_knuckle')
            element(knuckle, 'pose', f'{x} {y} {radius} 0 0 0')
            inertial = element(knuckle, 'inertial')
            element(inertial, 'mass', 0.1)
            inertia = element(inertial, 'inertia')
            for key in ('ixx', 'iyy', 'izz'):
                element(inertia, key, 0.0001)
            parent = name + '_knuckle'
            joint = element(model, 'joint', name=name + '_steer_joint', type='revolute')
            element(joint, 'parent', 'base_link')
            element(joint, 'child', parent)
            axis = element(joint, 'axis')
            element(axis, 'xyz', '0 0 1')
            limit = element(axis, 'limit')
            limit_angle = math.pi if chassis == 'omnidirectional' else settings['steering_limit']
            element(limit, 'lower', -limit_angle)
            element(limit, 'upper', limit_angle)
            element(limit, 'effort', 10)
            element(limit, 'velocity', 2)
            element(element(axis, 'dynamics'), 'damping', 0.1)
        link(name + '_wheel', x, y, radius, 0.5, 'cylinder', (radius, 0.06),
             '0.08 0.08 0.08 1', rotation=f'{-math.pi/2} 0 0')
        joint = element(model, 'joint', name=name + '_wheel_joint', type='revolute')
        element(joint, 'parent', parent)
        element(joint, 'child', name + '_wheel')
        axis = element(joint, 'axis')
        # Cylinder z is rotated onto model +y and follows the steering knuckle.
        element(axis, 'xyz', '0 0 1')
        limit = element(axis, 'limit')
        element(limit, 'lower', -1e16)
        element(limit, 'upper', 1e16)
        element(limit, 'effort', 8)
        element(limit, 'velocity', 100)
        element(element(axis, 'dynamics'), 'damping', 0.01)
    if chassis == 'differential':
        # Two drive wheels plus freely rotating ball supports, not four fixed
        # wheels whose lateral contact constraints would resist differential yaw.
        for name, x in (('front_ball', lf), ('rear_ball', -lr)):
            link(name, x, 0, 0.04, 0.1, 'sphere', (0.04,), '0.3 0.3 0.3 1')
            joint = element(model, 'joint', name=name + '_joint', type='ball')
            element(joint, 'parent', 'base_link')
            element(joint, 'child', name)
    plugin = element(model, 'plugin', name='physical_drive',
                     filename='libpath_follow_gazebo_drive.so')
    ros = element(plugin, 'ros')
    element(ros, 'parameter', 'true', name='use_sim_time', type='bool')
    for key, value in {
        'chassis': chassis, 'command_type': command_type, 'wheel_radius': radius,
        'track': track, 'front_axle_distance': lf, 'rear_axle_distance': lr,
        'command_topic': command_topic, 'odometry_topic': odometry_topic,
        'odom_frame': odom_frame, 'base_frame': base_frame,
        **settings,
    }.items():
        element(plugin, key, value)
    return ET.tostring(root, encoding='unicode')
