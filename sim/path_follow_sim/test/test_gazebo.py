"""Opt-in real Gazebo/ODE regressions (not a kinematic mock).

Source the install space, then PATH_FOLLOW_TEST_GAZEBO=1 python3 -m pytest test/test_gazebo.py -v.
"""
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest

from path_follow_sim.gazebo_model import robot_sdf


@pytest.mark.parametrize('chassis,count', [
    ('ackermann', 2), ('differential', 0),
    ('omnidirectional', 4), ('four_wheel_steering', 4),
])
def test_generated_robot(chassis, count):
    root = ET.fromstring(robot_sdf(chassis, 'twist'))
    model = root.find('model')
    joints = model.findall('joint')
    assert len([j for j in joints if j.get('name').endswith('_wheel_joint')]) == (
        2 if chassis == 'differential' else 4)
    assert len([j for j in joints if j.get('name').endswith('_steer_joint')]) == count
    assert not model.findall('.//static')  # the body must move under physics
    assert model.find('plugin').get('filename') == 'libpath_follow_gazebo_drive.so'
    assert float(model.find('plugin/rear_axle_distance').text) == (
        0.0 if chassis == 'ackermann' else 0.325)
    assert len(model.findall('.//collision')) == 5


def test_reject_invalid_geometry():
    with pytest.raises(ValueError):
        robot_sdf('ackermann', 'twist', radius=float('nan'))
    with pytest.raises(ValueError):
        robot_sdf('ackermann', 'twist', track=0.1)
    with pytest.raises(ValueError):
        robot_sdf('ackermann', 'twist', drive={'command_timeout': 0})


def test_drive_settings():
    root = ET.fromstring(robot_sdf('ackermann', 'twist', drive={
        'wheel_torque': 4.0, 'steering_limit': 0.6}))
    assert float(root.find('model/plugin/wheel_torque').text) == 4.0
    bounds = root.find("./model/joint[@name='front_left_steer_joint']/axis/limit")
    assert float(bounds.find('upper').text) == 0.6


physical = pytest.mark.skipif(
    os.environ.get('PATH_FOLLOW_TEST_GAZEBO') != '1',
    reason='opt-in: requires Gazebo Classic, built plugin and sourced install space')


@physical
@pytest.mark.parametrize('chassis,scenario,keep', [
    ('ackermann', 's_curve', 'false'),
    ('differential', 'arc', 'false'),
    ('omnidirectional', 'lane_change', 'true'),
    ('four_wheel_steering', 'lane_change', 'true'),
])
def test_physical_closed_loop(tmp_path, chassis, scenario, keep):
    env = dict(os.environ, ROS_DOMAIN_ID='177', ROS_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI='http://127.0.0.1:11477')
    cmd = ['ros2', 'launch', 'path_follow_sim', 'gazebo.launch.py',
           'gui:=false', 'rviz:=false', 'exit_on_finish:=true', f'chassis:={chassis}',
           f'scenario:={scenario}', f'keep_heading:={keep}',
           f'output_dir:={tmp_path}', 'timeout:=45.0']
    process = subprocess.Popen(
        cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=110)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
    (tmp_path / 'launch.log').write_text(output)
    assert process.returncode == 0, output[-6000:]
    assert 'Physical joint drive ready' in output, output[-6000:]
    assert 'Failed to load plugin' not in output
    summary = json.loads((tmp_path / 'summary.json').read_text())
    assert summary['samples'] > 200, summary
    assert summary['passed'], summary


@physical
@pytest.mark.parametrize('chassis,command_type', [
    ('ackermann', 'twist_stamped'), ('differential', 'twist'),
    ('omnidirectional', 'twist'), ('four_wheel_steering', 'four_wheel_steering'),
])
def test_joint_motion_and_watchdog(tmp_path, monkeypatch, chassis, command_type):
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from path_follow_mpc.msg import FourWheelSteeringCommand
    from sensor_msgs.msg import JointState

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    env = dict(os.environ, ROS_DOMAIN_ID='178', ROS_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI='http://127.0.0.1:11478')
    with (tmp_path / 'drive.log').open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'path_follow_sim', 'gazebo.launch.py', 'gui:=false',
            'rviz:=false',
            'launch_controller:=false', f'chassis:={chassis}', f'cmd_type:={command_type}',
            f'output_dir:={tmp_path}'], env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context, domain_id=178)
        node = rclpy.create_node('physical_drive_test', context=context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        poses, joints = [], []
        node.create_subscription(Odometry, 'odom', poses.append, 10)
        node.create_subscription(JointState, 'joint_states', joints.append, 10)
        msg_type = {'twist': Twist, 'twist_stamped': TwistStamped,
                    'four_wheel_steering': FourWheelSteeringCommand}[command_type]
        pub = node.create_publisher(msg_type, 'cmd_vel', 10)

        def spin(duration, message=None):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if message is not None:
                    pub.publish(message)
                executor.spin_once(timeout_sec=0.03)

        try:
            end = time.monotonic() + 30
            while (not poses or not joints or pub.get_subscription_count() == 0):
                assert process.poll() is None, (tmp_path / 'drive.log').read_text()
                assert time.monotonic() < end, (
                    f'poses={len(poses)}, joints={len(joints)}, '
                    f'subscribers={pub.get_subscription_count()}\n' +
                    (tmp_path / 'drive.log').read_text())
                executor.spin_once(timeout_sec=0.1)
            spin(0.5)
            start = poses[-1].pose.pose.position
            message = msg_type()
            if command_type == 'four_wheel_steering':
                message.front_steering_angle = message.rear_steering_angle = 0.2
                spin(0.7, message)  # Native messages preserve steering at zero speed.
                angles = [v for n, v in zip(joints[-1].name, joints[-1].position)
                          if n.endswith('_steer_joint')]
                assert len(angles) == 4
                assert all(abs(v - 0.2) < 0.04 for v in angles)
                message.longitudinal_velocity = 0.4
            else:
                twist = message.twist if command_type == 'twist_stamped' else message
                if chassis == 'omnidirectional':
                    twist.linear.y = 0.3
                elif chassis == 'differential':
                    twist.angular.z = 0.5
                else:
                    twist.linear.x, twist.angular.z = 0.4, 0.15
            spin(3.0, message)
            end_pose = poses[-1].pose.pose
            assert max(abs(v) for sample in joints for v in sample.velocity) > 0.5
            if chassis == 'differential':
                assert abs(end_pose.orientation.z) > 0.2  # physical in-place rotation
            elif chassis in ('omnidirectional', 'four_wheel_steering'):
                assert end_pose.position.y - start.y > (0.4 if chassis == 'omnidirectional' else 0.1)
            else:
                assert end_pose.position.x - start.x > 0.5
            spin(1.5)  # No new commands: plugin watchdog must brake the motors.
            twist = poses[-1].twist.twist
            assert abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.angular.z) < 0.05
            spin(0.5, message)
            if command_type == 'four_wheel_steering':
                message.longitudinal_velocity = float('nan')
            elif command_type == 'twist_stamped':
                message.twist.linear.x = float('nan')
            else:
                message.linear.x = float('nan')
            pub.publish(message)
            spin(1.0)  # invalid input must brake; allow wheel inertia to settle
            twist = poses[-1].twist.twist
            assert abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.angular.z) < 0.08
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
