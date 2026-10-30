"""通用仿真启动：运动学底盘 + 场景路径 + 指标记录 (+ 可选 MPC 控制器)。

示例：
  ros2 launch path_follow_sim sim.launch.py chassis:=ackermann scenario:=s_curve
  ros2 launch path_follow_sim sim.launch.py chassis:=four_wheel_steering scenario:=lane_change keep_heading:=true
  # 测试别的算法：不启动 MPC，让你的节点订阅 cmd_vel/odom/plan 即可
  ros2 launch path_follow_sim sim.launch.py launch_controller:=false

关键参数全部在 config/sim.yaml 与下面的 launch 参数里，不需要改源码。
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from path_follow_sim.gazebo_model import robot_sdf

# 每种底盘默认使用的指令类型（可用 cmd_type:= 覆盖）
DEFAULT_CMD = {'ackermann': 'ackermann', 'differential': 'twist',
               'omnidirectional': 'twist', 'four_wheel_steering': 'four_wheel_steering'}

ARGS = {
    'backend': ('gazebo', 'gazebo（默认，Gazebo Classic 11） / kinematic（轻量快速）'),
    'gui': ('true', 'Gazebo 后端是否显示窗口；无显示环境设 false'),
    'rviz': ('true', '启动 RViz 显示参考路径、Odometry 与 TF；无显示环境设 false'),
    'rviz_config': ('', '自定义 RViz 配置文件，空=使用本包默认配置'),
    'world': ('', 'Gazebo world 文件，空=本包离线平地场景'),
    'track': ('0.5', 'Gazebo 轮距 [m]'),
    'wheel_radius': ('0.1', 'Gazebo 轮半径 [m]'),
    'chassis': ('ackermann', 'ackermann / differential / omnidirectional / four_wheel_steering'),
    'cmd_type': ('', 'twist / twist_stamped / ackermann / four_wheel_steering，空=按底盘默认'),
    'path_topic': ('', '规划路径话题，空=沿用 sim_config 的 topics.path'),
    'scenario': ('s_curve', 'line / arc / s_curve / circle / figure8 / lane_change'),
    'keep_heading': ('false', '路径朝向固定（蟹行/全向平移测试）'),
    'heading': ('0.0', 'keep_heading 时的固定朝向 [rad]'),
    'wheelbase': ('0.65', '轴距 [m]（可用 sdf_to_params 从模型估算）'),
    'front_axle_distance': ('0.325', '四轮转向：参考点到前轴 [m]'),
    'rear_axle_distance': ('0.325', '四轮转向：参考点到后轴 [m]'),
    'launch_controller': ('true', '是否同时启动 MPC；false 则测试你自己的算法'),
    'mpc_config': ('', '外部 MPC YAML，空=使用 path_follow_mpc 默认配置'),
    'sim_config': ('', '外部仿真 YAML，空=使用本包 config/sim.yaml'),
    'output_dir': ('/tmp/path_follow_sim', '指标/CSV 输出目录'),
    'timeout': ('60.0', '单次测试超时 [s]'),
    'exit_on_finish': ('false', 'true=完成后自动关闭；false=保留 Gazebo/仿真，Ctrl+C 退出'),
}


def _setup(context):
    a = {k: context.launch_configurations[k] for k in ARGS}
    if a['backend'] not in ('kinematic', 'gazebo'):
        raise ValueError('backend must be kinematic or gazebo')
    if a['chassis'] not in DEFAULT_CMD:
        raise ValueError(f'unknown chassis: {a["chassis"]}')
    physical = a['backend'] == 'gazebo'
    sim_yaml = a['sim_config'] or get_package_share_directory('path_follow_sim') + '/config/sim.yaml'
    with open(sim_yaml, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    params = config.get('/**', {}).get('ros__parameters', {})
    params.update(config.get('sim_robot', {}).get('ros__parameters', {}))
    topics = params.get('topics', {})
    frames = params.get('frames', {})
    path_topic = a['path_topic'] or topics.get('path', 'plan')
    odom_topic = topics.get('odometry', 'odom')
    command_topic = topics.get('command', 'cmd_vel')
    cmd_type = a['cmd_type'] or DEFAULT_CMD[a['chassis']]
    geometry = {'wheelbase': float(a['wheelbase']),
                'front_axle_distance': float(a['front_axle_distance']),
                'rear_axle_distance': float(a['rear_axle_distance'])}

    clock = {'use_sim_time': physical}
    scenario = Node(package='path_follow_sim', executable='scenario_publisher',
                    name='scenario_publisher', output='screen',
                    parameters=[sim_yaml, clock, {'topic': path_topic, 'scenario': a['scenario'],
                                           'keep_heading': a['keep_heading'].lower() == 'true',
                                           'heading': float(a['heading'])}])
    metrics = Node(package='path_follow_sim', executable='metrics_recorder',
                   name='metrics_recorder', output='screen',
                   parameters=[sim_yaml, clock, {'topics.path': path_topic,
                                          'output_dir': a['output_dir'],
                                          'timeout': float(a['timeout']),
                                          'exit_on_finish': True}])
    experiment = [scenario, metrics]
    rviz_node = None
    if a['rviz'].lower() == 'true':
        rviz_path = a['rviz_config'] or os.path.join(
            get_package_share_directory('path_follow_sim'), 'rviz', 'simulation.rviz')
        rviz_node = Node(
            package='rviz2', executable='rviz2', name='simulation_rviz', output='screen',
            arguments=['-d', rviz_path], parameters=[clock],
            remappings=[('/plan', path_topic), ('/odom', odom_topic)],
            # VS Code's Snap GTK module path is incompatible with Ubuntu's system glibc.
            additional_env={
                'GTK_PATH': '', 'GTK_EXE_PREFIX': '', 'GTK_MODULES': '',
                'GDK_PIXBUF_MODULEDIR': '', 'GDK_PIXBUF_MODULE_FILE': '',
                'GTK_IM_MODULE_FILE': '',
            })

    if a['launch_controller'].lower() == 'true':
        mpc_yaml = a['mpc_config'] or get_package_share_directory('path_follow_mpc') + '/config/mpc.yaml'
        # 后面的字典覆盖 YAML：保证仿真与控制器的底盘、指令类型、几何一致
        experiment.append(Node(
            package='path_follow_mpc', executable='mpc_node', name='mpc_tracker', output='screen',
            parameters=[mpc_yaml, clock, {'chassis': a['chassis'], 'output.type': cmd_type,
                                   'topics.path': path_topic,
                                   'topics.odometry': odom_topic,
                                   'topics.command': command_topic,
                                   'limits.wheelbase': geometry['wheelbase'],
                                   'limits.front_axle_distance': geometry['front_axle_distance'],
                                   'limits.rear_axle_distance': geometry['rear_axle_distance']}]))

    actions = []
    if a['exit_on_finish'].lower() == 'true':
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=metrics, on_exit=[EmitEvent(event=Shutdown(reason='test finished'))])))
    if not physical:
        sim = Node(package='path_follow_sim', executable='sim_robot', name='sim_robot',
                   output='screen', parameters=[sim_yaml, clock, {
                       'chassis': a['chassis'], 'cmd_type': cmd_type,
                       'topics.path': path_topic, 'topics.odometry': odom_topic,
                       'topics.command': command_topic, **geometry}])
        return actions + [sim] + experiment + ([rviz_node] if rviz_node else [])

    # Resolve optional packages only for Gazebo; missing dependencies are explicit errors.
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin = os.path.join(get_package_prefix('path_follow_gazebo'), 'lib',
                          'libpath_follow_gazebo_drive.so')
    if not os.path.isfile(plugin):
        raise FileNotFoundError(f'Gazebo drive plugin not built: {plugin}')
    topics['path'] = path_topic
    xml = robot_sdf(a['chassis'], cmd_type, geometry['wheelbase'],
                    geometry['front_axle_distance'], geometry['rear_axle_distance'],
                    float(a['track']), float(a['wheel_radius']),
                    command_topic, odom_topic,
                    frames.get('odom', 'odom'), frames.get('base', 'base_link'),
                    params.get('gazebo'))
    # Keep filenames in SDF ASCII: Classic's XML entity conversion corrupts Chinese
    # absolute paths. dlopen via the environment preserves the original filesystem path.
    actions.append(SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH', os.path.dirname(plugin) + os.pathsep +
        os.environ.get('GAZEBO_PLUGIN_PATH', '')))
    directory = tempfile.TemporaryDirectory(prefix='path_follow_gazebo_')
    model_file = os.path.join(directory.name, 'robot.sdf')
    with open(model_file, 'w', encoding='utf-8') as stream:
        stream.write(xml)
    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: directory.cleanup() or [])])))
    world = a['world'] or os.path.join(
        get_package_share_directory('path_follow_sim'), 'worlds', 'tracking.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': a['gui'], 'verbose': 'true'}.items())
    initial = params.get('initial_pose', {})
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'path_follow_robot', '-file', model_file,
                            '-x', str(initial.get('x', 0.0)),
                            '-y', str(initial.get('y', 0.0)), '-z', '0.02',
                            '-Y', str(initial.get('yaw', 0.0)), '-timeout', '60'])

    def spawned(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazebo robot spawn failed; see spawn log'))]
        return experiment + ([rviz_node] if rviz_node else [])

    actions += [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
                gazebo, spawn]
    return actions


def generate_launch_description():
    decls = [DeclareLaunchArgument(k, default_value=v[0], description=v[1]) for k, v in ARGS.items()]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
