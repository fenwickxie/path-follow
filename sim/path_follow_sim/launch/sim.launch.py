"""通用仿真启动：运动学底盘 + 场景路径 + 指标记录 (+ 可选 MPC 控制器)。

示例：
  ros2 launch path_follow_sim sim.launch.py chassis:=ackermann scenario:=s_curve
  ros2 launch path_follow_sim sim.launch.py chassis:=four_wheel_steering scenario:=lane_change keep_heading:=true
  # 测试别的算法：不启动 MPC，让你的节点订阅 cmd_vel/odom/plan 即可
  ros2 launch path_follow_sim sim.launch.py launch_controller:=false

关键参数全部在 config/sim.yaml 与下面的 launch 参数里，不需要改源码。
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

# 每种底盘默认使用的指令类型（可用 cmd_type:= 覆盖）
DEFAULT_CMD = {'ackermann': 'ackermann', 'differential': 'twist',
               'omnidirectional': 'twist', 'four_wheel_steering': 'four_wheel_steering'}

ARGS = {
    'chassis': ('ackermann', 'ackermann / differential / omnidirectional / four_wheel_steering'),
    'cmd_type': ('', 'twist / twist_stamped / ackermann / four_wheel_steering，空=按底盘默认'),
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
    'exit_on_finish': ('true', '完成后自动关闭整个仿真（CI 用）'),
}


def _setup(context):
    a = {k: context.launch_configurations[k] for k in ARGS}
    sim_yaml = a['sim_config'] or get_package_share_directory('path_follow_sim') + '/config/sim.yaml'
    cmd_type = a['cmd_type'] or DEFAULT_CMD[a['chassis']]
    geometry = {'wheelbase': float(a['wheelbase']),
                'front_axle_distance': float(a['front_axle_distance']),
                'rear_axle_distance': float(a['rear_axle_distance'])}

    sim = Node(package='path_follow_sim', executable='sim_robot', name='sim_robot', output='screen',
               parameters=[sim_yaml, {'chassis': a['chassis'], 'cmd_type': cmd_type, **geometry}])
    scenario = Node(package='path_follow_sim', executable='scenario_publisher',
                    name='scenario_publisher', output='screen',
                    parameters=[sim_yaml, {'scenario': a['scenario'],
                                           'keep_heading': a['keep_heading'].lower() == 'true',
                                           'heading': float(a['heading'])}])
    metrics = Node(package='path_follow_sim', executable='metrics_recorder',
                   name='metrics_recorder', output='screen',
                   parameters=[sim_yaml, {'output_dir': a['output_dir'],
                                          'timeout': float(a['timeout']),
                                          'exit_on_finish': True}])
    actions = [sim, scenario, metrics]

    if a['launch_controller'].lower() == 'true':
        mpc_yaml = a['mpc_config'] or get_package_share_directory('path_follow_mpc') + '/config/mpc.yaml'
        # 后面的字典覆盖 YAML：保证仿真与控制器的底盘、指令类型、几何一致
        actions.append(Node(
            package='path_follow_mpc', executable='mpc_node', name='mpc_tracker', output='screen',
            parameters=[mpc_yaml, {'chassis': a['chassis'], 'output.type': cmd_type,
                                   'limits.wheelbase': geometry['wheelbase'],
                                   'limits.front_axle_distance': geometry['front_axle_distance'],
                                   'limits.rear_axle_distance': geometry['rear_axle_distance']}]))

    if a['exit_on_finish'].lower() == 'true':
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=metrics, on_exit=[EmitEvent(event=Shutdown(reason='test finished'))])))
    return actions


def generate_launch_description():
    decls = [DeclareLaunchArgument(k, default_value=v[0], description=v[1]) for k, v in ARGS.items()]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
