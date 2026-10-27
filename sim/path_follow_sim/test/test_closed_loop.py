"""端到端回归：真实启动仿真器 + MPC，四种底盘各跑一次，检查指标通过。

需要先 source 安装空间（含 path_follow_mpc 与 path_follow_sim）。
"""
import json
import os
import shutil
import subprocess
import tempfile

import pytest

CASES = [
    ('ackermann', 's_curve', 'false'),
    ('differential', 'circle', 'false'),
    ('omnidirectional', 'lane_change', 'true'),
    ('four_wheel_steering', 'lane_change', 'true'),
]


@pytest.mark.skipif(shutil.which('ros2') is None, reason='ROS 2 not sourced')
@pytest.mark.parametrize('chassis,scenario,keep', CASES)
def test_closed_loop(chassis, scenario, keep):
    out = tempfile.mkdtemp()
    env = dict(os.environ, ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1')
    cmd = ['ros2', 'launch', 'path_follow_sim', 'sim.launch.py', f'chassis:={chassis}',
           f'scenario:={scenario}', f'keep_heading:={keep}', f'output_dir:={out}', 'timeout:=40.0']
    subprocess.run(cmd, env=env, timeout=120, capture_output=True)
    with open(os.path.join(out, 'summary.json')) as f:
        summary = json.load(f)
    shutil.rmtree(out)
    assert summary['passed'], summary
