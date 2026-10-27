from setuptools import setup

package_name = 'path_follow_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'sim_robot = path_follow_sim.sim_node:main',
            'scenario_publisher = path_follow_sim.scenario_node:main',
            'metrics_recorder = path_follow_sim.metrics_node:main',
            'sdf_to_params = path_follow_sim.sdf_model:main',
            'plot_result = path_follow_sim.plot_result:main',
        ],
    },
)
