from setuptools import setup
import os
from glob import glob

package_name = 'station'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='norikaschneider',
    maintainer_email='noschneide@stud.hs-heilbronn.com',
    description='Warehouse RL ROS2 Jazzy package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'job_handler = station.job_handler_node:main',
            'station = station.station_node:main',
            'robot_interface = station.robot_interface_node:main',
            'job_spawner = station.job_spawner_node:main',
            'world_state_aggregator = station.world_state_aggregator_node:main',
        ],
    },
)
