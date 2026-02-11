from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='norikaschneider',
    maintainer_email='noschneide@stud.hs-heilbronn.com',
    description='Discrete training robot node implementing /planner/cmd without Gazebo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_node = warehouse_robot.robot_node:main',
        ],
    },
)
