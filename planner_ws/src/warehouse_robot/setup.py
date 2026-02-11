from setuptools import find_packages, setup

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/warehouse_robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noris_ASAP',
    maintainer_email='n/a',
    description='Discrete training robot node implementing /planner/cmd without Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = warehouse_robot.robot_node:main',
        ],
    },
)
