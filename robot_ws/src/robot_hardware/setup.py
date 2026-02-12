from setuptools import find_packages, setup

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crexodon',
    maintainer_email='github@crexy.moe',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'battery_controller = robot_hardware.battery_controller:main',
            'diff_drive_controller = robot_hardware.diff_drive_controller:main',
            #'gripper_controller = robot_hardware.gripper_controler:main'
        ],
    },
)
