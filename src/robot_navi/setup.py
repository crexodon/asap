from setuptools import find_packages, setup

package_name = 'robot_navi'

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
    maintainer='Crexodon',
    maintainer_email='ros@crexy.moe',
    description='Simulated robot navigation node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_navi = robot_navi.robot_navi:main'
        ],
    },
)
