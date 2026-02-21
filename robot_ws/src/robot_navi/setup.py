from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_navi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('robot_navi/*.json')),
    ],
    install_requires=[
        'setuptools',
        'numpy'],
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
            'robot_navi = robot_navi.robot_navi:main'
        ],
    },
)
