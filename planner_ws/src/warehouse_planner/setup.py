from setuptools import find_packages, setup

package_name = 'warehouse_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # default model dir (ship a placeholder so the directory exists after install)
        ('share/' + package_name + '/models', ['models/.keep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='Event-driven MDP warehouse planner using SB3 Maskable PPO.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_inference_node = warehouse_planner.planner_inference_node:main',
            'planner_train_node = warehouse_planner.planner_train_node:main',
        ],
    },
)
