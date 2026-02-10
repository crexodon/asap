from setuptools import setup

package_name = "warehouse_planner"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/planner.launch.py"]),
    ],
    install_requires=['setuptools', 'sb3-contrib', 'stable-baselines3'],
    zip_safe=True,
    maintainer="norika-schneider",
    maintainer_email="norika.schneider@gmail.com",
    description="Event-driven planner + world state aggregator (Maskable PPO ready)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_state_aggregator = warehouse_planner.world_state_aggregator_node:main",
            "planner_node = warehouse_planner.planner_node:main",
        ],
    },
)
