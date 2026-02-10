from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="warehouse_planner",
                executable="world_state_aggregator",
                name="world_state_aggregator",
                output="screen",
                parameters=[
                    {"coalesce_ms": 80},
                    {"timeout_tick_s": 2.0},
                    {"battery_threshold_s": 15.0},
                    {"num_packages": 20},
                ],
            ),
            Node(
                package="warehouse_planner",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[
                    {"use_model": True},
                    {"model_path": ""},  # set to your .zip
                ],
            ),
        ]
    )
