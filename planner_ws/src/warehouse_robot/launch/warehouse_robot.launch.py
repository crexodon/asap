from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("pick_drop_dt", default_value="0.1"),
        DeclareLaunchArgument("move_rand_min", default_value="0.0"),
        DeclareLaunchArgument("move_rand_max", default_value="0.5"),
        DeclareLaunchArgument("battery_drain_per_s", default_value="1.0"),
        DeclareLaunchArgument("publish_location_S_as", default_value="ON_TRANSIT"),

        Node(
            package="warehouse_robot",
            executable="robot_node",
            name="robot_node",
            output="screen",
            parameters=[{
                "seed": LaunchConfiguration("seed"),
                "pick_drop_dt": LaunchConfiguration("pick_drop_dt"),
                "move_rand_min": LaunchConfiguration("move_rand_min"),
                "move_rand_max": LaunchConfiguration("move_rand_max"),
                "battery_drain_per_s": LaunchConfiguration("battery_drain_per_s"),
                "publish_location_S_as": LaunchConfiguration("publish_location_S_as"),
            }],
        ),
    ])
