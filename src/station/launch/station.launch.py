from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_spawner = LaunchConfiguration("use_spawner")
    seed = LaunchConfiguration("seed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_spawner",
            default_value="true",
            description="Whether to start the job_spawner node."
        ),
        DeclareLaunchArgument(
            "seed",
            default_value="1",
            description="Seed for reset/spawn."
        ),

        Node(
            package="station",
            executable="job_handler",
            name="job_handler",
            output="screen",
        ),

        # Stations A..G
        Node(package="station", executable="station", name="station_a", output="screen",
             parameters=[{"station_id": "A"}]),
        Node(package="station", executable="station", name="station_b", output="screen",
             parameters=[{"station_id": "B"}]),
        Node(package="station", executable="station", name="station_c", output="screen",
             parameters=[{"station_id": "C"}]),
        Node(package="station", executable="station", name="station_d", output="screen",
             parameters=[{"station_id": "D"}]),
        Node(package="station", executable="station", name="station_e", output="screen",
             parameters=[{"station_id": "E"}]),
        Node(package="station", executable="station", name="station_f", output="screen",
             parameters=[{"station_id": "F"}]),
        Node(package="station", executable="station", name="station_g", output="screen",
             parameters=[{"station_id": "G"}]),

        # spawner
        Node(
            package="station",
            executable="job_spawner",
            name="job_spawner",
            output="screen",
            condition=None,  
        ),
    ])
