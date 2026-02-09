from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="station",
            executable="job_handler",
            name="job_handler",
            output="screen",
        ),

        Node(
            package="station",
            executable="job_spawner",
            name="job_spawner",
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
    ])
