import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'warehouse_simulation'
    pkg_share = get_package_share_directory(pkg_name)
    
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # 1. Gazebo starten (Harmonic ist Standard in Jazzy)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Robot State Publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 3. Spawner
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'warehouse_robot',
            '-x', '1.0', '-y', '13.0', '-z', '0.15' # Höhe etwas reduziert
        ],
        output='screen'
    )

    # 4. Bridge
    # Beachte: Jazzy nutzt oft gz_sim statt ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' # [ sorgt für Richtung GZ -> ROS
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_robot,
        bridge
    ])