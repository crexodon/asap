import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# https://docs.ros.org/en/jazzy/How-To-Guides/Launch-file-different-formats.html
def generate_launch_description():
    # Get package directories
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    world_file = PathJoinSubstitution([
        pkg_robot_bringup,
        'worlds',
        'default.sdf'
    ])
    
    config_file = PathJoinSubstitution([
        pkg_robot_bringup,
        'config',
        'robot_params.yaml'
    ])

    models_path = PathJoinSubstitution([
        pkg_robot_bringup,
        'models'
    ])

    set_gazebo_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )
    
    # Launch arguments, sim time needed for gazebo!
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo Harmonic (gz-sim)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r'],  # -r for auto-start
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Bridge: Gazebo topics <-> ROS2 topics
    # [ = bridge from gz to ros
    # ] = bridge from ros to gz
    # @ = bidirectional bridge
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Odometry: Gazebo -> ROS2
            '/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # Cmd_vel: ROS2 -> Gazebo
            '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # Clock for simulation time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Differential Drive Controller
    diff_drive_controller = Node(
        package='robot_hardware',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        set_gazebo_model_path,
        gz_sim,
        gz_ros_bridge,
        diff_drive_controller,
    ])