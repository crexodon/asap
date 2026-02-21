import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_name = 'warehouse_simulation'
    pkg_simulation = get_package_share_directory('warehouse_simulation')
    pkg_share = get_package_share_directory(pkg_name)
    
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    station_pkg = get_package_share_directory('station')
    planner_pkg = get_package_share_directory('warehouse_planner')

    # 1. Gazebo starten (Harmonic ist Standard in Jazzy)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


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

    models_path = PathJoinSubstitution([
        pkg_simulation,
        'urdf'
    ])

    set_gazebo_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Differential Drive Controller
    diff_drive_controller = Node(
        package='robot_hardware',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # Differential Drive Controller
    battery_controller = Node(
        package='robot_hardware',
        executable='battery_controller',
        name='battery_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    robot_navi = Node(
        package='robot_navi',
        executable='robot_navi',
        name='robot_navi',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(station_pkg, 'launch', 'station.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Warehouse Planner (RL inference)
    planner_inference = Node(
        package='warehouse_planner',
        executable='planner_inference_node',  # Entry point from setup.py
        name='planner_inference_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'model_path': ('share/' + planner_pkg + '/models', ['models/model.zip'])},
            {'max_episode_time_s': 2000.0},
            {'wait_cancel_immediately': True}
        ]
    )

    # # 3. Spawner
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
        '/model/warehouse_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/model/warehouse_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        spawn_robot,
        bridge,
        diff_drive_controller,
        battery_controller,
        robot_navi,
        station_launch,
        planner_inference
    ])