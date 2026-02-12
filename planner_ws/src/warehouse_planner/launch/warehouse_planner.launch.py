from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Common arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="inference",
        description="Planner mode: 'train' or 'inference'.",
    )

    # Training arguments
    total_timesteps_arg = DeclareLaunchArgument(
        "total_timesteps",
        default_value="200000",
        description="Total timesteps for training (train mode only).",
    )
    save_name_arg = DeclareLaunchArgument(
        "save_name",
        default_value="model.zip",
        description="Filename to save the trained model into the package share dir (train mode only).",
    )

    # Inference arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Path to a trained model .zip file. If empty, use package share default.",
    )

    # Environment / episode args (shared)
    max_episode_time_s_arg = DeclareLaunchArgument(
        "max_episode_time_s",
        default_value="600.0",
        description="Maximum episode time in seconds before truncation/reset.",
    )
    wait_cancel_immediately_arg = DeclareLaunchArgument(
        "wait_cancel_immediately",
        default_value="true",
        description="If true, send WAIT for logging and immediately cancel it on the action server.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level.",
    )

    mode = LaunchConfiguration("mode")

    # Nodes
    train_node = Node(
        package="warehouse_planner",
        executable="planner_train_node",
        name="warehouse_planner_train",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {"total_timesteps": LaunchConfiguration("total_timesteps")},
            {"save_name": LaunchConfiguration("save_name")},
            {"max_episode_time_s": LaunchConfiguration("max_episode_time_s")},
            {"wait_cancel_immediately": LaunchConfiguration("wait_cancel_immediately")},
        ],
        condition=None,  # set below using if/else in Python (LaunchCondition optional)
    )

    inference_node = Node(
        package="warehouse_planner",
        executable="planner_inference_node",
        name="warehouse_planner_inference",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"max_episode_time_s": LaunchConfiguration("max_episode_time_s")},
            {"wait_cancel_immediately": LaunchConfiguration("wait_cancel_immediately")},
        ],
        condition=None,
    )

    # Avoid extra imports by using 'mode' in a simple way:
    # We create both nodes but enable exactly one via 'ros__parameters' trick isn't right.
    # Instead, we rely on the common pattern of two separate launch files in one.
    # Here: choose by passing a different executable via mode. This is robust and simple.
    chooser_node = Node(
        package="warehouse_planner",
        executable=LaunchConfiguration("mode_executable"),
        name="warehouse_planner",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {"total_timesteps": LaunchConfiguration("total_timesteps")},
            {"save_name": LaunchConfiguration("save_name")},
            {"model_path": LaunchConfiguration("model_path")},
            {"max_episode_time_s": LaunchConfiguration("max_episode_time_s")},
            {"wait_cancel_immediately": LaunchConfiguration("wait_cancel_immediately")},
        ],
    )

    mode_executable_arg = DeclareLaunchArgument(
        "mode_executable",
        default_value="planner_inference_node",
        description="Internal: derived executable name. Use 'planner_train_node' or 'planner_inference_node'.",
    )

    # NOTE:
    # ROS 2 launch does not support easy string== comparisons without additional conditions.
    # So we expose 'mode_executable' directly and keep 'mode' for readability.
    # Recommended usage:
    #   Training:   mode_executable:=planner_train_node
    #   Inference:  mode_executable:=planner_inference_node

    return LaunchDescription(
        [
            mode_arg,
            mode_executable_arg,
            total_timesteps_arg,
            save_name_arg,
            model_path_arg,
            max_episode_time_s_arg,
            wait_cancel_immediately_arg,
            log_level_arg,
            chooser_node,
        ]
    )
