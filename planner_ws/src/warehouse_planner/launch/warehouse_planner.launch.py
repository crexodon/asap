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


    # Inference arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Path to a trained model .zip file. If empty, use package share default.",
    )

    # Environment / episode args (shared)
    max_episode_time_s_arg = DeclareLaunchArgument(
        "max_episode_time_s",
        default_value="1000.0",
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
        description="Internal: derived executable name. Use 'planner_inference_node'.",
    )


    return LaunchDescription(
        [
            mode_arg,
            mode_executable_arg,
            model_path_arg,
            max_episode_time_s_arg,
            wait_cancel_immediately_arg,
            log_level_arg,
            chooser_node,
        ]
    )
