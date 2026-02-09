# Robot
This workspace is for the robot hardware and it's needed packages. It is loosely based on ros2_control but written mainly in python.

| Package Type | Build Type | Purpose | Contains |
|-------------|------------|---------|----------|
| **robot_hardware** | ament_python | Python nodes | Executable scripts |
| **robot_msgs** | ament_cmake | Custom interfaces | .msg, .srv files |
| **robot_description** | ament_cmake | Robot model | URDF, meshes |
| **robot_bringup** | ament_python | System launch | Launch files |

## Launching
### Run individual nodes
- `ros2 run robot_hardware diff_drive_controller`
- `ros2 run robot_hardware battery_controller`
- `ros2 run robot_hardware gripper_controller`

# Or use launch file to start all at once
- `ros2 launch robot_bringup hardware.launch.py`