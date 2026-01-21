# Project for Autonomous Systems: Architecture and Planning

## Setup
1. Source the ROS2 Jazzy underlay
2. Install missing dependencies `rosdep install -i --from-path src --rosdistro jazzy -y`
3. Build with `colcon build`
4. Source the overlay `source install/local_setup.bash`

## Simulation
start simulation and spawn the robot with:
 `ros2 launch warehouse_simulation launch_sim.launch.py`