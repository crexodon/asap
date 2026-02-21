# Project for Autonomous Systems: Architecture and Planning
## Dependencies
- Ubuntu 22.04 with ROS2 Jazzy and Gazebo installed
    - Ideally inside a VM
- The following Python packages as system install
`pip install --break-system-packages stable-baselines3 sb3-contrib torch gymnasium numpy`
    - this is needed since ROS2's Colcon overwrites any virtual env
## Setup
1. Source the ROS2 Jazzy underlay
2. Install missing dependencies `rosdep install -i --from-path src --rosdistro jazzy -y`
3. Build with `colcon build` from root folder
4. Source the overlay `source install/local_setup.bash`

## Workspaces
### Planner WS
- This workspace contains the reinforcment agent
- Additional python code to train the agent
- a simulated warehouse_robot for trainig

### Robot WS
- Contains all packages for the robot
    - A battery controller to simulate the battery with gazebo sim time
    - A differential drive controller to simulate a drivetrain inside gazebo
- Contains a navigation package that implements A* to navigate in the world
    - This package also contains the core logic for the robot's behavior and interaction with the RL agent

### Station WS
- Contains a package for simulating the stations
    - Stations are generically coded and then spawned with a job_spawner and job_handler node

### Root/Interface WS
- This is a additional workspace that contains all the interfaces, actions and messages that are used inside this ros network

### Simulation WS
- This is the meta workspace to launch ROS2 with all Nodes

## Startup
- Make sure tosource under- and overlay!
- If not already done, generate the A* grid in robot_mavi
- Launch with `ros2 launch warehouse_simulation launch_sim.launch.py`
    - This launches all the nodes to drive the planner in inference (runnning) mode
    - it also spawns Gazebo to visualize