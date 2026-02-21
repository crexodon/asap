# Project for Autonomous Systems: Architecture and Planning

## Setup
1. Source the ROS2 Jazzy underlay
2. Install missing dependencies `rosdep install -i --from-path src --rosdistro jazzy -y`
3. Build with `colcon build`
4. Source the overlay `source install/local_setup.bash`

python3 -m venv asap_venv --system-site-packages
mia@reimu-rosbox:~/Projects/asap$ source asap_venv/bin/activate
(asap_venv) mia@reimu-rosbox:~/Projects/asap$ pip install stable-baselines3 sb3-contrib torch gymnasium numpy