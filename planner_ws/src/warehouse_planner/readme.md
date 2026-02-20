# warehouse_planner

Event-driven MDP planner node plus an headless training environment.

## Dependencies

- `stable-baselines3`
- `sb3-contrib`
- `torch`
- `gymnasium`
- `numpy`

Install Dependencies:
```bash
pip install stable-baselines3 sb3-contrib torch gymnasium numpy
```
## Train an agent

```bash
cd asap/planner_ws/src/warehouse_planner/warehouse_planner
```
Define model_path in the the code.

```bash
nano planner_train.py
```

```bash
python3 planner_train.py
```

### Training Dashboard

```bash
tensorboard --logdir /home/norika-schneider/asap/planner_ws/src/warehouse_planner/tensor_log
```

## Run (inference)

```bash
cd asap
```
```bash
colcon build
```
```bash
source install/setup.bash
```
Start Stations:
```bash
ros2 launch station station.launch.py 
```

Start Robot (preliminary):
```bash
ros2 launch warehouse_robot warehouse_robot.launch.py
```

Start the planner:
```bash
ros2 run warehouse_planner planner_inference_node --ros-args \
  -p model_path:=/home/norika-schneider/asap/planner_ws/src/warehouse_planner/models/model.zip \
  -p max_episode_time_s:=2000.0
```

## Benchmarking

Run benchmark_planner.py for iterative processing.

Run planner_inference_node_benchmark.py for benchmarking the PPO.