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
cd asap/planner_ws/src/warehouse_planner/warehouse_planner/training
```
Define model name in the code: "model_name" in line 52

```bash
nano planner_train.py
```

```bash
python3 planner_train.py
```

### Training Dashboard

```bash
cd asap/planner_ws/src/warehouse_planner
```

```bash
tensorboard --logdir tensor_log
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

Here we compare the PPO planner with an iterative planner. We use the headless environment to get quick results.


The iterative planner acts after following rules:

- Handle packages iteratively and transport and process in required order:
A→ B→ (G if B failed) → C →D/E → (E if D failed)
- Before each motion check if battery is smaller than 15 % and go charging if necessary

To run the iterative planner:

```bash
cd asap/planner_ws/src/warehouse_planner/warehouse_planner/benchmarking
```
```bash
python3 iterative_planner.py
```

To run the PPO Planner with the headless environment for comparison:

```bash
cd asap/planner_ws/src/warehouse_planner/warehouse_planner/benchmarking
```

Define model name in the code: "model_name" in line 31

```bash
nano planner_inference_node_benchmark.py
```

```bash
python3 planner_inference_node_benchmark.py
```
