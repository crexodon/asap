# warehouse_planner (ROS2 Jazzy, Python)

Event-driven MDP planner node plus an online-training entrypoint.

## Interfaces

- Subscribes: `/robot_state`, `/world_event`
- Service client: `/get_packages` (`interfaces/srv/GetPackages`)
- Service client: `/reset_episode` (`interfaces/srv/ResetEpisode`)
- Action client: `/planner/cmd` (`interfaces/action/PlannerCmd`)

## Actions (flattened)

Internally, the logical action is a tuple `(action_type, param)`:

- `WAIT` (param ignored)
- `CHARGE` (param ignored)
- `MOVE_TO` with `param 0..6 -> A..G`
- `PICK` with `param 0..19`
- `DROP` with `param 0..19` (masked to the carried package)
- `PICK_A` with `param 0..19`

For MaskablePPO compatibility we train and run with a flattened `Discrete(120)` encoding:

`flat = action_type * 20 + param`

## Reward / episode end

- Reward is `-dt` where `dt` is the *actual* time returned by `/planner/cmd` for non-WAIT actions.
- For `WAIT`, `dt` is measured as real wall time between entering WAIT and the first world event of type `PROCESS_STARTED` or `PROCESS_FINISHED`.
- `terminated=True` when all 20 packages have `next_location == "FINISH"`.
- `truncated=True` when `battery <= 0` or when `episode_elapsed_s >= max_episode_time_s` (default: 600s; ROS parameter).

## Dependencies

This package expects these Python libs to be available in your environment:

- `stable-baselines3`
- `sb3-contrib`
- `torch`
- `gymnasium`
- `numpy`

1. Create and activate virtual environment
```bash
python3 -m venv ros2-asap
```
```bash
source ros2-asap/bin/activate
```

2. Source ROS 2
```bash
source /opt/ros/jazzy/setup.bash
```

3. Install Dependencies:
```bash
pip install stable-baselines3 sb3-contrib torch gymnasium numpy
```

## Run (inference)

```bash
ros2 run warehouse_planner planner_inference_node \
  --ros-args -p model_path:=/path/to/model.zip
```

By default, it looks for `share/warehouse_planner/models/model.zip`.

## Run (online training)

```bash
ros2 run warehouse_planner planner_train_node \
  --ros-args -p total_timesteps:=50000 -p save_name:=model.zip
```

The node will spin in a background thread so the env can receive ROS callbacks during training.

## Notes about WAIT logging

WAIT is handled *internally* (the node waits for the next PROCESS_STARTED/PROCESS_FINISHED event), but it still sends the action goal `cmd="WAIT"` for logging.

To avoid accumulating long-running WAIT goals on the server, `wait_cancel_immediately` defaults to `true`.
