# warehouse_planner

Training: ros2 run warehouse_planner planner_train_node --ros-args -p total_timesteps:=50000 -p save_name:=model.zip

Interference: ros2 run warehouse_planner planner_inference_node --ros-args -p model_path:=.../model.zip