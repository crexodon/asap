from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .constants import FLAT_ACTIONS_N
from .env import WarehouseMDPEnv
from .masking import compute_action_mask, flat_to_type_param, station_param_to_station
from .ros_interface import WarehouseROSInterface


def _resolve_default_model_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("warehouse_planner"))
        return share / "models" / "model.zip"
    except Exception:
        return Path(os.getcwd()) / "warehouse_planner_models" / "model.zip"


def _action_to_cmd(flat_action: int) -> str:
    atype, param = flat_to_type_param(int(flat_action))
    if atype == 0:
        return "WAIT"
    if atype == 1:
        return "CHARGE"
    if atype == 2:
        return f"MOVE_TO {station_param_to_station(param)}"
    if atype == 3:
        return f"PICK {param}"
    if atype == 4:
        return f"DROP {param}"
    if atype == 5:
        return f"PICK_A {param}"
    return "WAIT"


def main():
    rclpy.init()
    ros = WarehouseROSInterface()

    ros.declare_parameter("model_path", str(_resolve_default_model_path()))
    ros.declare_parameter("decision_sleep_s", 0.05)

    model_path = Path(str(ros.get_parameter("model_path").value))
    decision_sleep_s = float(ros.get_parameter("decision_sleep_s").value)

    try:
        from sb3_contrib import MaskablePPO
    except Exception:
        ros.get_logger().error(
            "Missing RL dependencies. Install with: pip install stable-baselines3 sb3-contrib torch gymnasium"
        )
        raise

    if not model_path.exists():
        ros.get_logger().error(f"Model file not found: {model_path}")
        raise FileNotFoundError(str(model_path))

    env = WarehouseMDPEnv(ros)

    ros.get_logger().info(f"Loading model: {model_path}")
    model = MaskablePPO.load(str(model_path))

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros)

    # Spin in background so callbacks keep state fresh
    import threading

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Initial obs
    obs, _ = env.reset()

    try:
        while rclpy.ok():
            # Build action mask
            mask = compute_action_mask(obs)

            # Predict with mask
            action, _ = model.predict(obs, action_masks=mask, deterministic=True)

            cmd = _action_to_cmd(int(action))
            ros.get_logger().info(f"Chosen action={int(action)} cmd='{cmd}'")

            # Execute a single env step (will block until done / wait interrupt)
            obs, reward, terminated, truncated, info = env.step(int(action))
            ros.get_logger().info(
                f"Step finished: reward={reward:.3f} dt={info.get('dt', 0.0):.3f} terminated={terminated} truncated={truncated}"
            )

            if terminated or truncated:
                ros.get_logger().info("Episode ended -> resetting")
                obs, _ = env.reset()

            time.sleep(decision_sleep_s)

    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            ros.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
