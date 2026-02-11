from __future__ import annotations

import os
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .env import WarehouseMDPEnv
from .ros_interface import WarehouseROSInterface

import time


def _resolve_default_model_dir() -> Path:
    """Prefer package share/models, fallback to CWD."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("warehouse_planner"))
        model_dir = share / "models"
        model_dir.mkdir(parents=True, exist_ok=True)
        # Check writability (installed shares can be read-only)
        test_file = model_dir / ".write_test"
        test_file.write_text("ok")
        test_file.unlink()
        return model_dir
    except Exception:
        cwd_dir = Path(os.getcwd()) / "warehouse_planner_models"
        cwd_dir.mkdir(parents=True, exist_ok=True)
        return cwd_dir


def main():
    rclpy.init()

    ros = WarehouseROSInterface()
    ros.declare_parameter("total_timesteps", 50_000)
    ros.declare_parameter("save_name", "model.zip")

    total_timesteps = int(ros.get_parameter("total_timesteps").value)
    save_name = str(ros.get_parameter("save_name").value)

    env = WarehouseMDPEnv(ros)

    try:
        from sb3_contrib import MaskablePPO
    except Exception:
        ros.get_logger().error(
            "Missing RL dependencies. Install with: pip install stable-baselines3 sb3-contrib torch gymnasium"
        )
        raise

    model_dir = _resolve_default_model_dir()
    model_path = model_dir / save_name

    if model_path.exists():
        ros.get_logger().info(f"Loading existing model: {model_path}")
        model = MaskablePPO.load(str(model_path), env=env)
    else:
        ros.get_logger().info("Creating new MaskablePPO model (MultiInputPolicy)")
        model = MaskablePPO(
            "MultiInputPolicy",
            env,
            verbose=1,
            tensorboard_log=str(model_dir / "tb"),
        )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros)

    # Train in the same process: spin in background so the env can receive callbacks
    spin_thread = None
    try:
        import threading

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # --- IMPORTANT: one clean reset at startup ---
        # Ensures job_handler emits EPISODE_RESET and all nodes (robot/stations/spawner) synchronize.
        ros.get_logger().info("Triggering initial /reset_episode before training ...")
        try:
            ros.reset_episode(num_packages=20, timeout_s=5.0)
        except Exception as e:
            ros.get_logger().error(f"Initial reset failed: {e}")
            raise
        # Give the system a short moment to process EPISODE_RESET and publish fresh robot_state
        time.sleep(0.2)

        ros.get_logger().info(f"Training for total_timesteps={total_timesteps} ...")
        model.learn(total_timesteps=total_timesteps, progress_bar=False)

        ros.get_logger().info(f"Saving model to: {model_path}")
        model.save(str(model_path))

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
