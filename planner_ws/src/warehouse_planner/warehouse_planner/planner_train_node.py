from __future__ import annotations

import os
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .env import WarehouseMDPEnv
from .ros_interface import WarehouseROSInterface

import time

from stable_baselines3.common.callbacks import BaseCallback


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

from stable_baselines3.common.callbacks import BaseCallback

class StopOnMaxEpisodes(BaseCallback):
    def __init__(self, max_episodes: int):
        super().__init__()
        self.max_episodes = int(max_episodes)
        self.episodes = 0

    def _on_step(self) -> bool:
        # In SB3 liegt "dones" bei VecEnv in self.locals
        dones = self.locals.get("dones")
        if dones is not None:
            # bei n_envs>1 könnten mehrere true sein
            self.episodes += int(dones.sum()) if hasattr(dones, "sum") else int(any(dones))
        else:
            done = self.locals.get("done")
            if done:
                self.episodes += 1

        if self.episodes >= self.max_episodes:
            print(f"[StopOnMaxEpisodes] Reached {self.episodes}/{self.max_episodes} episodes -> stopping.")
            return False
        return True



def main():
    rclpy.init()

    ros = WarehouseROSInterface()
    ros.declare_parameter("total_timesteps", 50_000)
    ros.declare_parameter("save_name", "model.zip")
    ros.declare_parameter("max_episodes", 0)

    total_timesteps = int(ros.get_parameter("total_timesteps").value)
    save_name = str(ros.get_parameter("save_name").value)
    max_episodes = int(ros.get_parameter("max_episodes").value)

    callback = StopOnMaxEpisodes(max_episodes) if max_episodes > 0 else None

    env = WarehouseMDPEnv(ros)

    from sb3_contrib import MaskablePPO

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

    try:
        import threading, time

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        ros.get_logger().info("Triggering initial /reset_episode before training ...")
        ros.reset_episode(num_packages=20, timeout_s=5.0)
        time.sleep(0.2)

        ros.get_logger().info(
            f"Training: total_timesteps={total_timesteps}, max_episodes={max_episodes if max_episodes>0 else 'disabled'}"
        )

        model.learn(
            total_timesteps=total_timesteps,
            callback=callback,
            progress_bar=True,
        )

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
