from __future__ import annotations

import os
from pathlib import Path


from env_train import WarehouseMDPEnv
from training_simulator import Simulator


from stable_baselines3.common.callbacks import BaseCallback
from sb3_contrib import MaskablePPO

import time

from pathlib import Path


class StopOnMaxEpisodes(BaseCallback):
    def __init__(self, max_episodes: int, verbose: int = 0):
        super().__init__(verbose)
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


    total_timesteps = 50_000
    save_name = "model.zip"
    max_episodes = 1000
    max_episode_time_s = 600
    wait_cancel_immediately = True

    headless = Simulator()

    callback = StopOnMaxEpisodes(max_episodes) if max_episodes > 0 else None

    # define path for model
    model_path = Path("/home/norika-schneider/asap/planner_ws/src/warehouse_planner/models/model.zip")
    tensor_path = Path("/home/norika-schneider/asap/planner_ws/src/warehouse_planner/tensor_log")

    env = WarehouseMDPEnv(headless)


    if model_path.exists():
        print(f"Loading existing model: {model_path}")
        model = MaskablePPO.load(str(model_path), env=env)
    else:
        print("Creating new MaskablePPO model (MultiInputPolicy)")
        model = MaskablePPO(
            "MultiInputPolicy",
            env,
            verbose=1,
            tensorboard_log=str(tensor_path / "tb"),
        )

    headless.reset()
    time.sleep(0.2)

    print(f"Training: total_timesteps={total_timesteps}, max_episodes={max_episodes if max_episodes>0 else 'disabled'} (will save to {model_path}" )

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
        progress_bar=True,
    )

    
    # Definiere den Pfad zur Datei
    print(f"Saving model to: {model_path}")


    # Jetzt kannst du sicher speichern
    model.save(model_path)


if __name__ == "__main__":
    main()
