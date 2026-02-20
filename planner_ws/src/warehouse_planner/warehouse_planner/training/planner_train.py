from __future__ import annotations

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

        dones = self.locals.get("dones")
        if dones is not None:

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

    total_timesteps = 100 # 400.000
    max_episodes = 10_000
    max_episode_time_s = 2000

    headless = Simulator()

    callback = StopOnMaxEpisodes(max_episodes) if max_episodes > 0 else None

    # define path for model
    model_name = "model.zip"
    model_path_strin = f"../../models/{model_name}"
    model_path = Path(model_path_strin)
    tensor_path = Path("../../tensor_log")

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
    
    print(f"Saving model to: {model_path}")
    model.save(model_path)


if __name__ == "__main__":
    main()
