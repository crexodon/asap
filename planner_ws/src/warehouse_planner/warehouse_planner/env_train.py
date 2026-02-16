from __future__ import annotations

from typing import Dict

import gymnasium as gym
import numpy as np

from constants import FLAT_ACTIONS_N, STATIONS
from encoding_train import EncodedState
from masking_train import compute_action_mask, flat_to_type_param, station_param_to_station
from training_simulator import Simulator

SUCCESS_BONUS = 1000.0
FAIL_PENALTY = 600
DROP_BONUS = 10
PICK_BONUS = 10
WAIT_PENALTY = -1


class WarehouseMDPEnv(gym.Env):
    """
    anstatt env.py
    Gymnasium env backed by a running ROS2 simulation.

    Observations are a Dict compatible with SB3 MultiInputPolicy.
    Actions are flattened to Discrete(120) to ensure compatibility with
    sb3_contrib MaskablePPO. The encoding corresponds to a logical MultiDiscrete
    (action_type, param) with:
      - action_type in {WAIT, CHARGE, MOVE_TO, PICK, DROP, PICK_A}
      - param in 0..19 (MOVE_TO uses 0..6 -> A..G)

    WAIT is handled internally: we still send a "WAIT" cmd for logging, but the
    step blocks until the first PROCESS_STARTED/PROCESS_FINISHED world event.
    """

    metadata = {"render_modes": []}

    def __init__(self, model: Simulator):
        self.model = model
        self.action_space = gym.spaces.Discrete(FLAT_ACTIONS_N)

        # Observation space
        # Note: battery is 0..100 (per your correction)
        self.observation_space = gym.spaces.Dict(
            {
                "battery_status": gym.spaces.Box(low=0.0, high=100.0, shape=(1,), dtype=np.float32),
                "robot_location": gym.spaces.Box(low=0, high=7, shape=(1,), dtype=np.int64),
                "robot_carrying_idx": gym.spaces.Box(low=-1, high=19, shape=(1,), dtype=np.int64),
                "episode_step": gym.spaces.Box(low=0, high=1_000_000, shape=(1,), dtype=np.int64),
                "delta_time": gym.spaces.Box(low=0.0, high=100, shape=(1,), dtype=np.float32),
                "package_location": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_next_station": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_shipping_type": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
                "package_lifecycle_state": gym.spaces.Box(low=0, high=5, shape=(20,), dtype=np.int64),
                "package_availability": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
            }
        )

        self._episode_step: int = 0

        self._last_step_time = model.time
        self._last_delta_time = 0.0
        # Initialize with a zero observation so action_masks() is safe before first reset().
        self._last_obs = {
            "battery_status": np.array([0.0], dtype=np.float32),
            "robot_location": np.array([0], dtype=np.int64),
            "robot_carrying_idx": np.array([-1], dtype=np.int64),
            "delta_time": np.array([0.0], dtype=np.float32),
            "package_location": np.zeros((20,), dtype=np.int64),
            "package_next_station": np.zeros((20,), dtype=np.int64),
            "package_shipping_type": np.zeros((20,), dtype=np.int64),
            "package_lifecycle_state": np.zeros((20,), dtype=np.int64),
            "package_availability": np.zeros((20,), dtype=np.int64),
        }

    # SB3-contrib hook
    def action_masks(self) -> np.ndarray:
        return compute_action_mask(self._last_obs)

    def _get_obs(self, delta_time: float) -> Dict[str, np.ndarray]:
        st = self.model.build_encoded_state(delta_time=delta_time)
        if st is None:
            # Fallback empty state
            st = EncodedState(
                battery_status=0.0,
                robot_location=0,
                robot_carrying_idx=-1,
                delta_time=float(delta_time),
                package_location=np.zeros((20,), dtype=np.int64),
                package_next_station=np.zeros((20,), dtype=np.int64),
                package_shipping_type=np.zeros((20,), dtype=np.int64),
                package_lifecycle_state=np.zeros((20,), dtype=np.int64),
                package_availability=np.zeros((20,), dtype=np.int64),
            )
        obs = st.as_dict()
        obs["episode_step"] = np.array([self._episode_step], dtype=np.int64)
        return obs

    def reset(self, seed: int | None = None, options: dict | None = None):

        self.model.reset()

        # 3) Build obs with delta_time = 0.0 at the beginning of episode
        self._episode_step = 0
        obs = self._get_obs(delta_time=0.0)
        info = {"reset_ok": True}
        return obs, info


    def step(self, action: int):
        atype, param = flat_to_type_param(int(action))

        dt = 0.0
        terminated = False
        truncated = False

        reward_pick = 0.0
        reward_drop = 0.0
        reward_wait = 0.0



        # Execute action
        if atype == 0:  # WAIT
            res, dt = self.model.cmd("WAIT", -1)
            if res:
                reward_wait = WAIT_PENALTY
            print("WAIT")

        elif atype == 1:  # CHARGE
            res, dt = self.model.cmd("CHARGE", -1)
            print("CHARGE")

        elif atype == 2:  # MOVE_TO
            res, dt = self.model.cmd("MOVE_TO", param)
            print(f"MOVE_TO{param}")

        elif atype == 3:  # PICK
            res, dt = self.model.cmd("PICK", param)
            if res:
                reward_pick = PICK_BONUS
            print(f"PICK{param}")

        elif atype == 4:  # DROP
            res, dt = self.model.cmd("DROP", param)
            if res:
                reward_drop = DROP_BONUS
            print(f"DROP{param}")

        elif atype == 5:  # PICK_A
            res, dt = self.model.cmd("PICK_A", param)
            print(f"PICK_A{param}")

        else:
            dt = 0.0

        if res == False:
            print("invalid action!!!!")

        # Reward is negative actual elapsed time
        reward = -float(dt) + reward_pick + reward_drop + reward_wait
        battery_depleted = False

        # Episode termination logic
        terminated = self.model.is_done()
                
        # Battery depleted ends the episode as terminal failure
        if self.model.robot_battery <= 0.0:
            battery_depleted = True
            terminated = True
            truncated = False
            reward -= FAIL_PENALTY
            print("battery is empty")

        # Success bonus (only if we are terminating successfully)
        # Note: if both battery_depleted and is_done could be True (shouldn't happen),
        # failure penalty already applied; you can prioritize one explicitly if needed.
        if terminated and (not battery_depleted) and self.model.is_done():
            reward += SUCCESS_BONUS
            print("SUCCESSFUL")

        if self.model.episode_elapsed_s() >= self.model.max_episode_time_s:
            truncated = True
            print("Time elapsed")

        # Update obs
        self._last_delta_time = dt
        self._last_step_time = self.model.time
        self._last_obs = self._get_obs(delta_time=dt)
        info = {"dt": float(dt)}
        self._episode_step += 1
        return self._last_obs, reward, terminated, truncated, info
