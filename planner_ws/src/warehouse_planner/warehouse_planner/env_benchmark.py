from __future__ import annotations

import time
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
WAIT_PENALTY = 0


class WarehouseMDPEnv(gym.Env):
    """Gymnasium env backed for inference with with headless environment
    """

    def __init__(self, ros: Simulator):
        self.ros = ros
        self.action_space = gym.spaces.Discrete(FLAT_ACTIONS_N)

        # Observation space
        self.observation_space = gym.spaces.Dict(
            {
                "battery_status": gym.spaces.Box(low=0.0, high=100.0, shape=(1,), dtype=np.float32),
                "robot_location": gym.spaces.Box(low=0, high=7, shape=(1,), dtype=np.int64),
                "robot_carrying_idx": gym.spaces.Box(low=-1, high=19, shape=(1,), dtype=np.int64),
                "time": gym.spaces.Box(low=0.0, high=1000, shape=(1,), dtype=np.float32),
                "package_location": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_next_station": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_shipping_type": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
                "package_lifecycle_state": gym.spaces.Box(low=0, high=5, shape=(20,), dtype=np.int64),
                "package_availability": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
            }
        )

        self._episode_step: int = 0
        self._last_step_time = ros.time
        self._last_delta_time = 0.0

        # Initialize with a zero observation so action_masks() is safe before first reset().
        self._last_obs = {
            "battery_status": np.array([0.0], dtype=np.float32),
            "robot_location": np.array([0], dtype=np.int64),
            "robot_carrying_idx": np.array([-1], dtype=np.int64),
            "time": np.array([0.0], dtype=np.float32),
            "package_location": np.zeros((20,), dtype=np.int64),
            "package_next_station": np.zeros((20,), dtype=np.int64),
            "package_shipping_type": np.zeros((20,), dtype=np.int64),
            "package_lifecycle_state": np.zeros((20,), dtype=np.int64),
            "package_availability": np.zeros((20,), dtype=np.int64),
        }

    def action_masks(self) -> np.ndarray:
        return compute_action_mask(self._last_obs)

    def _get_obs(self, time: float) -> Dict[str, np.ndarray]:
        st = self.ros.build_encoded_state(time=time)
        if st is None:
            # Fallback empty state
            st = EncodedState(
                battery_status=0.0,
                robot_location=0,
                robot_carrying_idx=-1,
                time=float(time),
                package_location=np.zeros((20,), dtype=np.int64),
                package_next_station=np.zeros((20,), dtype=np.int64),
                package_shipping_type=np.zeros((20,), dtype=np.int64),
                package_lifecycle_state=np.zeros((20,), dtype=np.int64),
                package_availability=np.zeros((20,), dtype=np.int64),
            )
        obs = st.as_dict()
       
        return obs

    def reset(self, seed: int | None = None, options: dict | None = None):

        self.ros.reset()

        # Build obs with time = 0.0 at the beginning of episode
        self._episode_step = 0
        obs = self._get_obs(time=0.0)
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
        penalty_charge = 0.0


        # Execute action
        if atype == 0:  # WAIT
            res, dt = self.ros.cmd("WAIT", -1)
            if res:
                reward_wait = WAIT_PENALTY
            print("WAIT")

        elif atype == 1:  # CHARGE
            penalty_charge = self.ros.robot_battery / 5 # large penalty if charging is performed at high battery status
            res, dt = self.ros.cmd("CHARGE", -1)
            if not res:
                penalty_charge = 0.0
                
            print("CHARGE")

        elif atype == 2:  # MOVE_TO
            res, dt = self.ros.cmd("MOVE_TO", param)
            print(f"MOVE_TO{param}")

        elif atype == 3:  # PICK
            res, dt = self.ros.cmd("PICK", param)
            if res:
                reward_pick = PICK_BONUS
            print(f"PICK{param}")

        elif atype == 4:  # DROP
            res, dt = self.ros.cmd("DROP", param)
            if res:
                reward_drop = DROP_BONUS
            print(f"DROP{param}")

        elif atype == 5:  # PICK_A
            res, dt = self.ros.cmd("PICK_A", param)
            if res:
                reward_pick = PICK_BONUS
            print(f"PICK_A{param}")

        else:
            dt = 0.0

        if res == False:
            print("invalid action!!!!")

        # Reward is negative actual elapsed time
        reward = -2* float(dt) + reward_pick + reward_drop + reward_wait - penalty_charge
        battery_depleted = False

        # Episode termination logic
        terminated = self.ros.is_done()

        # Battery depleted ends the episode as terminal failure
        if self.ros.robot_battery <= 0.0:
            battery_depleted = True
            terminated = True
            truncated = False
            reward -= FAIL_PENALTY
            print("battery is empty")

        # Success bonus 
        if terminated and (not battery_depleted) and self.ros.is_done():
            reward += SUCCESS_BONUS
            print("SUCCESSFUL")

        if self.ros.episode_elapsed_s() >= self.ros.max_episode_time_s:
            truncated = True
            print("Time elapsed")

        # Update obs
        self._last_delta_time = dt
        self._last_step_time = time.monotonic()
        self._last_obs = self._get_obs(time=self._last_step_time)
        info = {"dt": float(dt)}
        self._episode_step += 1
        return self._last_obs, reward, terminated, truncated, info
