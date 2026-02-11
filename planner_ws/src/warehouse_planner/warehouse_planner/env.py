from __future__ import annotations

import time
import math
from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

from .constants import FLAT_ACTIONS_N, STATIONS
from .encoding import EncodedState
from .masking import compute_action_mask, flat_to_type_param, station_param_to_station
from .ros_interface import WarehouseROSInterface


class WarehouseMDPEnv(gym.Env):
    """Gymnasium env backed by a running ROS2 simulation.

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

    def __init__(self, ros: WarehouseROSInterface):
        super().__init__()
        self.ros = ros
        self.action_space = gym.spaces.Discrete(FLAT_ACTIONS_N)

        # Observation space
        # Note: battery is 0..100 (per your correction)
        self.observation_space = gym.spaces.Dict(
            {
                "battery_status": gym.spaces.Box(low=0.0, high=100.0, shape=(1,), dtype=np.float32),
                "robot_location": gym.spaces.Box(low=0, high=7, shape=(1,), dtype=np.int64),
                "robot_carrying_idx": gym.spaces.Box(low=-1, high=19, shape=(1,), dtype=np.int64),
                "delta_time": gym.spaces.Box(low=0.0, high=1e6, shape=(1,), dtype=np.float32),
                "package_location": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_next_station": gym.spaces.Box(low=0, high=7, shape=(20,), dtype=np.int64),
                "package_shipping_type": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
                "package_lifecycle_state": gym.spaces.Box(low=0, high=5, shape=(20,), dtype=np.int64),
                "package_availability": gym.spaces.Box(low=0, high=1, shape=(20,), dtype=np.int64),
            }
        )

        self._last_step_time = time.monotonic()
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
        st = self.ros.build_encoded_state(delta_time=delta_time)
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
        return st.as_dict()

    def reset(self, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)

        # 1) Reset the world (job_handler emits EPISODE_RESET; robot/stations/spawner react)
        # Avoid doing this if the caller explicitly disabled it
        do_reset = True
        if options and isinstance(options, dict):
            do_reset = bool(options.get("do_reset", True))

        if do_reset:
            # You can make num_packages configurable; for now fixed 20
            ok = self.ros.reset_episode(num_packages=20, timeout_s=5.0)
            if not ok:
                raise RuntimeError("Env.reset(): /reset_episode returned False")

        # 2) Wait until the world is "ready enough" for an initial observation:
        # - have robot_state
        # - have packages list (len == 20)
        t0 = time.monotonic()
        timeout_s = 5.0

        last_err = None
        while time.monotonic() - t0 < timeout_s:
            try:
                # Ensure we have at least one robot_state cached
                if not self.ros.wait_for_robot_state(timeout_s=0.2):
                    time.sleep(0.02)
                    continue

                # Ensure get_packages works and returns full set
                pkgs_res = self.ros.get_packages(timeout_s=2.0)
                if pkgs_res is None:
                    time.sleep(0.02)
                    continue
                if not hasattr(pkgs_res, "packages"):
                    time.sleep(0.02)
                    continue
                if len(pkgs_res.packages) != 20:
                    time.sleep(0.02)
                    continue

                # Ready
                break

            except Exception as e:
                last_err = e
                time.sleep(0.05)
                continue
        else:
            # If we get here, timeout.
            # Fail loudly: training on a half-reset world is worse than stopping.
            raise RuntimeError(f"Env.reset() timed out waiting for robot_state/packages. last_err={last_err}")

        # 3) Build obs with delta_time = 0.0 at the beginning of episode
        obs = self._get_obs(delta_time=0.0)
        info = {"reset_ok": True}
        return obs, info


    def step(self, action: int):
        atype, param = flat_to_type_param(int(action))

        dt = 0.0
        terminated = False
        truncated = False

        # Execute action
        if atype == 0:  # WAIT
            self.ros.send_wait_for_logging()
            waited = self.ros.wait_for_interrupt_event(timeout_s=1.0)
            # If no event occurred, force a bounded WAIT duration so agent can act again
            dt = float(waited) if waited is not None else 1.0
            if dt is None:
                dt = 1.0

        elif atype == 1:  # CHARGE
            res = self.ros.send_cmd("CHARGE")
            dt = res.dt
        elif atype == 2:  # MOVE_TO
            station = station_param_to_station(param)
            res = self.ros.send_cmd(f"MOVE_TO {station}")
            dt = res.dt
        elif atype == 3:  # PICK
            res = self.ros.send_cmd(f"PICK {param}")
            dt = res.dt
        elif atype == 4:  # DROP
            res = self.ros.send_cmd(f"DROP {param}")
            dt = res.dt
        elif atype == 5:  # PICK_A
            res = self.ros.send_cmd(f"PICK_A {param}")
            dt = res.dt
        else:
            dt = 0.0

        # Reward is negative actual elapsed time
        reward = -float(dt)

        # Episode termination logic
        terminated = self.ros.is_done()

        # Truncation: battery empty or max episode time
        # battery comes from obs and is 0..100
        if self.ros.wait_for_robot_state(timeout_s=0.1):
            with self.ros._robot_state_lock:
                rs = self.ros._robot_state
            if rs is not None and float(rs.battery) <= 0.0:
                truncated = True

        if self.ros.episode_elapsed_s() >= self.ros.max_episode_time_s:
            truncated = True

        # Update obs
        self._last_delta_time = dt
        self._last_step_time = time.monotonic()
        self._last_obs = self._get_obs(delta_time=dt)
        info = {"dt": float(dt)}
        return self._last_obs, reward, terminated, truncated, info
