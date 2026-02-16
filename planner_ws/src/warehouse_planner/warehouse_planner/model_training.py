from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from training_simulator import Simulator, PackageState


import numpy as np



from constants import STATIONS
from encoding_train import (
    EncodedState,
    LIFECYCLE_TO_IDX,
    PACKAGE_LOCATION_TO_IDX,
    NEXT_LOCATION_TO_IDX,
    ROBOT_LOCATION_TO_IDX,
    SHIPPING_TO_IDX,
    safe_idx,
    carrying_from_packages,
)


@dataclass
class CmdResult:
    ok: bool
    dt: float
    raw_result: str


class WarehouseModel():
    """Model used for training fro stations and robot.
        anstatt ros_interface
    """

    def __init__(self, max_episode_time_s, wait_cancel_immediately):


        # Parameters

        self.max_episode_time_s = float(max_episode_time_s)
        self.wait_cancel_immediately = bool(wait_cancel_immediately)

        self._episode_start_time = time.monotonic()

        

    # ---------------- callbacks ----------------
    def _on_robot_state(self):
        self._robot_state = self.robot.robot_location

    def get_packages(self):
        packages_info =self.job_handler.get_packages()
        return packages_info

    def reset_episode(self, num_packages: int = 20, timeout_s: float = 5.0) -> bool:
        self.station_A.reset_episode()
        self.station_B.reset_episode()
        self.station_C.reset_episode()
        self.station_D.reset_episode()
        self.station_E.reset_episode()
        self.station_F.reset_episode()
        self.station_G.reset_episode()
        self.job_spawner.reset_episode()
        self.job_handler.reset_episode()
        self.robot.reset_episode()
        self._episode_start_time = time.monotonic()
        return True

    def episode_elapsed_s(self) -> float:
        return float(time.monotonic() - self._episode_start_time)

    def send_cmd(self, cmd: str, timeout_s: float = 60.0) -> CmdResult:
        """Send a non-WAIT command and block until result."""
        if self.act_cmd is None:
            return CmdResult(False, 0.0, "NO_ACTION_CLIENT")

        if not self.act_cmd.wait_for_server(timeout_sec=2.0):
            return CmdResult(False, 0.0, "NO_SERVER")

        goal = PlannerCmd.Goal()
        goal.cmd = str(cmd)
        send_future = self.act_cmd.send_goal_async(goal)

        t0 = time.monotonic()
        while not send_future.done():
            if time.monotonic() - t0 > 2.0:
                return CmdResult(False, 0.0, "SEND_TIMEOUT")
            time.sleep(0.01)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return CmdResult(False, 0.0, "REJECTED")

        result_future = goal_handle.get_result_async()
        t1 = time.monotonic()
        while not result_future.done():
            if time.monotonic() - t1 > timeout_s:
                # Attempt cancel
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return CmdResult(False, 0.0, "RESULT_TIMEOUT")
            time.sleep(0.01)

        res_wrap = result_future.result()
        if res_wrap is None:
            return CmdResult(False, 0.0, "NO_RESULT")

        res = res_wrap.result
        ok = (res.result == "OK")
        return CmdResult(ok, float(res.dt), str(res.result))

    def send_wait_for_logging(self) -> None:
        """Send WAIT command only for logging.

        We don't rely on the result. By default we cancel immediately to avoid
        accumulating long-running goals on the server.
        """
        if self.act_cmd is None:
            return
        if not self.act_cmd.wait_for_server(timeout_sec=0.1):
            return

        goal = PlannerCmd.Goal()
        goal.cmd = "WAIT"
        send_future = self.act_cmd.send_goal_async(goal)

        def _done_cb(fut):
            try:
                gh = fut.result()
                if gh is None or (not gh.accepted):
                    return
                if self.wait_cancel_immediately:
                    try:
                        gh.cancel_goal_async()
                    except Exception:
                        pass
            except Exception:
                pass

        send_future.add_done_callback(_done_cb)

    def wait_for_interrupt_event(self, timeout_s: Optional[float] = None) -> Optional[float]:
        """Block until first PROCESS_STARTED/FINISHED event arrives.

        Returns real elapsed seconds spent waiting.
        """
        with self._event_cv:
            self._last_interrupt_event_time = None
            t0 = time.monotonic()
            if timeout_s is None:
                while self._last_interrupt_event_time is None:
                    self._event_cv.wait(timeout=0.25)
            else:
                deadline = time.monotonic() + float(timeout_s)
                while self._last_interrupt_event_time is None and time.monotonic() < deadline:
                    self._event_cv.wait(timeout=0.25)
            t1 = time.monotonic()
        if self._last_interrupt_event_time is None and timeout_s is not None:
            return None
        return float(t1 - t0)

    def build_encoded_state(self, delta_time: float) -> Optional[EncodedState]:
        if not self.wait_for_robot_state(timeout_s=2.0):
            return None

        pkgs_res = self.get_packages(timeout_s=2.0)
        if pkgs_res is None:
            return None

        with self._robot_state_lock:
            rs = self._robot_state
        if rs is None:
            return None

        # Prepare arrays
        n = 20
        loc = np.zeros((n,), dtype=np.int64)
        nxt = np.zeros((n,), dtype=np.int64)
        ship = np.zeros((n,), dtype=np.int64)
        life = np.zeros((n,), dtype=np.int64)
        avail = np.zeros((n,), dtype=np.int64)

        for p in pkgs_res.packages:
            i = int(p.package_idx)
            if i < 0 or i >= n:
                continue
            loc[i] = safe_idx(PACKAGE_LOCATION_TO_IDX, str(p.current_location), 0)
            # next_location is station id or "FINISH"
            nxt[i] = safe_idx(NEXT_LOCATION_TO_IDX, str(p.next_location), 0)
            ship[i] = safe_idx(SHIPPING_TO_IDX, str(p.shipping_type), 0)
            life[i] = safe_idx(LIFECYCLE_TO_IDX, str(p.lifecycle_state), 0)
            avail[i] = 1 if bool(p.availability) else 0

        # Prefer SoT from package location
        carrying = carrying_from_packages(loc)

        return EncodedState(
            battery_status=float(rs.battery),
            robot_location=safe_idx(ROBOT_LOCATION_TO_IDX, str(rs.robot_location), 0),
            robot_carrying_idx=carrying,
            delta_time=float(delta_time),
            package_location=loc,
            package_next_station=nxt,
            package_shipping_type=ship,
            package_lifecycle_state=life,
            package_availability=avail,
        )

    def is_done(self) -> bool:
        pkgs_res = self.get_packages(timeout_s=2.0)
        if pkgs_res is None:
            return False
        done_count = 0
        for p in pkgs_res.packages:
            if str(p.next_location) == "FINISH":
                done_count += 1
        return done_count >= 20
