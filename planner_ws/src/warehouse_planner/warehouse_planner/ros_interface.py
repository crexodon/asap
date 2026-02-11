from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from interfaces.msg import RobotState, WorldEvent
from interfaces.srv import GetPackages, ResetEpisode

import numpy as np


try:
    # Expected by your specification
    from interfaces.action import PlannerCmd
except Exception:  # pragma: no cover
    PlannerCmd = None  # type: ignore

from .constants import STATIONS
from .encoding import (
    EncodedState,
    LIFECYCLE_TO_IDX,
    PACKAGE_LOCATION_TO_IDX,
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


class WarehouseROSInterface(Node):
    """ROS2 interface node.

    - Caches /robot_state
    - Provides synchronous helpers for /get_packages, /reset_episode and /planner/cmd
    - Provides WAIT interruption via /world_event (PROCESS_STARTED / PROCESS_FINISHED)

    Note: WAIT is treated internally (no blocking action goal), but we still send
    a "WAIT" goal for logging and immediately cancel it (to avoid piling up goals).
    """

    def __init__(self):
        super().__init__("warehouse_planner")

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("max_episode_time_s", 600.0)
        self.declare_parameter("wait_cancel_immediately", True)

        self.max_episode_time_s = float(self.get_parameter("max_episode_time_s").value)
        self.wait_cancel_immediately = bool(self.get_parameter("wait_cancel_immediately").value)

        # State cache
        self._robot_state_lock = threading.Lock()
        self._robot_state: Optional[RobotState] = None
        self._last_state_update_time = time.monotonic()

        # World events for WAIT
        self._event_cv = threading.Condition()
        self._last_interrupt_event_time: Optional[float] = None

        # Clients
        self.cli_get_packages = self.create_client(GetPackages, "/get_packages", callback_group=self.cb_group)
        self.cli_reset = self.create_client(ResetEpisode, "/reset_episode", callback_group=self.cb_group)

        self.sub_robot_state = self.create_subscription(RobotState, "/robot_state", self._on_robot_state, 10)
        self.sub_world_event = self.create_subscription(WorldEvent, "/world_event", self._on_world_event, 10)

        self.act_cmd: Optional[ActionClient] = None
        if PlannerCmd is not None:
            self.act_cmd = ActionClient(self, PlannerCmd, "/planner/cmd", callback_group=self.cb_group)
        else:
            self.get_logger().warning(
                "Could not import interfaces.action.PlannerCmd. /planner/cmd client will be disabled."
            )

        self._episode_start_time = time.monotonic()

    # ---------------- callbacks ----------------
    def _on_robot_state(self, msg: RobotState):
        with self._robot_state_lock:
            self._robot_state = msg
            self._last_state_update_time = time.monotonic()

    def _on_world_event(self, msg: WorldEvent):
        if msg.event_type not in ("PROCESS_STARTED", "PROCESS_FINISHED"):
            return
        # Only the first relevant event should interrupt WAIT; subsequent ones are ignored until WAIT restarts.
        with self._event_cv:
            if self._last_interrupt_event_time is None:
                self._last_interrupt_event_time = time.monotonic()
                self._event_cv.notify_all()

    # ---------------- helpers ----------------
    def wait_for_robot_state(self, timeout_s: float = 5.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._robot_state_lock:
                if self._robot_state is not None:
                    return True
            time.sleep(0.01)
        return False

    def _call_service_blocking(self, client, req, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=min(1.0, timeout_s)):
            return None
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout_s:
                return None
            time.sleep(0.01)
        try:
            return fut.result()
        except Exception:
            return None

    def get_packages(self, timeout_s: float = 5.0):
        req = GetPackages.Request()
        req.all = True
        return self._call_service_blocking(self.cli_get_packages, req, timeout_s=timeout_s)

    def reset_episode(self, num_packages: int = 20, timeout_s: float = 5.0) -> bool:
        req = ResetEpisode.Request()
        req.num_packages = int(num_packages)
        res = self._call_service_blocking(self.cli_reset, req, timeout_s=timeout_s)
        if res is None:
            self.get_logger().error("/reset_episode failed (no response)")
            return False
        if not res.ok:
            self.get_logger().error(f"/reset_episode rejected: {res.error}")
            return False
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

    def wait_for_interrupt_event(self, timeout_s: Optional[float] = None) -> float:
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
            nxt[i] = safe_idx(PACKAGE_LOCATION_TO_IDX, str(p.next_location), 0)
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
