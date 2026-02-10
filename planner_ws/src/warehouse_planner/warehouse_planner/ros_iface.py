from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.msg import RobotState, WorldEvent
from interfaces.srv import GetPackages, ResetEpisode

try:
    from interfaces.action import PlannerCmd  # expected by user spec
except Exception:  # pragma: no cover
    PlannerCmd = None


@dataclass
class CachedRobotState:
    tick_id: int = 0
    robot_location: str = "ON_TRANSIT"
    battery_status: float = 100.0


class RosInterface(Node):
    """ROS I/O layer: subscribes robot/world events and calls services/actions."""

    def __init__(self):
        super().__init__("warehouse_planner_iface")

        self._lock = threading.Lock()
        self._robot_state = CachedRobotState()

        # Condition to wake WAIT when PROCESS_STARTED/PROCESS_FINISHED arrives
        self._wait_cond = threading.Condition()
        self._wait_event_seen: bool = False
        self._wait_event_msg: Optional[WorldEvent] = None

        self.sub_robot = self.create_subscription(RobotState, "/robot_state", self._on_robot_state, 10)
        self.sub_world = self.create_subscription(WorldEvent, "/world_event", self._on_world_event, 50)

        self.cli_get_packages = self.create_client(GetPackages, "/get_packages")
        self.cli_reset = self.create_client(ResetEpisode, "/reset_episode")

        self._action_client = None
        if PlannerCmd is not None:
            self._action_client = ActionClient(self, PlannerCmd, "/planner/cmd")

    # ---------------- callbacks ----------------
    def _on_robot_state(self, msg: RobotState) -> None:
        with self._lock:
            self._robot_state.tick_id = int(msg.tick_id)
            self._robot_state.robot_location = str(msg.robot_location)
            # user clarified battery_status is 0..100
            self._robot_state.battery_status = float(msg.battery)

    def _on_world_event(self, msg: WorldEvent) -> None:
        # WAIT interruption events
        if msg.event_type not in ("PROCESS_FINISHED", "PROCESS_STARTED"):
            return
        with self._wait_cond:
            if not self._wait_event_seen:
                self._wait_event_seen = True
                self._wait_event_msg = msg
                self._wait_cond.notify_all()

    # ---------------- state access ----------------
    def get_robot_state(self) -> CachedRobotState:
        with self._lock:
            return CachedRobotState(
                tick_id=self._robot_state.tick_id,
                robot_location=self._robot_state.robot_location,
                battery_status=self._robot_state.battery_status,
            )

    def get_packages(self, timeout_sec: float = 2.0):
        if not self.cli_get_packages.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/get_packages service not available")
        req = GetPackages.Request()
        req.all = True
        fut = self.cli_get_packages.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            if time.time() - t0 > timeout_sec:
                raise TimeoutError("/get_packages timed out")
            time.sleep(0.01)
        res = fut.result()
        if res is None:
            raise RuntimeError("/get_packages returned None")
        return res.packages

    def reset_episode(self, num_packages: int = 20, timeout_sec: float = 3.0) -> None:
        if not self.cli_reset.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/reset_episode service not available")
        req = ResetEpisode.Request()
        req.num_packages = int(num_packages)
        fut = self.cli_reset.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            if time.time() - t0 > timeout_sec:
                raise TimeoutError("/reset_episode timed out")
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.ok:
            raise RuntimeError(f"/reset_episode failed: {getattr(res, 'error', 'unknown')}")

    # ---------------- action execution ----------------
    def send_cmd(self, cmd: str, timeout_sec: float = 60.0) -> Tuple[str, float]:
        """Send a non-WAIT command as an action goal and block for result.

        Returns: (result_string, dt)
        """
        if PlannerCmd is None or self._action_client is None:
            raise RuntimeError("PlannerCmd action type not available (interfaces.action.PlannerCmd import failed)")

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("/planner/cmd action server not available")

        goal = PlannerCmd.Goal()
        goal.cmd = str(cmd)

        send_fut = self._action_client.send_goal_async(goal)
        t0 = time.time()
        while rclpy.ok() and not send_fut.done():
            if time.time() - t0 > 2.0:
                raise TimeoutError("send_goal timed out")
            time.sleep(0.01)

        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            return ("FAIL", 0.0)

        res_fut = goal_handle.get_result_async()
        t0 = time.time()
        while rclpy.ok() and not res_fut.done():
            if time.time() - t0 > timeout_sec:
                # Non-interruptable by spec; treat as fail and continue
                return ("FAIL", timeout_sec)
            time.sleep(0.01)

        res = res_fut.result()
        if res is None:
            return ("FAIL", 0.0)
        result_msg = res.result
        return (str(result_msg.result), float(result_msg.dt))

    def wait_for_process_event(self, timeout_sec: float = 600.0) -> WorldEvent:
        """Block until the first PROCESS_STARTED/PROCESS_FINISHED event arrives."""
        with self._wait_cond:
            self._wait_event_seen = False
            self._wait_event_msg = None
            t0 = time.time()
            while rclpy.ok() and not self._wait_event_seen:
                remaining = timeout_sec - (time.time() - t0)
                if remaining <= 0:
                    raise TimeoutError("WAIT interrupted: timeout")
                self._wait_cond.wait(timeout=0.1)
            assert self._wait_event_msg is not None
            return self._wait_event_msg
