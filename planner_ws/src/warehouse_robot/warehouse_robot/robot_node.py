import math
import time
import random
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from interfaces.msg import RobotState, WorldEvent
from interfaces.srv import Enqueue, Dequeue, ResetEpisode
from interfaces.action import Pick, Charge, PlannerCmd

from action_msgs.msg import GoalStatus




STATION_COORDS: Dict[str, Tuple[float, float]] = {
    "A": (5.0, 15.0),
    "B": (9.0, 15.0),
    "C": (13.0, 15.0),
    "D": (17.0, 15.0),
    "E": (13.0, 1.0),
    "F": (9.0, 1.0),
    "G": (5.0, 1.0),
}
START_COORDS: Tuple[float, float] = (1.0, 13.0)  # S (not an external station id)


class WarehouseRobotNode(Node):
    """Discrete training robot (no Gazebo).

    Implements /planner/cmd as an Action server.
    Publishes /robot_state with TRANSIENT_LOCAL durability (latched-like).

    NOTE on 'S':
      External systems in this project use 'ON_TRANSIT' instead of an explicit 'S' station.
      This node keeps internal pose at START_COORDS and publishes robot_location as the
      parameter 'publish_location_S_as' (default: 'ON_TRANSIT').
    """

    def __init__(self):
        super().__init__("robot_node")
        self.cb = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("seed", 0)
        self.declare_parameter("move_speed_mps", 2.0)          # max speed
        self.declare_parameter("move_factor", 1.5)             # slow-down factor
        self.declare_parameter("move_rand_min", 0.0)           # seconds
        self.declare_parameter("move_rand_max", 0.5)           # seconds
        self.declare_parameter("battery_drain_per_s", 1.0)     # battery units per second
        self.declare_parameter("pick_drop_dt", 0.1)            # dt for enqueue/dequeue (service has no dt)
        self.declare_parameter("robot_state_rate_hz", 10.0)    # periodic publish
        self.declare_parameter("publish_location_S_as", "ON_TRANSIT")  # 'ON_TRANSIT' (recommended) or 'S'

        self.rng = random.Random(int(self.get_parameter("seed").value))

        self.move_speed = float(self.get_parameter("move_speed_mps").value)
        self.move_factor = float(self.get_parameter("move_factor").value)
        self.move_rand_min = float(self.get_parameter("move_rand_min").value)
        self.move_rand_max = float(self.get_parameter("move_rand_max").value)
        self.battery_drain_per_s = float(self.get_parameter("battery_drain_per_s").value)
        self.pick_drop_dt = float(self.get_parameter("pick_drop_dt").value)
        self.state_rate = float(self.get_parameter("robot_state_rate_hz").value)
        self.publish_location_S_as = str(self.get_parameter("publish_location_S_as").value)

        # Internal state
        self._tick_id = 0
        self._pose: Tuple[float, float] = START_COORDS
        self.robot_location: str = self.publish_location_S_as
        self.carrying_idx: int = -1
        self.battery_status: float = 100.0
        self.robot_mode: str = "IDLE"

        # Latched-like /robot_state publisher (Transient Local)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_state = self.create_publisher(RobotState, "/robot_state", qos)

        # World events (for reset)
        self.sub_event = self.create_subscription(WorldEvent, "/world_event", self.on_world_event, 10)

        # Station service clients
        self.cli_enqueue: Dict[str, rclpy.client.Client] = {}
        self.cli_dequeue: Dict[str, rclpy.client.Client] = {}
        for sid in STATION_COORDS.keys():
            self.cli_enqueue[sid] = self.create_client(
                Enqueue, f"/robot_action/{sid}/enqueue", callback_group=self.cb
            )
            self.cli_dequeue[sid] = self.create_client(
                Dequeue, f"/robot_action/{sid}/dequeue", callback_group=self.cb
            )

        # Station action clients
        self.ac_pick_a = ActionClient(self, Pick, "/robot_action/Station_A/pick", callback_group=self.cb)
        self.ac_charge = ActionClient(self, Charge, "/robot_action/Station_F/charge", callback_group=self.cb)

        # Reset service client exists on job_handler (not used directly, but kept for completeness)
        self.cli_reset_episode = self.create_client(ResetEpisode, "/reset_episode", callback_group=self.cb)

        # /planner/cmd action server
        self.as_cmd = ActionServer(
            self,
            PlannerCmd,
            "/planner/cmd",
            execute_callback=self.execute_cmd,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )

        # Periodic /robot_state publish
        self.timer = self.create_timer(1.0 / max(self.state_rate, 1.0), self.publish_state, callback_group=self.cb)

        self.publish_state()

    def _next_tick(self) -> int:
        self._tick_id += 1
        return self._tick_id

    def publish_state(self):
        msg = RobotState()
        msg.tick_id = self._next_tick()
        msg.robot_location = self.robot_location
        msg.carrying_idx = int(self.carrying_idx)
        msg.battery = float(self.battery_status)
        if hasattr(msg, "robot_mode"):
            msg.robot_mode = str(self.robot_mode)
        self.pub_state.publish(msg)

    def on_world_event(self, msg: WorldEvent):
        if msg.event_type == "EPISODE_RESET":
            self.reset_local_state()

    def reset_local_state(self):
        self.get_logger().info("Robot reset (local) due to EPISODE_RESET")
        self._pose = START_COORDS
        self.robot_location = self.publish_location_S_as
        self.carrying_idx = -1
        self.battery_status = 100.0
        self.robot_mode = "IDLE"
        self.publish_state()

    def _drain_battery(self, dt: float):
        if dt <= 0.0:
            return
        self.battery_status -= self.battery_drain_per_s * dt
        if self.battery_status < 0.0:
            self.battery_status = 0.0

    def on_goal(self, goal_request: PlannerCmd.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _compute_move_time(self, src: Tuple[float, float], dst: Tuple[float, float]) -> float:
        dx = dst[0] - src[0]
        dy = dst[1] - src[1]
        dist = math.sqrt(dx * dx + dy * dy)
        base = (dist / max(self.move_speed, 1e-6)) * self.move_factor
        rnd = self.rng.uniform(self.move_rand_min, self.move_rand_max)
        return float(base + rnd)

    def _wait_for_service(self, client, name: str, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while not client.service_is_ready():
            if time.time() - t0 > timeout_s:
                self.get_logger().error(f"Service not ready: {name}")
                return False
            time.sleep(0.05)
        return True
    
    def _wait_future(self, fut, timeout_s: float, name: str) -> bool:
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            if time.time() - t0 > timeout_s:
                self.get_logger().error(f"Timeout waiting for {name}")
                return False
            time.sleep(0.01)
        return fut.done()


    def _call_enqueue(self, station_id: str, package_idx: int) -> Enqueue.Response:
        cli = self.cli_enqueue[station_id]
        if not self._wait_for_service(cli, f"/robot_action/{station_id}/enqueue"):
            raise RuntimeError("enqueue service not ready")
        req = Enqueue.Request()
        req.package_idx = int(package_idx)
        fut = cli.call_async(req)
        if not self._wait_future(fut, 5.0, f"enqueue({station_id})"):
            raise RuntimeError("enqueue service timeout")
        return fut.result()


    def _call_dequeue(self, station_id: str, package_idx: int) -> Dequeue.Response:
        cli = self.cli_dequeue[station_id]
        if not self._wait_for_service(cli, f"/robot_action/{station_id}/dequeue"):
            raise RuntimeError("dequeue service not ready")
        req = Dequeue.Request()
        req.package_idx = int(package_idx)
        fut = cli.call_async(req)
        if not self._wait_future(fut, 5.0, f"enqueue({station_id})"):
            raise RuntimeError("enqueue service timeout")
        return fut.result()
    
    def _do_pick_a(self, package_idx: int) -> Tuple[str, int, float]:
        if not self.ac_pick_a.wait_for_server(timeout_sec=5.0):
            return "FAIL no pick_a server", -1, 0.0

        goal = Pick.Goal()
        goal.package_idx = int(package_idx)

        t0 = time.time()

        goal_fut = self.ac_pick_a.send_goal_async(goal)
        if not self._wait_future(goal_fut, 5.0, "pick_a send_goal"):
            return "FAIL send_goal timeout", -1, 0.0
        goal_handle = goal_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            return "FAIL not accepted", -1, 0.0

        res_fut = goal_handle.get_result_async()
        if not self._wait_future(res_fut, 60.0, "pick_a result"):
            return "FAIL result timeout", -1, 0.0

        res = res_fut.result()
        status_ok = (res.status == GoalStatus.STATUS_SUCCEEDED)
        result_msg = str(res.result.result).strip()
        picked = int(res.result.package_idx)

        dt = float(time.time() - t0)

        # Gib beides zurück
        return status_ok, result_msg, picked, dt


    def _do_charge(self) -> Tuple[str, float, float]:
        if not self.ac_charge.server_is_ready():
            self.ac_charge.wait_for_server(timeout_sec=5.0)

        goal = Charge.Goal()
        goal.current_battery_status = float(self.battery_status)

        goal_fut = self.ac_charge.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_fut)
        goal_handle = goal_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            return "FAIL", float(self.battery_status), 0.0

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        dt = float(getattr(res.result, "dt", 0.0))
        return str(res.result.result), float(res.result.battery), dt

    def execute_cmd(self, goal_handle):
        cmd = str(goal_handle.request.cmd).strip()
        self.get_logger().info(f"/planner/cmd: '{cmd}'")

        fb = PlannerCmd.Feedback()

        # WAIT: run until canceled
        if cmd.upper() == "WAIT":
            self.robot_mode = "WAITING_ACTION"
            self.publish_state()
            t0 = time.time()
            rate_dt = 1.0 / max(self.state_rate, 1.0)
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    dt = float(time.time() - t0)
                    self._drain_battery(dt)
                    goal_handle.canceled()
                    self.robot_mode = "IDLE"
                    self.publish_state()
                    res = PlannerCmd.Result()
                    res.result = "OK"
                    res.dt = dt
                    return res
                fb.progress = 0.0
                goal_handle.publish_feedback(fb)
                time.sleep(rate_dt)

        parts = cmd.split()
        if not parts:
            goal_handle.abort()
            res = PlannerCmd.Result()
            res.result = "FAIL"
            res.dt = 0.0
            return res

        verb = parts[0].upper()

        # MOVE_TO X
        if verb == "MOVE_TO":
            if len(parts) != 2 or parts[1] not in STATION_COORDS:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL"
                res.dt = 0.0
                return res

            target = parts[1]
            dst = STATION_COORDS[target]
            src = self._pose
            total = self._compute_move_time(src, dst)

            self.robot_mode = "BUSY"
            self.robot_location = "ON_TRANSIT"
            self.publish_state()

            t0 = time.time()
            step = 0.1
            while True:
                elapsed = time.time() - t0
                if elapsed >= total:
                    break
                fb.progress = float(elapsed / total) if total > 0 else 1.0
                goal_handle.publish_feedback(fb)
                time.sleep(step)

            dt = float(total)
            self._drain_battery(dt)

            self._pose = dst
            self.robot_location = target
            self.robot_mode = "IDLE"
            self.publish_state()

            goal_handle.succeed()
            res = PlannerCmd.Result()
            res.result = "OK"
            res.dt = dt
            return res

        # PICK k
        if verb == "PICK":
            if len(parts) != 2:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL"
                res.dt = 0.0
                return res
            try:
                pkg = int(parts[1])
            except ValueError:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL"
                res.dt = 0.0
                return res

            station = self.robot_location
            if station not in STATION_COORDS:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL"
                res.dt = 0.0
                return res

            self.robot_mode = "BUSY"
            self.publish_state()

            try:
                resp = self._call_dequeue(station, pkg)
                ok = str(resp.result) == "ACCEPTED"
                if ok:
                    self.carrying_idx = int(resp.actual_package_idx)
            except Exception as e:
                self.get_logger().error(f"PICK failed: {e}")
                ok = False

            dt = float(self.pick_drop_dt)
            self._drain_battery(dt)

            self.robot_mode = "IDLE"
            self.publish_state()

            (goal_handle.succeed() if ok else goal_handle.abort())
            res = PlannerCmd.Result()
            res.result = "OK" if ok else "FAIL"
            res.dt = dt
            return res

        # DROP k
        if verb == "DROP":
            if len(parts) != 2:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL no package_idx given"
                res.dt = 0.0
                return res
            try:
                pkg = int(parts[1])
            except ValueError:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL package_idx no integer"
                res.dt = 0.0
                return res

            station = self.robot_location
            if station not in STATION_COORDS:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL location not valid"
                res.dt = 0.0
                return res

            self.robot_mode = "BUSY"
            self.publish_state()

            try:
                resp = self._call_enqueue(station, pkg)
                ok = str(resp.result) == "ACCEPTED"
                if ok:
                    self.carrying_idx = -1
            except Exception as e:
                self.get_logger().error(f"DROP failed: {e}")
                ok = False

            dt = float(self.pick_drop_dt)
            self._drain_battery(dt)

            self.robot_mode = "IDLE"
            self.publish_state()

            (goal_handle.succeed() if ok else goal_handle.abort())
            res = PlannerCmd.Result()
            res.result = "OK" if ok else "FAIL"
            res.dt = dt
            self.get_logger().info(f"DROP result raw result_str={ok} picked_pkg={picked_pkg} dt={dt}")
            return res

        # PICK_A k
        if verb == "PICK_A":
            if len(parts) != 2:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL no package_idx"
                res.dt = 0.0
                return res
            try:
                pkg = int(parts[1])
            except ValueError:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL package_idx not readable"
                res.dt = 0.0
                return res

            self.robot_mode = "BUSY"
            self.publish_state()

            status_ok, result_str, picked_pkg, dt = self._do_pick_a(pkg)
            ok = status_ok
            if ok:
                self.carrying_idx = int(picked_pkg)
            self._drain_battery(dt)

            self.robot_mode = "IDLE"
            self.publish_state()

            (goal_handle.succeed() if ok else goal_handle.abort())
            res = PlannerCmd.Result()
            res.result = "OK" if ok else "FAIL picking action failed"
            res.dt = float(dt)

            self.get_logger().info(f"PICK_A raw result_str={result_str} picked_pkg={picked_pkg} dt={dt}")

            return res

        # CHARGE
        if verb == "CHARGE":
            self.robot_mode = "BUSY"
            self.publish_state()

            result_str, new_batt, dt = self._do_charge()
            ok = (result_str == "SUCCEEDED")
            if ok:
                self.battery_status = float(new_batt)
            self.robot_mode = "IDLE"
            self.publish_state()

            (goal_handle.succeed() if ok else goal_handle.abort())
            res = PlannerCmd.Result()
            res.result = "OK" if ok else "FAIL"
            res.dt = float(dt)
            return res

        # Unknown command
        goal_handle.abort()
        res = PlannerCmd.Result()
        res.result = "FAIL"
        res.dt = 0.0
        return res


def main():
    rclpy.init()
    node = WarehouseRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
