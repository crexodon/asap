import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.srv import Enqueue, Dequeue, SetPackageInfo
from interfaces.action import Pick, Charge
from interfaces.msg import StationState, WorldEvent, RobotState

STATIONS = ["A","B","C","D","E","F","G"]

class StationNode(Node):
    def __init__(self):
        super().__init__("station")
        self.declare_parameter("station_id", "B")
        self.declare_parameter("seed", 0)
        self.declare_parameter("process_dt", 0.2)

        self.station_id = self.get_parameter("station_id").get_parameter_value().string_value
        self.rng = random.Random(self.get_parameter("seed").get_parameter_value().integer_value)
        self.process_dt = float(self.get_parameter("process_dt").get_parameter_value().double_value)

        if self.station_id not in STATIONS:
            raise RuntimeError("Invalid station_id")

        self._tick_id = 0
        self.robot_location = "A"
        self.input_queue = []
        self.output_queue = []
        self.output_success_queue = []
        self.output_failed_queue = []
        self.processing_idx = -1
        self.processing_state = "IDLE"
        self._remaining = 0.0

        self.sub_robot = self.create_subscription(RobotState, "/robot_state", self.on_robot_state, 10)
        self.pub_event = self.create_publisher(WorldEvent, "/world_event", 10)

        # Service clients
        self.cli_set_pkg = self.create_client(SetPackageInfo, "/set_package_info")

        # Services for robot interaction
        self.srv_enqueue = self.create_service(Enqueue, f"/robot_action/{self.station_id}/enqueue", self.on_enqueue)
        self.srv_dequeue = self.create_service(Dequeue, f"/robot_action/{self.station_id}/dequeue", self.on_dequeue)

        # Actions (only for A and F)
        self.pick_server = None
        self.charge_server = None
        if self.station_id == "A":
            self.pick_server = ActionServer(self, Pick, "/robot_action/Station_A/pick", self.execute_pick)
        if self.station_id == "F":
            self.charge_server = ActionServer(self, Charge, "/robot_action/Station_F/charge", self.execute_charge)

        # Autonomy loop for processing stations (B,C,D,E,G)
        if self.station_id in ["B","C","D","E","G"]:
            self.timer = self.create_timer(self.process_dt, self.processing_step)

        self.emit_event("STATION_READY")

    def on_robot_state(self, msg: RobotState):
        self.robot_location = msg.robot_location

    def emit_event(self, event_type: str, package_idx: int = -1):
        self._tick_id += 1
        ev = WorldEvent()
        ev.tick_id = self._tick_id
        ev.event_type = event_type
        ev.source = f"station_{self.station_id}"
        ev.package_idx = package_idx
        ev.station_id = self.station_id
        self.pub_event.publish(ev)

    def robot_present(self) -> bool:
        return self.robot_location == self.station_id

    def set_pkg(self, package_idx: int, fields, values):
        # sync call pattern (simple)
        if not self.cli_set_pkg.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_package_info service not available")
            return False
        req = SetPackageInfo.Request()
        req.package_idx = int(package_idx)
        req.fields = list(fields)
        req.values = [str(v) for v in values]
        fut = self.cli_set_pkg.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().error("set_package_info call failed")
            return False
        return fut.result().ok

    # ---------------- Services ----------------
    def on_enqueue(self, req: Enqueue.Request, res: Enqueue.Response):
        # Robot drops carried package into station input
        if not self.robot_present():
            res.result = "DECLINED_NOT_PRESENT"
            res.position = -1
            res.actual_package_idx = -1
            return res

        pkg = int(req.package_idx)
        self.input_queue.append(pkg)
        res.result = "ACCEPTED"
        res.position = len(self.input_queue) - 1
        res.actual_package_idx = pkg

        # package is now at station input
        self.set_pkg(pkg, ["current_location","lifecycle_state","availability"], [self.station_id, "WAITING", "false"])
        self.emit_event("QUEUE_CHANGED", package_idx=pkg)
        return res

    def on_dequeue(self, req: Dequeue.Request, res: Dequeue.Response):
        if not self.robot_present():
            res.result = "DECLINED_NOT_PRESENT"
            res.position = -1
            res.actual_package_idx = -1
            return res
        if len(self.output_queue) == 0:
            res.result = "DECLINED_QUEUE_EMPTY"
            res.position = -1
            res.actual_package_idx = -1
            return res

        wanted = int(req.package_idx)
        if wanted == -1:
            pkg = self.output_queue.pop(0)
        else:
            if wanted not in self.output_queue:
                res.result = "DECLINED_NOT_AVAILABLE"
                res.position = -1
                res.actual_package_idx = -1
                return res
            self.output_queue.remove(wanted)
            pkg = wanted

        res.result = "ACCEPTED"
        res.position = -1
        res.actual_package_idx = pkg

        # picked up by robot
        self.set_pkg(pkg, ["current_location","availability"], ["ROBOT","false"])
        self.emit_event("QUEUE_CHANGED", package_idx=pkg)
        return res

    # ---------------- Actions ----------------
    async def execute_pick(self, goal_handle):
        # A-specific: pick requires duration and robot presence
        if self.station_id != "A":
            goal_handle.abort()
            result = Pick.Result()
            result.result = "ABORTED_WRONG_STATION"
            result.package_idx = -1
            return result

        if not self.robot_present():
            goal_handle.abort()
            result = Pick.Result()
            result.result = "ABORTED_NOT_PRESENT"
            result.package_idx = -1
            return result

        # In this simplified model: spawner puts items into A.input, and A immediately makes them ready in output.
        # We'll pick from A.output_queue
        wanted = int(goal_handle.request.package_idx)
        if len(self.output_queue) == 0:
            goal_handle.abort()
            result = Pick.Result()
            result.result = "ABORTED_NOT_AVAILABLE"
            result.package_idx = -1
            return result

        if wanted == -1:
            pkg = self.output_queue[0]
        else:
            if wanted not in self.output_queue:
                goal_handle.abort()
                result = Pick.Result()
                result.result = "ABORTED_NOT_AVAILABLE"
                result.package_idx = -1
                return result
            pkg = wanted

        # simulate duration that requires robot presence
        duration = 1.0 + self.rng.random() * 1.0  # U(1.0,2.0)
        elapsed = 0.0
        while elapsed < duration:
            if not self.robot_present():
                # abort and keep pkg in output
                goal_handle.abort()
                result = Pick.Result()
                result.result = "ABORTED_LEFT_STATION"
                result.package_idx = pkg
                self.emit_event("ACTION_RESULT", package_idx=pkg)
                return result

            fb = Pick.Feedback()
            fb.progress = float(elapsed / duration)
            goal_handle.publish_feedback(fb)
            await rclpy.task.Future()  # placeholder; replaced below

        # This async approach without asyncio is awkward in rclpy.
        # We'll implement a simple blocking loop with rate instead:
        goal_handle.execute()

        # blocking part
        rate = self.create_rate(20)
        elapsed = 0.0
        while elapsed < duration:
            if not self.robot_present():
                goal_handle.abort()
                result = Pick.Result()
                result.result = "ABORTED_LEFT_STATION"
                result.package_idx = pkg
                self.emit_event("ACTION_RESULT", package_idx=pkg)
                return result
            fb = Pick.Feedback()
            fb.progress = float(elapsed / duration)
            goal_handle.publish_feedback(fb)
            rate.sleep()
            elapsed += 0.05

        # commit: remove from output_queue
        if pkg in self.output_queue:
            self.output_queue.remove(pkg)

        # update package as on robot and next_location=B
        self.set_pkg(pkg, ["current_location","next_location","lifecycle_state","availability"], ["ROBOT","B","READY","false"])
        goal_handle.succeed()
        result = Pick.Result()
        result.result = "SUCCEEDED"
        result.package_idx = pkg
        self.emit_event("ACTION_RESULT", package_idx=pkg)
        return result

    async def execute_charge(self, goal_handle):
        if self.station_id != "F":
            goal_handle.abort()
            result = Charge.Result()
            result.result = "ABORTED_WRONG_STATION"
            result.battery = 0.0
            return result
        if not self.robot_present():
            goal_handle.abort()
            result = Charge.Result()
            result.result = "ABORTED_NOT_PRESENT"
            result.battery = 0.0
            return result

        # Battery model placeholder: pretend battery increases over time; RobotInterface owns real battery later.
        target = float(goal_handle.request.target_battery)
        b = 0.5
        rate = self.create_rate(10)
        while b < target:
            if not self.robot_present():
                goal_handle.abort()
                result = Charge.Result()
                result.result = "ABORTED_LEFT_STATION"
                result.battery = float(b)
                self.emit_event("ACTION_RESULT")
                return result
            b = min(target, b + 0.02)
            fb = Charge.Feedback()
            fb.battery = float(b)
            goal_handle.publish_feedback(fb)
            rate.sleep()

        goal_handle.succeed()
        result = Charge.Result()
        result.result = "SUCCEEDED"
        result.battery = float(b)
        self.emit_event("ACTION_RESULT")
        return result

    # ---------------- Processing loop (B,C,D,E,G) ----------------
    def processing_step(self):
        if self.processing_state == "IDLE":
            if len(self.input_queue) == 0:
                return
            self.processing_idx = self.input_queue.pop(0)
            self.processing_state = "RUNNING"

            if self.station_id == "B":
                self._remaining = 1.5 + self.rng.random() * 1.0  # U(1.5; 2.5)
            elif self.station_id == "C":
                self._remaining = 1.5 + self.rng.random() * 1.0  # U(1.5; 2.5)
            elif self.station_id == "D":
                self._remaining = 1.0 + self.rng.random() * 1.0  # U(1.0; 2.0)
            elif self.station_id == "E":
                self._remaining = 1.0 + self.rng.random() * 1.0  # U(1.0; 2.0)
            elif self.station_id == "G":
                self._remaining = 2.0 + self.rng.random() * 2.0  # U(2.0; 4.0)
            self.set_pkg(self.processing_idx, ["lifecycle_state","availability"], ["PROCESSING","false"])
            self.emit_event("PROCESS_STARTED", package_idx=self.processing_idx)
            return

        # RUNNING
        self._remaining -= self.process_dt
        if self._remaining > 0.0:
            return

        pkg = self.processing_idx
        self.processing_idx = -1
        self.processing_state = "IDLE"

        # success/failure logic
        if self.station_id == "B":
            success = (self.rng.random() < 0.80) # Success Prob 0.8
        elif self.station_id == "C":
            success = (self.rng.random() < 0.80) # Success Prob 0.95
        elif self.station_id == "D":
            success = (self.rng.random() < 0.80) # Success Prob 0.85
        elif self.station_id == "E":
            success = (self.rng.random() < 0.80) # Success Prob 0.85
        elif self.station_id == "G":
            success = True # Success Prob 1.0

        # routing logic per station
        if self.station_id == "B":
            next_loc = "C" if success else "G"
            self.output_queue.append(pkg)
            self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], [next_loc, "READY", "true"])

        elif self.station_id == "C":
            # Station C does NOT decide next_location.
            # It only finishes processing and makes the package ready in output.
            if success:
                self.output_queue.append(pkg)
                # Do NOT set next_location here!
                self.set_pkg(pkg, ["lifecycle_state", "availability"], ["READY", "true"])
            else:
                # repeat: put back to input
                self.input_queue.append(pkg)
                self.set_pkg(pkg, ["lifecycle_state", "availability"], ["WAITING", "false"])


        elif self.station_id == "G":
            next_loc = "C"
            self.output_queue.append(pkg)
            self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], [next_loc, "READY", "true"])

        elif self.station_id == "D":
            if success:
                self.output_success_queue.append(pkg)
                self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], ["FINISH","FINISHED","false"])
            else:
                # failed at D: send to E but mark FAILED
                self.output_queue.append(pkg)
                self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], ["E","FAILED","true"])

        elif self.station_id == "E":
            # if already FAILED => put into failed queue, else process
            if success:
                self.output_success_queue.append(pkg)
                self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], ["FINISH","FINISHED","false"])
            else:
                self.output_failed_queue.append(pkg)
                self.set_pkg(pkg, ["next_location","lifecycle_state","availability"], ["FINISH","FAILED","false"])

        self.emit_event("PROCESS_FINISHED", package_idx=pkg)
        self.emit_event("QUEUE_CHANGED", package_idx=pkg)

def main():
    rclpy.init()
    node = StationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
