import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.srv import Enqueue, Dequeue, SetPackageInfo, GetPackages
from interfaces.action import Pick, Charge
from interfaces.msg import WorldEvent, RobotState
import time
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


STATIONS = ["A","B","C","D","E","F","G"]

class StationNode(Node):
    def __init__(self):
        super().__init__("station")
        self.cb_group_srv = MutuallyExclusiveCallbackGroup()
        self.cb_group_cli = ReentrantCallbackGroup()

        self.declare_parameter("station_id", "B")
        self.declare_parameter("seed", 0)
        self.declare_parameter("process_dt", 0.2)

        self.station_id = self.get_parameter("station_id").get_parameter_value().string_value
        self.rng = random.Random(self.get_parameter("seed").get_parameter_value().integer_value)
        self.process_dt = float(self.get_parameter("process_dt").get_parameter_value().double_value)

        self.cli_get_pkgs = self.create_client(GetPackages, "/get_packages", callback_group=self.cb_group_cli)
        self.cli_set_pkg = self.create_client(SetPackageInfo, "/set_package_info", callback_group=self.cb_group_cli)

        if self.station_id not in STATIONS:
            raise RuntimeError("Invalid station_id")

        self._tick_id = 0
        self.robot_location = "ON_TRANSIT"
        self.input_queue = []
        self.output_queue = []
        self.output_success_queue = []
        self.output_failed_queue = []
        self.processing_idx = -1
        self.processing_state = "IDLE"
        self._remaining = 0.0

        self.sub_robot = self.create_subscription(RobotState, "/robot_state", self.on_robot_state, 10)
        self.pub_event = self.create_publisher(WorldEvent, "/world_event", 10)


        # Services for robot interaction
        self.srv_enqueue = self.create_service(
            Enqueue, f"/robot_action/{self.station_id}/enqueue", self.on_enqueue, callback_group=self.cb_group_srv
        )
        self.srv_dequeue = self.create_service(
            Dequeue, f"/robot_action/{self.station_id}/dequeue", self.on_dequeue, callback_group=self.cb_group_srv
        )

        # Service for Job spawn
        # Spawn service (only Station A) - NO robot-present gate
        self.srv_spawn = None
        if self.station_id == "A":
            self.srv_spawn = self.create_service(
                Enqueue, "/station/A/spawn", self.on_spawn, callback_group=self.cb_group_srv
            )


        # Actions (only for A and F)
        self.pick_server = None
        self.charge_server = None
        if self.station_id == "A":
            self.pick_server = ActionServer(
                self, Pick, "/robot_action/Station_A/pick", self.execute_pick, callback_group=self.cb_group_srv
            )
        if self.station_id == "F":
            self.charge_server = ActionServer(
                self, Charge, "/robot_action/Station_F/charge", self.execute_charge, callback_group=self.cb_group_srv
            )


        # Autonomy loop for processing stations (B,C,D,E,G)
        if self.station_id in ["B","C","D","E","G"]:
            self.timer = self.create_timer(self.process_dt, self.processing_step, callback_group=self.cb_group_srv)

        self.emit_event("STATION_READY")

    def on_robot_state(self, msg: RobotState):
        self.emit_event("ROBOT_CHANGED_LOCATION")
        self.robot_location = msg.robot_location
    
    def on_spawn(self, req: Enqueue.Request, res: Enqueue.Response):
        # NO robot_present check!
        pkg = int(req.package_idx)

        # In your concept: spawned job should be immediately available for pickup at A
        self.output_queue.append(pkg)

        res.result = "ACCEPTED"
        res.position = len(self.output_queue) - 1
        res.actual_package_idx = pkg

        # Update SoT: package is at A and READY for pickup
        self.set_pkg(pkg,
                    ["current_location", "lifecycle_state", "availability", "next_location"],
                    ["A", "READY", "true", "B"])

        self.emit_event("PACKAGE_SPAWNED", package_idx=pkg)
        return res


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
        if not self.cli_set_pkg.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_package_info service not available")
            return False

        req = SetPackageInfo.Request()
        req.package_idx = int(package_idx)
        req.fields = list(fields)
        req.values = [str(v) for v in values]

        fut = self.cli_set_pkg.call_async(req)

        # Wait without nested spinning
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 2.0:
                self.get_logger().error("set_package_info call timed out (future not done)")
                return False
            time.sleep(0.01)

        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"set_package_info call raised exception: {e}")
            return False

        if res is None:
            self.get_logger().error("set_package_info returned None")
            return False

        if not res.ok:
            self.get_logger().error(f"set_package_info rejected: {res.error}")
            return False

        return True


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
        if self.station_id == "E":
            # DO NOT overwrite lifecycle_state, because FAILED must remain FAILED for bypass logic
            self.set_pkg(pkg, ["current_location", "availability"], [self.station_id, "false"])
        else:
            self.set_pkg(pkg, ["current_location","lifecycle_state","availability"], [self.station_id, "WAITING", "false"])

        self.emit_event("ENQUEUED", package_idx=pkg)
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
        self.emit_event("DEQUEUED", package_idx=pkg)
        return res
    
    def get_pkg_lifecycle(self, package_idx: int) -> str | None:
        if not self.cli_get_pkgs.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("get_packages service not available")
            return None

        req = GetPackages.Request()
        req.all = True

        fut = self.cli_get_pkgs.call_async(req)

        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 2.0:
                self.get_logger().error("get_packages call timed out (future not done)")
                return None
            time.sleep(0.01)

        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"get_packages call raised exception: {e}")
            return None

        if res is None:
            self.get_logger().error("get_packages returned None")
            return None

        for p in res.packages:
            if int(p.package_idx) == int(package_idx):
                return str(p.lifecycle_state)

        return None



    # ---------------- Actions ----------------
    def execute_pick(self, goal_handle):
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

        # Retry loop: repeat same package until success, while robot stays present
        attempt = 0
        while True:
            attempt += 1

            # simulate duration (hidden)
            duration = 1.0 + self.rng.random() * 1.0  # U(1.0,2.0)
            t0 = time.time()

            while (time.time() - t0) < duration:
                if not self.robot_present():
                    goal_handle.abort()
                    result = Pick.Result()
                    result.result = "ABORTED_LEFT_STATION"
                    result.package_idx = pkg
                    self.emit_event("PICK_A_FAILED", package_idx=pkg)
                    return result

                elapsed = time.time() - t0
                fb = Pick.Feedback()
                fb.progress = float(min(1.0, elapsed / duration))
                goal_handle.publish_feedback(fb)
                time.sleep(0.05)

            # success probability 0.95
            success = (self.rng.random() < 0.95)
            if success:
                break
            # else: failed -> repeat same package again (no state change necessary)

        # commit: remove from output_queue
        if pkg in self.output_queue:
            self.output_queue.remove(pkg)

        # update package as on robot and next_location=B
        self.set_pkg(pkg, ["current_location", "next_location", "lifecycle_state", "availability"],
                    ["ROBOT", "B", "READY", "false"])

        goal_handle.succeed()
        result = Pick.Result()
        result.result = f"SUCCEEDED with {attempt} tries"
        result.package_idx = pkg
        self.emit_event("PICK_A_SUCCESS", package_idx=pkg)
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
                self.emit_event("CHARGE_ABORTED")
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
        self.emit_event("CHARGE_FINISHED")
        return result

    # ---------------- Processing loop (B,C,D,E,G) ----------------
    def processing_step(self):
        if self.processing_state == "IDLE":
            if len(self.input_queue) == 0:
                return
            self.processing_idx = self.input_queue.pop(0)
            if self.station_id == "E":
                lifecycle = self.get_pkg_lifecycle(self.processing_idx)
                if lifecycle == "FAILED":
                    # bypass: do not process, directly into failed output queue
                    pkg = self.processing_idx
                    self.processing_idx = -1
                    self.processing_state = "IDLE"

                    self.output_failed_queue.append(pkg)
                    # terminal: keep FAILED and send to FINISH
                    self.set_pkg(pkg, ["next_location", "availability"], ["FINISH", "false"])

                    self.emit_event("PUT_FAILED_TO_STORE", package_idx=pkg)
                    return

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

def main():
    rclpy.init()
    node = StationNode()

    # Allow concurrent processing so service-client responses can be handled
    # while we are inside another callback (e.g., on_spawn/processing_step).
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
