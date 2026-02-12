import rclpy
from rclpy.node import Node
from interfaces.srv import ResetEpisode, Enqueue
from interfaces.msg import WorldEvent


class JobSpawner(Node):
    def __init__(self):
        super().__init__("job_spawner")
        self.sub_event = self.create_subscription(
            WorldEvent,
            "/world_event",
            self.on_world_event,
            10
        )

        self.cli_spawn_a = self.create_client(Enqueue, "/station/A/spawn")

        self._started = False
        self._next_idx = 0
        self._spawn_timer = None

        # one-shot kick after startup
        self._start_timer = self.create_timer(1.0, self.start_once)

    def on_world_event(self, msg):
        if msg.event_type == "EPISODE_RESET":
            self._next_idx = 0
            if self._spawn_timer:
                self._spawn_timer.cancel()
            self._spawn_timer = self.create_timer(20.0, self.spawn_one)


    def start_once(self):
        if self._started:
            return
        self._started = True

        # stop this one-shot timer
        if self._start_timer is not None:
            self._start_timer.cancel()
        # The planner controls episode boundaries.
        if not self.cli_spawn_a.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/station/A/spawn not available")
            return
        self.get_logger().info("JobSpawner ready. Waiting for EPISODE_RESET to start spawning.")

    def on_world_event(self, msg: WorldEvent):
        if msg.event_type != "EPISODE_RESET":
            return
        # reset internal spawner state
        self._next_idx = 0
        if self._spawn_timer is not None:
            self._spawn_timer.cancel()
            self._spawn_timer = None

        # spawn first package directly
        self.get_logger().info("EPISODE_RESET received -> Spawning first package immediately.")
        self.spawn_one()

        # spawn every 20 seconds (restart)
        self._spawn_timer = self.create_timer(20.0, self.spawn_one)
        self.get_logger().info("Spawner interval started (next package in 20s)")
    
    def spawn_one(self):
        if self._next_idx >= 20:
            self.get_logger().info("All 20 packages spawned. Stopping spawner timer.")
            if self._spawn_timer is not None:
                self._spawn_timer.cancel()
            return

        req = Enqueue.Request()
        req.package_idx = self._next_idx

        fut = self.cli_spawn_a.call_async(req)
        fut.add_done_callback(lambda f, idx=self._next_idx: self._on_spawn_done(f, idx))

    def _on_spawn_done(self, fut, idx: int):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"spawn({idx}) call exception: {e}")
            return

        if res is None:
            self.get_logger().error(f"spawn({idx}) failed: service returned None")
            return

        if res.result != "ACCEPTED":
            self.get_logger().warn(f"spawn declined for idx={idx}: {res.result}")
            return

        self.get_logger().info(f"Spawned package_idx={idx} at Station A")
        self._next_idx += 1


def main():
    rclpy.init()
    node = JobSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
