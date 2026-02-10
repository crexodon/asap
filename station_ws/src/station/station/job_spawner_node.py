import rclpy
from rclpy.node import Node
from interfaces.srv import ResetEpisode, Enqueue


class JobSpawner(Node):
    def __init__(self):
        super().__init__("job_spawner")

        self.cli_reset = self.create_client(ResetEpisode, "/reset_episode")
        self.cli_spawn_a = self.create_client(Enqueue, "/station/A/spawn")

        self._started = False
        self._next_idx = 0
        self._spawn_timer = None

        # one-shot kick after startup
        self._start_timer = self.create_timer(1.0, self.start_once)

    def start_once(self):
        if self._started:
            return
        self._started = True

        # stop this one-shot timer
        if self._start_timer is not None:
            self._start_timer.cancel()

        if not self.cli_reset.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("reset_episode not available")
            return

        req = ResetEpisode.Request()
        req.num_packages = 20

        fut = self.cli_reset.call_async(req)
        fut.add_done_callback(self._on_reset_done)

    def _on_reset_done(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"reset service call raised exception: {e}")
            return

        if res is None:
            self.get_logger().error("reset failed: service returned None")
            return

        if not res.ok:
            self.get_logger().error(f"reset failed: {res.error}")
            return

        self.get_logger().info("Reset succeeded")

        if not self.cli_spawn_a.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/station/A/spawn not available")
            return

        # spawn every 20 seconds
        self._spawn_timer = self.create_timer(20.0, self.spawn_one)
        self.get_logger().info("Spawner started: spawning 1 package every 20s")

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
