import rclpy
from rclpy.node import Node
from interfaces.srv import ResetEpisode, Enqueue

class JobSpawner(Node):
    def __init__(self):
        super().__init__("job_spawner")

        self.cli_reset = self.create_client(ResetEpisode, "/reset_episode")
        self.cli_enqueue_a = self.create_client(Enqueue, "/robot_action/A/enqueue")

        self.create_timer(1.0, self.start_once)
        self._started = False

    def start_once(self):
        if self._started:
            return
        self._started = True

        if not self.cli_reset.wait_for_service(2.0):
            self.get_logger().error("reset_episode not available")
            return
        req = ResetEpisode.Request()
        req.num_packages = 20
        fut = self.cli_reset.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is None or not fut.result().ok:
            self.get_logger().error(f"reset failed: {getattr(fut.result(),'error','?')}")
            return

        if not self.cli_enqueue_a.wait_for_service(2.0):
            self.get_logger().error("A enqueue not available")
            return

        # spawn 20 packages into A input
        for i in range(20):
            er = Enqueue.Request()
            er.package_idx = i
            ef = self.cli_enqueue_a.call_async(er)
            rclpy.spin_until_future_complete(self, ef, timeout_sec=1.0)

        self.get_logger().info("Spawned 20 packages to Station A")

def main():
    rclpy.init()
    node = JobSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
