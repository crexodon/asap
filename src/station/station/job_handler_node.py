import random
from rclpy.node import Node
import rclpy
from interfaces.srv import SetPackageInfo, ResetEpisode
from interfaces.msg import PackageState, WorldEvent

VALID_FIELDS = {
    "shipping_type", "current_location", "next_location",
    "lifecycle_state", "availability"
}

class JobHandler(Node):
    def __init__(self):
        super().__init__("job_handler")
        self._tick_id = 0
        self._packages = []
        self._num_packages = 20

        self.srv_set = self.create_service(SetPackageInfo, "/set_package_info", self.on_set)
        self.srv_reset = self.create_service(ResetEpisode, "/reset_episode", self.on_reset)

        self.pub_event = self.create_publisher(WorldEvent, "/world_event", 10)
        self.sub_event = self.create_subscription(WorldEvent, "/world_event", self.on_world_event, 10)


        self._init_packages(self._num_packages)

    def _emit_event(self, event_type: str, source: str, package_idx: int = -1, station_id: str = ""):
        self._tick_id += 1
        msg = WorldEvent()
        msg.tick_id = self._tick_id
        msg.event_type = event_type
        msg.source = source
        msg.package_idx = package_idx
        msg.station_id = station_id
        self.pub_event.publish(msg)

    def _init_packages(self, n: int):
        self._packages = []
        for i in range(n):
            p = PackageState()
            p.package_idx = i
            p.shipping_type = "standard"
            p.current_location = "A"
            p.next_location = "B"
            p.lifecycle_state = "SPAWNED"
            p.availability = False
            self._packages.append(p)

    def on_reset(self, req: ResetEpisode.Request, res: ResetEpisode.Response):
        self._num_packages = int(req.num_packages) if req.num_packages > 0 else 20
        if self._num_packages != 20:
            res.ok = False
            res.error = "This implementation assumes fixed 20 packages."
            return res

        self._init_packages(20)

        # alternation shipping types
        for p in self._packages:
            p.shipping_type = "express" if p.package_idx % 2 == 1 else "standard"
            p.current_location = "A"
            p.next_location = "B"
            p.lifecycle_state = "SPAWNED"
            p.availability = False

        self._emit_event("EPISODE_RESET", "job_handler")
        res.ok = True
        res.error = ""
        return res

    def on_set(self, req: SetPackageInfo.Request, res: SetPackageInfo.Response):
        idx = req.package_idx
        if idx < 0 or idx >= len(self._packages):
            res.ok = False
            res.error = "package_idx out of range"
            return res
        if len(req.fields) != len(req.values):
            res.ok = False
            res.error = "fields/values length mismatch"
            return res

        p = self._packages[idx]
        for f, v in zip(req.fields, req.values):
            if f not in VALID_FIELDS:
                res.ok = False
                res.error = f"invalid field: {f}"
                return res
            if f == "availability":
                p.availability = (v.lower() == "true")
            else:
                setattr(p, f, v)

        self._emit_event("PACKAGE_UPDATED", "job_handler", package_idx=idx)
        res.ok = True
        res.error = ""
        return res
    
    def on_world_event(self, msg: WorldEvent):
        # We only care about Station C finishing successfully.
        # Station C only emits PROCESS_FINISHED when it really finished and put pkg into output.
        if msg.event_type != "PROCESS_FINISHED":
            return
        if msg.station_id != "C":
            return
        idx = int(msg.package_idx)
        if idx < 0 or idx >= len(self._packages):
            return

        p = self._packages[idx]

        # Decide routing based on shipping_type (SoT)
        if p.shipping_type == "express":
            p.next_location = "E"
        else:
            p.next_location = "D"

        # Emit a package updated event so the aggregator/MDP gets a new decision point
        self._emit_event("PACKAGE_ROUTED", "job_handler", package_idx=idx, station_id="C")


def main():
    rclpy.init()
    node = JobHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
