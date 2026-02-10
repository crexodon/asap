from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import Event, WorldState, json_loads, now_wall


def default_packages(n: int = 20) -> List[Dict[str, Any]]:
    # Minimal defaults; adapt to your real package fields.
    return [
        {
            "location": "A",              # station or "ROBOT" etc.
            "availability": False,        # pickable now?
            "next_station": "A",
            "shipping_type": "STD",       # STD/EXP
            "lifecycle_state": "NEW",     # NEW/IN_PROCESS/READY/DONE/FAILED...
        }
        for _ in range(n)
    ]


class WorldStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__("world_state_aggregator")

        # Params
        self.declare_parameter("coalesce_ms", 80)
        self.declare_parameter("timeout_tick_s", 2.0)  # safety tick while waiting
        self.declare_parameter("battery_threshold_s", 15.0)
        self.declare_parameter("num_packages", 20)

        self.coalesce_ms = int(self.get_parameter("coalesce_ms").value)
        self.timeout_tick_s = float(self.get_parameter("timeout_tick_s").value)
        self.battery_threshold_s = float(self.get_parameter("battery_threshold_s").value)
        self.num_packages = int(self.get_parameter("num_packages").value)

        # Publishers
        self.pub_world = self.create_publisher(String, "/planner/world_state", 10)

        # Subscriptions
        self.sub_robot = self.create_subscription(String, "/robot_state", self.on_robot_state, 10)
        self.sub_action = self.create_subscription(String, "/action_result", self.on_action_result, 10)
        self.sub_station = self.create_subscription(String, "/station_events", self.on_station_event, 10)

        # Internal state
        self.robot_state: Dict[str, Any] = {
            "robot_location": "S",
            "robot_carrying_idx": -1,
            "battery": 100.0,
        }
        self.packages: List[Dict[str, Any]] = default_packages(self.num_packages)

        self.jobs_done: int = 0
        self.jobs_failed_to_E: int = 0

        self.episode_start_wall: float = now_wall()
        self.last_decision_wall: float = now_wall()

        self.pending_events: List[Event] = []
        self.last_event_wall: float = now_wall()

        # Timer for coalescing + timeout ticks
        self.flush_timer = self.create_timer(self.coalesce_ms / 1000.0, self.flush_if_ready)
        self.timeout_timer = self.create_timer(self.timeout_tick_s, self.on_timeout_tick)

        self.get_logger().info("WorldStateAggregator started.")

    # ---------------- Subscriptions ----------------

    def on_robot_state(self, msg: String) -> None:
        try:
            data = json_loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"/robot_state JSON parse error: {e}")
            return

        # Expect fields at least: robot_location, battery, carrying_idx
        old_loc = self.robot_state.get("robot_location", "S")
        self.robot_state.update({
            "robot_location": data.get("robot_location", old_loc),
            "robot_carrying_idx": int(data.get("robot_carrying_idx", self.robot_state.get("robot_carrying_idx", -1))),
            "battery": float(data.get("battery", self.robot_state.get("battery", 0.0))),
        })

        # Optional: location change event, but not required if MOVE_TO ends produce ACTION_RESULT
        new_loc = self.robot_state["robot_location"]
        if new_loc != old_loc:
            self.enqueue_event("ROBOT_LOCATION_CHANGED", {"from": old_loc, "to": new_loc})

        # Battery threshold event
        if self.robot_state["battery"] <= self.battery_threshold_s:
            self.enqueue_event("BATTERY_THRESHOLD", {"battery": self.robot_state["battery"], "threshold": self.battery_threshold_s})

    def on_action_result(self, msg: String) -> None:
        try:
            data = json_loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"/action_result JSON parse error: {e}")
            return

        # Expected: {type:"ACTION_RESULT", payload:{action_type,status,duration_dt,...}}
        evt = Event.from_json(data if "type" in data else {"type": "ACTION_RESULT", "stamp": now_wall(), "payload": data})
        self.pending_events.append(evt)
        self.last_event_wall = now_wall()

        # You can update internal state here if action_result contains useful deltas:
        payload = evt.payload
        if payload.get("action_type") == "PICK" and payload.get("status") == "SUCCEEDED":
            # If your executor already updates /robot_state + /station_events you can omit this.
            pass

    def on_station_event(self, msg: String) -> None:
        """
        station_events should emit:
        - QUEUE_CHANGED
        - PACKAGE_STATUS_CHANGED
        - optional other events
        """
        try:
            data = json_loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"/station_events JSON parse error: {e}")
            return

        evt = Event.from_json(data)
        self.pending_events.append(evt)
        self.last_event_wall = now_wall()

        # Apply some internal bookkeeping if present (optional)
        if evt.type == "JOB_DONE":
            self.jobs_done += 1
        elif evt.type == "JOB_FAILED_TO_E":
            self.jobs_failed_to_E += 1

        # If station events include package updates, apply them (optional but helpful)
        if evt.type == "PACKAGE_STATUS_CHANGED":
            pidx = evt.payload.get("package_idx", None)
            if isinstance(pidx, int) and 0 <= pidx < len(self.packages):
                # allow partial update
                self.packages[pidx].update(evt.payload.get("update", {}))

        if evt.type == "QUEUE_CHANGED":
            # no-op here; planner can read queue from station payload if you add it
            pass

    # ---------------- Event + Flush logic ----------------

    def enqueue_event(self, evt_type: str, payload: Dict[str, Any]) -> None:
        self.pending_events.append(Event(type=evt_type, stamp=now_wall(), payload=payload))
        self.last_event_wall = now_wall()

    def flush_if_ready(self) -> None:
        if not self.pending_events:
            return

        # Coalesce window: flush all pending since last tick
        events = self.pending_events[:]
        self.pending_events.clear()

        self.publish_world_state(events, decision_required=True)

    def on_timeout_tick(self) -> None:
        # Safety tick: if nothing happened recently, still allow the planner to wake up (mainly for WAIT)
        if (now_wall() - self.last_event_wall) >= self.timeout_tick_s:
            self.publish_world_state(
                events=[Event(type="TIMEOUT_TICK", stamp=now_wall(), payload={"timeout_s": self.timeout_tick_s})],
                decision_required=True,
            )

    def publish_world_state(self, events: List[Event], decision_required: bool) -> None:
        wall_now = now_wall()
        dt = max(0.0, wall_now - self.last_decision_wall)
        self.last_decision_wall = wall_now

        episode_time = wall_now - self.episode_start_wall

        # Build WorldState JSON
        ws = WorldState(
            stamp=wall_now,
            decision_required=decision_required,
            dt_since_last_decision=dt,
            last_events=[{"type": e.type, "stamp": e.stamp, "payload": e.payload} for e in events],
            robot_location=str(self.robot_state.get("robot_location", "S")),
            robot_carrying_idx=int(self.robot_state.get("robot_carrying_idx", -1)),
            battery=float(self.robot_state.get("battery", 0.0)),
            packages=self.packages,
            jobs_done=self.jobs_done,
            jobs_failed_to_E=self.jobs_failed_to_E,
            episode_time=episode_time,
        )

        msg = String()
        msg.data = ws.to_json()
        self.pub_world.publish(msg)


def main() -> None:
    rclpy.init()
    node = WorldStateAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
