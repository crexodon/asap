from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import PlannerCommand, json_loads, now_wall
from .masks import compute_action_mask, action_index_to_command, ACTIONS

# Optional: SB3 Maskable PPO
try:
    from sb3_contrib import MaskablePPO  # type: ignore
except Exception:
    MaskablePPO = None


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("use_model", True)

        self.model_path = str(self.get_parameter("model_path").value)
        self.use_model = bool(self.get_parameter("use_model").value)

        self.model = None
        if self.use_model and self.model_path and MaskablePPO is not None:
            if os.path.exists(self.model_path):
                self.get_logger().info(f"Loading MaskablePPO model from: {self.model_path}")
                self.model = MaskablePPO.load(self.model_path)
            else:
                self.get_logger().warn(f"model_path does not exist: {self.model_path}. Using fallback policy.")
        elif self.use_model and MaskablePPO is None:
            self.get_logger().warn("sb3_contrib not available. Install sb3-contrib to use MaskablePPO. Using fallback policy.")

        self.sub_world = self.create_subscription(String, "/planner/world_state", self.on_world_state, 10)
        self.pub_cmd = self.create_publisher(String, "/planner/command", 10)

        self.get_logger().info("PlannerNode started.")

    def on_world_state(self, msg: String) -> None:
        try:
            world = json_loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"/planner/world_state JSON parse error: {e}")
            return

        if not bool(world.get("decision_required", False)):
            return

        mask = compute_action_mask(world)
        if not any(mask):
            # Should not happen; env should reset earlier.
            self.get_logger().error("No valid actions in current state. Publishing WAIT.")
            self.publish_command({"action": "WAIT", "params": {}})
            return

        obs = self.build_observation(world)

        if self.model is not None:
            action, _ = self.model.predict(obs, deterministic=True, action_masks=np.array(mask, dtype=bool))
            action_index = int(action)
        else:
            action_index = self.fallback_policy(world, mask)

        cmd_dict = action_index_to_command(action_index, world)
        self.publish_command(cmd_dict)

    def build_observation(self, world: Dict[str, Any]) -> np.ndarray:
        """
        Minimal observation vector.
        Replace with your full 20-package vector encoding.
        """
        # Example: [battery, carrying_idx, loc_onehot...]
        battery = float(world.get("battery", 0.0))
        carrying = float(world.get("robot_carrying_idx", -1))

        loc = str(world.get("robot_location", "S"))
        stations = ["A", "B", "C", "D", "E", "F", "G", "S"]
        loc_oh = [1.0 if loc == s else 0.0 for s in stations]

        # Very small package summary: count available at current station
        packages = list(world.get("packages", []))
        avail_here = 0.0
        for p in packages:
            if str(p.get("location")) == loc and bool(p.get("availability", False)):
                avail_here += 1.0

        # jobs done progress
        jobs_done = float(world.get("jobs_done", 0))

        vec = np.array([battery, carrying, avail_here, jobs_done] + loc_oh, dtype=np.float32)
        return vec

    def fallback_policy(self, world: Dict[str, Any], mask: List[bool]) -> int:
        """
        Simple heuristic:
        - If can DROP -> DROP
        - If can PICK -> PICK
        - If low battery and can MOVE_TO(F) -> MOVE_TO(F)
        - Else move to A if possible
        - Else WAIT
        """
        # indices known from masks.py ACTIONS order
        idx_wait = 0
        idx_charge = 1
        idx_pick = 2
        idx_drop = 3

        if mask[idx_drop]:
            return idx_drop
        if mask[idx_pick]:
            return idx_pick

        battery = float(world.get("battery", 0.0))
        if battery <= 15.0:
            # find MOVE_TO F
            for i, (name, param) in enumerate(ACTIONS):
                if name == "MOVE_TO" and param.get("station") == "F" and mask[i]:
                    return i

        # else go to A
        for i, (name, param) in enumerate(ACTIONS):
            if name == "MOVE_TO" and param.get("station") == "A" and mask[i]:
                return i

        return idx_wait

    def publish_command(self, cmd_dict: Dict[str, Any]) -> None:
        cmd = PlannerCommand(
            stamp=now_wall(),
            action=str(cmd_dict.get("action", "WAIT")),
            params=dict(cmd_dict.get("params", {})),
        )
        out = String()
        out.data = cmd.to_json()
        self.pub_cmd.publish(out)


def main() -> None:
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
