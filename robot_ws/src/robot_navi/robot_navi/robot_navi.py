import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool
from .grid_path import PathLoader
from .grid_map import resolution

import math, time, random
from typing import Dict, Tuple
import json

from interfaces.msg import RobotState
from interfaces.srv import Enqueue, Dequeue, ResetEpisode
from interfaces.action import Pick, Charge, PlannerCmd
from action_msgs.msg import GoalStatus 
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np


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

share_dir = get_package_share_directory('robot_navi')
json_path = os.path.join(share_dir, 'grid_path.json')

class RobotNavi(Node):    
    def __init__(self):
        super().__init__('robot_navi')
        self.cb = ReentrantCallbackGroup()
        
        # Load pre-computed paths and convert to real coordinates
        self.get_logger().info('Loading pre-computed paths...')
        raw_paths_data = PathLoader.load_paths(json_path)
    
        # Convert all grid paths to world coordinates
        self.paths_data = {
            'metadata': raw_paths_data['metadata'],
            'paths': {}
        }
        
        for path_key, path_info in raw_paths_data['paths'].items():
            if path_info['path'] is not None:
                # Convert grid coordinates to world coordinates
                grid_path = path_info['path']
                world_path = [self.grid_to_world_helper(gx, gy, raw_paths_data['metadata']) 
                            for gx, gy in grid_path]
                
                # Store converted path
                self.paths_data['paths'][path_key] = {
                    'from': path_info['from'],
                    'to': path_info['to'],
                    'path': world_path,  # Now in world coordinates
                    'length': path_info['length'],
                    'time': path_info['time']
                }
            else:
                self.paths_data['paths'][path_key] = path_info
        
        self.get_logger().info(f'Converted {len(self.paths_data["paths"])} paths to world coordinates')
        
        # Current state (RAW odometry values, start at 0)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_location = 'START'
        
        # ADD ODOMETRY OFFSET:
        self.odom_offset_x = START_COORDS[0]  # 1.0
        self.odom_offset_y = START_COORDS[1]  # 13.0
        
        # Navigation state
        self.current_path = None
        self.current_waypoint_index = 0
        self.goal_tolerance = 0.2  # meters
        self.nav_goal_tolerance = 0.5
        self.location_name_map = {
            "ON_TRANSIT": "START",
            "S": "START"
        }
        
        # Control parameters
        self.nav_linear_speed = 2.0  # m/s
        self.nav_angular_speed = 10.0  # rad/s

        self.state_rate = 10.0 # update state rate in hz

        # Robot State
        self._tick_id = 0
        self._pose: Tuple[float, float] = START_COORDS
        self.robot_location: str = 'ON_TRANSIT'
        self.carrying_idx: int = -1
        self.battery_status: float = 100.0
        self.robot_mode: str = "IDLE" 

        # Latched-like /robot_state publisher (Transient Local)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_state = self.create_publisher(RobotState, "/robot_state", qos)

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
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/warehouse_robot/odometry',
            self.odom_callback,
            10,
            callback_group=self.cb
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/robot/battery_status',
            self.battery_callback,
            10,
            callback_group=self.cb
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
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

        self.charging_client = self.create_client(
            SetBool,
            '/robot/set_charging',
            callback_group=self.cb
        )
        
        # Periodic /robot_state publish
        self.timer = self.create_timer(1.0 / max(self.state_rate, 1.0), self.publish_state, callback_group=self.cb)

        self.publish_state()
        
        self.get_logger().info('Robot Navigator initialized')
        self.get_logger().info(f'Starting location: {self.current_location}')
    
    def on_goal(self, goal_request: PlannerCmd.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # Helper for publish_state
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

    def battery_callback(self, msg: BatteryState):
        """Update battery status from battery controller"""
        self.battery_status = msg.capacity
    
    # Convert from scaled grid to real world coordinates
    def grid_to_world_helper(self, gx: int, gy: int, metadata: dict) -> Tuple[float, float]:
        map_width = metadata['map_width']
        map_height = metadata['map_height']
        res = metadata['resolution']
        
        wx = (gx + 0.5) * res
        wy = map_height - (gy + 0.5) * res
        
        return (wx, wy)

    # Get odometry from robot
    def odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        
        self.current_x = raw_x + self.odom_offset_x
        self.current_y = raw_y + self.odom_offset_y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # debug
        #print(f'odom_callback: {self.current_x, self.current_y}')
    
    def stop_robot(self):
        """Stop the robot"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def navigate_to_waypoint(self, target_x: float, target_y: float) -> bool:
        """
        Navigate to a single waypoint. Returns True if reached.
        """
        # Calculate distance to waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info(f'Target: ({target_x:.2f}, {target_y:.2f}), Current: ({self.current_x:.2f}, {self.current_y:.2f}), Distance: {distance:.2f}m')
        
        # Check if reached
        if distance < self.nav_goal_tolerance:
            return True
        
        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
        
        # Create velocity command
        cmd = Twist()
        
        # If need to turn more than 10 degrees, just rotate
        if abs(yaw_error) > 0.17:  # ~10 degrees
            cmd.angular.z = self.nav_angular_speed if yaw_error > 0 else -self.nav_angular_speed
            cmd.linear.x = 0.0
        else:
            # Move forward with correction
            cmd.linear.x = self.nav_linear_speed
            cmd.angular.z = 2.0 * yaw_error  # Proportional correction
        
        self.cmd_vel_pub.publish(cmd)
        return False

    # Alternative version with teleport instead of driving
    def execute_move_to(self, goal_handle, target_station: str) -> PlannerCmd.Result:
        """Execute MOVE_TO - Teleport waypoint by waypoint"""
        fb = PlannerCmd.Feedback()
        
        # Map current location
        from_location = self.robot_location
        if from_location in self.location_name_map:
            from_location = self.location_name_map[from_location]
        
        if self.paths_data is None:
            goal_handle.abort()
            return PlannerCmd.Result(result="FAIL no paths", dt=0.0)
        
        # Get path
        from_key = from_location if from_location == "START" else f"STATION_{from_location}"
        to_key = f"STATION_{target_station}"
        lookup_key = f"('{from_key}', '{to_key}')"
        path_info = self.paths_data['paths'].get(lookup_key)
        
        if not path_info or path_info['path'] is None:
            goal_handle.abort()
            return PlannerCmd.Result(result="FAIL no path", dt=0.0)
        
        world_path = path_info['path']
        
        self.get_logger().info(f'Teleporting from {from_key} to {to_key}: {len(world_path)} waypoints')
        
        # Set state
        self.robot_mode = "BUSY"
        self.robot_location = "ON_TRANSIT"
        self.publish_state()
        
        t0 = time.time()
        
        # Teleport through each waypoint
        import subprocess
        
        # Calculate time per cell movement (resolution / speed)
        time_per_cell = 0.5
        
        for idx, (wx, wy) in enumerate(world_path):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                dt = float(time.time() - t0)
                self.robot_mode = "IDLE"
                self.publish_state()
                return PlannerCmd.Result(result="CANCELLED", dt=dt)
            
            # Teleport robot in Gazebo to this waypoint
            subprocess.run([
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "warehouse_robot" position: {{x: {wx} y: {wy} z: 0.15}}'
            ], capture_output=True)
            
            self.get_logger().info(f'Teleported to waypoint {idx+1}/{len(world_path)}: ({wx:.2f}, {wy:.2f})')
            
            # Update internal position
            self.current_x = wx
            self.current_y = wy
            
            # Publish feedback
            fb.progress = float(idx + 1) / float(len(world_path))
            goal_handle.publish_feedback(fb)
            
            # Wait for movement time (1 second per 2m cell)
            time.sleep(time_per_cell)
        
        # Calculate total time
        dt = float(time.time() - t0)
        
        # Update final state
        self.robot_location = target_station
        self.robot_mode = "IDLE"
        self.publish_state()
        
        goal_handle.succeed()
        self.get_logger().info(f'Successfully teleported to {target_station} in {dt:.2f}s')
        return PlannerCmd.Result(result="OK", dt=dt)

    # Difficulties with this approach of driving to the station, robot gets stuck!
    # def execute_move_to(self, goal_handle, target_station: str) -> PlannerCmd.Result:
    #     """
    #     Execute MOVE_TO command using A* pre-computed paths.
    #     """
    #     fb = PlannerCmd.Feedback()
        
    #     # Map current location name
    #     from_location = self.robot_location
    #     if from_location in self.location_name_map:
    #         from_location = self.location_name_map[from_location]
        
    #     # Check if paths are loaded
    #     if self.paths_data is None:
    #         self.get_logger().error('Paths not loaded!')
    #         goal_handle.abort()
    #         res = PlannerCmd.Result()
    #         res.result = "FAIL no paths"
    #         res.dt = 0.0
    #         return res
        
    #     # Get path from lookup table
    #     # Station names in grid are like "STATION_A", but action uses just "A"
    #     from_key = from_location if from_location == "START" else f"STATION_{from_location}"
    #     to_key = f"STATION_{target_station}"
        
    #     lookup_key = f"('{from_key}', '{to_key}')"
    #     path_info = self.paths_data['paths'].get(lookup_key)
        
    #     if not path_info or path_info['path'] is None:
    #         self.get_logger().error(f'No path from {from_key} to {to_key}')
    #         goal_handle.abort()
    #         res = PlannerCmd.Result()
    #         res.result = "FAIL no path"
    #         res.dt = 0.0
    #         return res
        
    #     world_path = path_info['path']

    #     # Perhaps fix?
    #     world_path = self.simplify_path(world_path)
        
    #     self.get_logger().info(f'Navigating from {from_key} to {to_key}: {len(world_path)} waypoints')

    #     self.get_logger().info(f'Current robot pose: ({self.current_x}, {self.current_y})')
    #     self.get_logger().info(f'First waypoint: {world_path[0]}')
    #     self.get_logger().info(f'Last waypoint: {world_path[-1]}')
        
    #     # Execute navigation
    #     self.robot_mode = "BUSY"
    #     self.robot_location = "ON_TRANSIT"
    #     self.publish_state()
        
    #     t0 = time.time()
    #     waypoint_idx = 0
        
    #     while waypoint_idx < len(world_path) and rclpy.ok():
    #         # Check for cancellation
    #         if goal_handle.is_cancel_requested:
    #             self.stop_robot()
    #             goal_handle.canceled()
    #             dt = float(time.time() - t0)
    #             self.robot_mode = "IDLE"
    #             self.publish_state()
    #             res = PlannerCmd.Result()
    #             res.result = "CANCELLED"
    #             res.dt = dt
    #             return res
            
    #         # Navigate to current waypoint
    #         target_x, target_y = world_path[waypoint_idx]
    #         reached = self.navigate_to_waypoint(target_x, target_y)
            
    #         if reached:
    #             self.get_logger().info(f'Reached waypoint {waypoint_idx + 1}/{len(world_path)}')
    #             waypoint_idx += 1
            
    #         # Publish feedback
    #         fb.progress = float(waypoint_idx) / float(len(world_path))
    #         goal_handle.publish_feedback(fb)
            
    #         time.sleep(0.01)  # Control loop rate, 50hz
        
    #     # Stop robot
    #     self.stop_robot()
        
    #     # Calculate actual time taken
    #     dt = float(time.time() - t0)
        
    #     # Update pose and location
    #     if waypoint_idx >= len(world_path):
    #         # Successfully reached destination
    #         #self._pose = STATION_COORDS[target_station]
    #         self.robot_location = target_station
    #         self.robot_mode = "IDLE"
    #         self.publish_state()
            
    #         goal_handle.succeed()
    #         res = PlannerCmd.Result()
    #         res.result = "OK"
    #         res.dt = dt
    #         self.get_logger().info(f'Successfully reached {target_station} in {dt:.2f}s')
    #         return res
    #     else:
    #         # Did not complete
    #         self.robot_mode = "IDLE"
    #         self.publish_state()
            
    #         goal_handle.abort()
    #         res = PlannerCmd.Result()
    #         res.result = "FAIL incomplete"
    #         res.dt = dt
    #         return res
    
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

        # MOVE_TO
        if verb == "MOVE_TO":
            if len(parts) != 2:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL invalid syntax"
                res.dt = 0.0
                return res
            
            target = parts[1]
            
            # Validate target station
            if target not in STATION_COORDS:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL invalid station"
                res.dt = 0.0
                return res
            
            # Execute navigation with A* path
            return self.execute_move_to(goal_handle, target)
        
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

            self.robot_mode = "IDLE"
            self.publish_state()

            (goal_handle.succeed() if ok else goal_handle.abort())
            res = PlannerCmd.Result()
            res.result = "OK" if ok else "FAIL"
            res.dt = dt
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
            # Check if at charging station
            if self.robot_location != "F":
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL not at charging station (must be at station F)"
                res.dt = 0.0
                self.get_logger().warn(f"Attempted to charge at {self.robot_location}, must be at F")
                return res
            self.robot_mode = "BUSY"
            self.publish_state()
            
            t0 = time.time()
            
            # Enable charging
            if not self.charging_client.wait_for_service(timeout_sec=5.0):
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL charging service not available"
                res.dt = 0.0
                return res
            
            # Start charging
            req = SetBool.Request()
            req.data = True
            future = self.charging_client.call_async(req)
            
            # Wait for service response
            while not future.done():
                time.sleep(0.01)
            
            if not future.result().success:
                goal_handle.abort()
                res = PlannerCmd.Result()
                res.result = "FAIL could not start charging"
                res.dt = 0.0
                return res
            
            self.get_logger().info("Charging started")
            
            # Wait until battery is full (or timeout)
            charge_timeout = 60.0  # Maximum 60 seconds
            while self.battery_status < 99.0 and (time.time() - t0) < charge_timeout:
                time.sleep(0.1)
            
            # Stop charging
            req.data = False
            self.charging_client.call_async(req)
            
            dt = float(time.time() - t0)
            
            self.robot_mode = "IDLE"
            self.publish_state()
            
            goal_handle.succeed()
            res = PlannerCmd.Result()
            res.result = "OK"
            res.dt = float(dt)
            self.get_logger().info(f"Charging complete: {self.battery_status:.1f}% in {dt:.2f}s")
            return res

        # Unknown command
        goal_handle.abort()
        res = PlannerCmd.Result()
        res.result = "FAIL"
        res.dt = 0.0
        return res

def main(args=None):
    rclpy.init(args=args)
    
    navigator = RobotNavi()

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()