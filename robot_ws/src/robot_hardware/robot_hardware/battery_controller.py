import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
import math

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_capacity', 100.0),     # Wh
                ('charging_rate', 10.0),     # Wh/tick
                ('discharging_rate', 1.0),   # Wh/tick
                ('battery_ticks', 1.0),      # seconds
                ('publish_rate', 50.0),      # Hz
            ]
        self.max_capacity = self.get_parameter('max_capacity').value
        self.charging_rate = self.get_parameter('charging_rate').value
        self.discharging_rate = self.get_parameter('discharging_rate').value
        self.battery_ticks = self.get_parameter('battery_ticks').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.capacity_wh = 100.0  # current capacity
        self.is_charging = False

        self.last_update_time = self.get_clock().now()

        self.last_cmd_time = self.get_clock().now()

        self.battery_state_pub = self.create_publisher(
            BatteryState,
            '/robot/battery_status',
            10
        )

        self.is_charging_pub = self.create_publisher(
            Bool,
            '/robot/battery_is_charging',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if time_since_cmd > self.battery_ticks:
            time_since_update = (current_time - self.last_update_time).nanoseconds / 1e9
        
        # if time_since_cmd > self.battery_ticks and not is_charging
        # decrease battery until 0 by discharge_rate

        # if time_since_cmd > self.battery_ticks and is_charging
        # increase battery until max_capacity by charging_rate
    
        # publish current battery level with battery_state_pub

def main(args=None):
    rclpy.init(args=args)

    node = BatterySimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down BatterySimulator')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()