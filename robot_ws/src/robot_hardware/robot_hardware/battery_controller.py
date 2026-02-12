import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
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
            ])
        self.max_capacity = self.get_parameter('max_capacity').value
        self.charging_rate = self.get_parameter('charging_rate').value
        self.discharging_rate = self.get_parameter('discharging_rate').value
        self.battery_ticks = self.get_parameter('battery_ticks').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.capacity_wh = 100.0  # current capacity
        self.is_charging = False
        self.last_update_time = self.get_clock().now()

        self.cb_group = ReentrantCallbackGroup()

        # Publishes Battery State
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

        self.set_charging_srv = self.create_service(
            SetBool,
            '/robot/set_charging',
            self.set_charging_callback,
            callback_group=self.cb_group
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('BatterySimulator started')

    def get_battery_percentage(self) -> float:
        return self.capacity_wh / self.max_capacity

    def set_charging_callback(self, request, response):
        """Service callback to enable/disable charging."""
        self.is_charging = request.data
        response.success = True
        response.message = f"Charging {'enabled' if self.is_charging else 'disabled'}"
        self.get_logger().info(response.message)
        return response
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        time_since_last_update = (current_time - self.last_update_time).nanoseconds / 1e9

        # Check if a battery tick has elapsed
        if time_since_last_update >= self.battery_ticks:
            self.last_update_time = current_time

            if self.is_charging:
                self.capacity_wh = min(
                    self.capacity_wh + self.charging_rate,
                    self.max_capacity
                )
            else:
                self.capacity_wh = max(
                    self.capacity_wh - self.discharging_rate,
                    0.0
                )

        # Publish current battery state
        msg = BatteryState()
        msg.header.stamp = current_time.to_msg()
        msg.percentage = self.get_battery_percentage()
        msg.capacity = self.capacity_wh
        msg.design_capacity = self.max_capacity
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.is_charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.present = True
        self.battery_state_pub.publish(msg)

        # Publish charging state
        charging_msg = Bool()
        charging_msg.data = self.is_charging
        self.is_charging_pub.publish(charging_msg)

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