import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class DiffDriveController(Node):
    """
    Differential Drive Controller for robot with Gazebo Harmonic.
    
    Acts as an interface between high-level navigation commands
    and the Gazebo Harmonic (gz-sim) simulation via ros_gz_bridge.
    
    Topic Flow:
    - Subscribes to /cmd_vel (from navigation/planner) TODO
    - Publishes to /model/vehicle/cmd_vel (to Gazebo via bridge)
    - Subscribes to /model/vehicle/odometry (from Gazebo via bridge)
    - Publishes to /robot/odom (for other nodes)
    """
    
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_velocity', 2.0),       # m/s
                ('max_angular_velocity', 1.5),      # rad/s
                ('cmd_vel_timeout', 0.5),           # seconds
                ('publish_rate', 50.0),             # Hz
            ]
        )
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # State variables
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        # Current pose (from Gazebo odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Flag if receiving commands
        self.receiving_commands = False
        
        # Subscribers
        # High-level commands from navigation/planner
        # self.cmd_vel_sub = self.create_subscription(
        #     Twist,
        #     '/cmd_vel',
        #     self.cmd_vel_callback,
        #     10
        # )
        
        # Odometry from Gazebo (via ros_gz_bridge)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/vehicle/odometry',
            self.odom_callback,
            10
        )
        
        # Publishers
        # Commands to Gazebo (via ros_gz_bridge)
        self.gz_cmd_vel_pub = self.create_publisher(
            Twist,
            '/model/vehicle/cmd_vel',
            10
        )
        
        # Republish odometry with consistent naming
        self.odom_pub = self.create_publisher(
            Odometry,
            '/robot/odom',
            10
        )
        
        # Publish movement status (for battery simulation)
        self.is_moving_pub = self.create_publisher(
            Bool,
            '/robot/is_moving',
            10
        )
        
        # Timer for safety check and status publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )
        
        self.get_logger().info('=== Differential Drive Controller Started ===')
        self.get_logger().info(f'Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max angular velocity: {self.max_angular_vel} rad/s')
        #self.get_logger().info('Listening to: /cmd_vel (high-level commands)')
        self.get_logger().info('Publishing to: /model/vehicle/cmd_vel (Gazebo)')
        self.get_logger().info('Subscribing to: /model/vehicle/odometry (Gazebo)')
        self.get_logger().info('Publishing to: /robot/odom (system-wide)')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for incoming velocity commands.
        Clamps velocities to maximum values and forwards to Gazebo.
        """
        # Update last command time
        self.last_cmd_time = self.get_clock().now()
        self.receiving_commands = True
        
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Clamp to max velocities
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))
        
        # Store current commanded velocities
        self.current_linear_vel = linear_x
        self.current_angular_vel = angular_z
        
        # Create and publish clamped command to Gazebo
        gz_cmd = Twist()
        gz_cmd.linear.x = linear_x
        gz_cmd.angular.z = angular_z
        
        self.gz_cmd_vel_pub.publish(gz_cmd)
        
        # Log if velocity was clamped
        if abs(msg.linear.x - linear_x) > 0.001 or abs(msg.angular.z - angular_z) > 0.001:
            self.get_logger().warn(
                f'Velocity clamped: linear {msg.linear.x:.3f} -> {linear_x:.3f}, '
                f'angular {msg.angular.z:.3f} -> {angular_z:.3f}'
            )
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry from Gazebo (via ros_gz_bridge).
        Stores current pose and velocity, then republishes.
        """
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        self.current_theta = self.quaternion_to_yaw(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        
        # Extract velocities
        self.current_linear_velocity = msg.twist.twist.linear.x
        self.current_angular_velocity = msg.twist.twist.angular.z
        
        self.odom_pub.publish(msg)
    
    def timer_callback(self):
        """
        Periodic callback for safety checks and status publishing.
        Stops robot if no commands received within timeout period.
        """
        current_time = self.get_clock().now()
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        # Safety: Stop robot if no command received within timeout
        if time_since_cmd > self.cmd_timeout and self.receiving_commands:
            self.get_logger().warn(
                f'No cmd_vel received for {time_since_cmd:.2f}s, stopping robot'
            )
            
            # Send stop command to Gazebo
            stop_msg = Twist()
            self.gz_cmd_vel_pub.publish(stop_msg)
            
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.receiving_commands = False
        
        # Publish moving status
        is_moving = abs(self.current_linear_velocity) > 0.01 or \
                    abs(self.current_angular_velocity) > 0.01
        
        is_moving_msg = Bool()
        is_moving_msg.data = is_moving
        self.is_moving_pub.publish(is_moving_msg)
    
    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert quaternion to yaw angle (rotation around z-axis).
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def get_current_pose(self):
        """Get current pose (x, y, theta)."""
        return (self.current_x, self.current_y, self.current_theta)
    
    def get_current_velocity(self):
        """Get current velocity (linear, angular)."""
        return (self.current_linear_velocity, self.current_angular_velocity)


def main(args=None):
    rclpy.init(args=args)
    
    node = DiffDriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        stop_msg = Twist()
        node.gz_cmd_vel_pub.publish(stop_msg)
        
        node.get_logger().info('Shutting down Differential Drive Controller')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()