from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import rclpy

class SpeedCalculator(Node):
    def __init__(self):
        super().__init__('speed_calculator')
        self.subscription = self.create_subscription(Float32, '/speed', self.process_speed, 1)
        self.stop_publisher = self.create_publisher(Bool, '/stop_signal', 1)
        self.turn_mode_publisher = self.create_publisher(Bool, '/turn_mode', 1)
        self.start_publisher = self.create_publisher(Bool, '/start_signal', 1)

        self.total_distance = 0.0
        self.wheel_circumference = 0.024
        self.stop_flag = False
        self.turn_flag = False  # New flag to track the turn mode state
        self.stop_timer = None  # Timer variable to ensure only one timer is created

        self.last_time = self.get_clock().now()

    def process_speed(self, msg):
        if self.stop_flag:
            return  # Don't process further if we are in stop state

        current_time = self.get_clock().now()
        time_interval = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if time_interval <= 0:
            return

        rps = msg.data
        distance_traveled = rps * self.wheel_circumference * time_interval
        self.total_distance += distance_traveled

        self.get_logger().info(f"🔹 Speed: {rps:.2f} r/s, Distance: {self.total_distance:.3f} m")

        if self.total_distance >= 8.0 and not self.stop_flag and not self.turn_flag:
            self.get_logger().info("🛑 Stopping vehicle for 1 sec before turn...")
            self.stop_publisher.publish(Bool(data=True))  # Stop the robot
            self.stop_flag = True

            # If no timer is set yet, set one to restart after 1 second
            if self.stop_timer is None:
                self.stop_timer = self.create_timer(1.0, self.start_turn)

    def start_turn(self):
        """After 1 second delay, send start command and enable turn mode."""
        if not self.turn_flag:  # Ensure turn mode is only enabled once
            self.get_logger().info("▶️ Sending start command again...")
            self.start_publisher.publish(Bool(data=True))  # Send start command again

            self.get_logger().info("🔄 Starting 180° turn with 15° steering angle...")
            self.turn_mode_publisher.publish(Bool(data=True))  # Enable turn mode

            # Reset the stop flag and set the turn flag so we don't enter the loop again
            self.stop_flag = False
            self.turn_flag = True
            self.stop_timer = None  # Reset the timer reference after turn

            # Shutdown the node after starting the turn and enabling the turn mode
            self.get_logger().info("✅ Turn initiated. Shutting down the node.")
            rclpy.shutdown()  # Shut down ROS 2 node

def main(args=None):
    rclpy.init(args=args)
    node = SpeedCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()