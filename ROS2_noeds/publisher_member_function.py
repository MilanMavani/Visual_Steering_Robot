import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Bool, Float32

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')

        # Initialize Serial Connection
        self.arduino_serial = None
        for _ in range(5):  # Retry up to 5 times
            try:
                self.arduino_serial = serial.Serial('/dev/ttyACM0', 2000000, timeout=1)
                self.arduino_serial.write(b'1\n')  # Send initial start signal
                self.get_logger().info("✅ Serial connection established successfully.")
                break
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Serial connection failed, retrying...: {e}")

        if self.arduino_serial is None:
            self.get_logger().error("❌ Failed to establish serial connection.")
            return

        # Subscribers
        self.subscription_stop = self.create_subscription(Bool, '/stop_signal', self.send_stop_command, 1)
        self.subscription_start = self.create_subscription(Bool, '/start_signal', self.send_start_command, 1)  # NEW
        self.subscription_pwm = self.create_subscription(Float32, '/imu_pwm', self.store_pwm_value, 1)

        # Timer for sending PWM every 0.5 seconds
        self.timer = self.create_timer(0.5, self.send_pwm_timer_callback)

        # Store the latest PWM value (default 1500 µs for neutral position)
        self.latest_pwm = 1500

    def send_stop_command(self, msg):
        """Sends stop command ('0') to Arduino immediately when stop signal is received."""
        if msg.data and self.arduino_serial:
            try:
                self.arduino_serial.write(b'0\n')  # Send stop signal
                self.get_logger().info("🛑 STOP command sent successfully.")
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Failed to send STOP command: {e}")

    def send_start_command(self, msg):
        """Sends start command ('1') to Arduino when restart is needed."""
        if msg.data and self.arduino_serial:
            try:
                self.arduino_serial.write(b'1\n')  # Send start signal
                self.get_logger().info("▶️ START command sent successfully.")
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Failed to send START command: {e}")

    def store_pwm_value(self, msg):
        """Stores the latest PWM value to be sent at regular intervals."""
        self.latest_pwm = int(msg.data)  # Convert to integer

    def send_pwm_timer_callback(self):
        """Sends the latest PWM value every 0.5 seconds."""
        try:
            if self.arduino_serial:
                self.arduino_serial.write(f"{self.latest_pwm}\n".encode())  # Send stored PWM value
                self.get_logger().info(f"🔄 Sent PWM to Arduino: {self.latest_pwm}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to send PWM command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()