import rclpy
from rclpy.node import Node
import busio
import board
import adafruit_bno055
from std_msgs.msg import Float32, Bool
import time

# Constants
SERVO_MIN_PWM = 1000
SERVO_MAX_PWM = 1600
MAX_ANGLE = 30  # Max steering angle
TURN_ANGLE = 28  # Steering angle during turn
TURN_THRESHOLD = 175  # Degrees for turn completion, now checking full 180 degrees instead of 175

# Initialize IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = 0x0C  # NDOF mode

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('desired_heading', 0.0)

        # Publishers & Subscribers
        self.pwm_publisher = self.create_publisher(Float32, '/imu_pwm', 10)
        self.turn_mode_subscription = self.create_subscription(Bool, '/turn_mode', self.enable_turn_mode, 10)
        self.stop_publisher = self.create_publisher(Bool, '/stop_signal', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # PID parameters
        self.Kp = 2.0
        self.Ki = 0.2
        self.Kd = 0.1

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        # Turn mode variables
        self.turn_mode = False
        self.initial_heading = None

    def enable_turn_mode(self, msg):
        """Enable turn mode and record initial heading if not already set."""
        if msg.data:
            if self.initial_heading is None:  # Only set once per turn
                self.initial_heading = self.read_imu_heading()
                if self.initial_heading is None:
                    self.get_logger().error("❌ IMU failed to provide initial heading!")
                    return

                self.get_logger().info(f"🔄 Turn mode activated! Initial Heading: {self.initial_heading:.2f}°")
                self.turn_mode = True
            else:
                self.get_logger().warn("⚠️ Turn mode already activated!")

    def read_imu_heading(self):
        """Reads the IMU heading safely."""
        try:
            heading, _, _ = sensor.euler
            return heading if heading is not None else None
        except Exception as e:
            self.get_logger().error(f"IMU read error: {e}")
            return None

    def compute_steering_angle(self, current_heading, desired_heading=0):
        """Calculates steering angle using PID control."""
        error = desired_heading - current_heading

        # Normalize error to shortest path (-180 to 180)
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # PID calculations
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0
        self.prev_error = error

        # Prevent integral windup
        self.integral = max(-20, min(20, self.integral))

        # Compute PID output
        steering_angle = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        steering_angle = max(-MAX_ANGLE, min(MAX_ANGLE, steering_angle))

        return steering_angle

    def angle_to_pwm(self, angle):
        """Converts steering angle to PWM value."""
        pwm = int(((angle + MAX_ANGLE) / (2 * MAX_ANGLE)) * (SERVO_MAX_PWM - SERVO_MIN_PWM) + SERVO_MIN_PWM)
        return pwm

    def control_loop(self):
        """Main control loop for steering adjustment."""
        current_heading = self.read_imu_heading()
        if current_heading is None:
            self.get_logger().warn("⚠️ Failed to read IMU heading, skipping this loop cycle.")
            return

        if self.turn_mode:
            self.get_logger().info(f"🔄 Turning... Current Heading: {current_heading:.2f}°")

            # Check if 180-degree turn is completed, accounting for wraparound
            heading_diff = abs(self.initial_heading - current_heading)

            # Correct for wraparound (crossing 0 degrees)
            if heading_diff > 180:
                heading_diff = 360 - heading_diff

            # Log the heading difference for error handling
            self.get_logger().info(f"🔧 Heading Difference: {heading_diff:.2f}°")

            if heading_diff >= TURN_THRESHOLD:
                self.get_logger().info("✅ 180° turn completed! Switching to normal mode...")

                # Publish the stop signal to stop the robot
                stop_msg = Bool(data=True)
                self.stop_publisher.publish(stop_msg)  # Send stop signal

                self.turn_mode = False  # Disable turn mode
                self.initial_heading = None  # Reset heading for future turns
                return

            # Keep steering at 15° during turn mode (Inverting to turn left)
            steering_angle = -TURN_ANGLE  # Change the sign for left turn

        else:
            # Normal PID-based steering
            desired_heading = self.get_parameter('desired_heading').value
            steering_angle = self.compute_steering_angle(current_heading, desired_heading)

        pwm_value = self.angle_to_pwm(steering_angle)
        pwm_msg = Float32()
        pwm_msg.data = float(pwm_value)
        self.pwm_publisher.publish(pwm_msg)

        self.get_logger().info(f"Heading: {current_heading:.2f}°, Steering: {steering_angle:.2f}°, PWM: {pwm_value}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()