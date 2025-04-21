import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import Float32

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(Float32, '/speed', 10)
        self.arduino_serial = None
        self.running = True  # Flag to control the thread

        # Attempt to establish serial connection
        for _ in range(5):
            try:
                self.arduino_serial = serial.Serial('/dev/ttyACM0', 2000000, timeout=1)
                self.get_logger().info("✅ Serial connection established successfully.")
                self.arduino_serial.flushInput()
                break
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Serial connection failed, retrying...: {e}")

        if self.arduino_serial is None:
            self.get_logger().error("❌ Failed to establish serial connection.")
            return

        # Start a separate thread for continuous serial reading
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()

    def read_serial_data(self):
        """ Continuously reads serial data in a separate thread """
        while self.running:
            if self.arduino_serial and self.arduino_serial.in_waiting > 0:
                try:
                    line = self.arduino_serial.readline().decode(errors='ignore').strip()
                    if line:
                        try:
                            rps = float(line)
                            if 0 <= rps <= 100:
                                self.publisher_.publish(Float32(data=rps))
                                #self.get_logger().info(f"🔹 Published Speed: {rps:.2f} r/s")
                            else:
                                self.get_logger().error(f"⚠ Unrealistic RPS value received: {rps}")
                        except ValueError:
                            self.get_logger().error(f"⚠ Invalid data received: '{line}'")
                except Exception as e:
                    self.get_logger().error(f"❌ Error reading serial data: {e}")

    def destroy_node(self):
        """ Stops the reading thread before shutting down """
        self.get_logger().info("🛑 Stopping SerialReader node...")
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()  # Ensure the thread exits safely
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    try:
        rclpy.spin(node)  # Keeps ROS2 node alive
    except KeyboardInterrupt:
        node.get_logger().info("🔌 Shutting down SerialReader...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  