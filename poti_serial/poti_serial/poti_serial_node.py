import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import serial
import serial.tools.list_ports

# --- Configuration ---
ARDUINO_PORT = '/dev/ttyACM0'  # Change if your Arduino appears under a different path
BAUD_RATE    = 115200          # Must match Serial.begin() in your Arduino sketch
MAX_RAW      = 1023.0          # ADC max value (0..1023)

MIN_DELTA    = 0.01            # Minimum change in normalized value (0..1) before publishing


class PotiSerialNode(Node):
    def __init__(self):
        super().__init__('poti_serial_node')

        # Publisher for normalized potentiometer value (0.0 .. 1.0)
        self.publisher_ = self.create_publisher(Float64, 'poti_value', 10)
        self.last_value = None  # Last published normalized value (for deadband)

        # Try to open the serial port
        try:
            self.ser = serial.Serial(
                ARDUINO_PORT,
                BAUD_RATE,
                timeout=0.1
            )
            self.get_logger().info(
                f"Opened serial port {ARDUINO_PORT} at {BAUD_RATE} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f"Failed to open serial port {ARDUINO_PORT}: {e}"
            )
            self.ser = None

        # Timer: run the read loop periodically (50 Hz = every 0.02 s)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        """Read one line from serial, convert to normalized Float64, and publish with deadband."""
        if self.ser is None:
            return

        try:
            # Read one line from serial, decode to string, strip whitespace/newline
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return  # No data this cycle

            # Expect an integer string like "0" .. "1023"
            raw_value = int(line)

            # Clamp to valid ADC range
            if raw_value < 0:
                raw_value = 0
            elif raw_value > MAX_RAW:
                raw_value = MAX_RAW

            # Normalize to 0.0 .. 1.0
            norm = raw_value / MAX_RAW

            # Deadband: ignore tiny changes compared to the last published value
            if self.last_value is not None:
                if abs(norm - self.last_value) < MIN_DELTA:
                    return  # Change too small, skip publishing

            # Update stored value and publish
            self.last_value = norm

            msg = Float64()
            msg.data = norm
            self.publisher_.publish(msg)

            # Optional debug:
            # self.get_logger().info(f"raw={raw_value} norm={norm:.3f}")

        except ValueError:
            # Could not parse an integer from the line; ignore gracefully
            # self.get_logger().warn(f"Non-numeric line from serial: {line}")
            pass
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PotiSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
