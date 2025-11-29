import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import gpiod
from gpiod.line import Direction, Value


# --- GPIO Config (match your working test) ---
PUL_PIN = 21          # Pin 40
DIR_PIN = 20          # Pin 38
CHIP_PATH = "/dev/gpiochip4"

# --- Stepper Config (match your working test) ---
STEPS_PER_REV = 800   # 1/8 microstep, from your DIP switch setup
TARGET_RPM   = 60.0   # "max speed" for ramp target

# Compute min delay as in your test script
steps_per_sec = (TARGET_RPM * STEPS_PER_REV) / 60.0
MIN_DELAY     = 1.0 / steps_per_sec    # seconds between edges at max speed

# Start delay for ramp (slow start)
START_DELAY  = 0.002   # 2 ms


class StepperDriverNode(Node):
    """
    Subscribes to /stepper_target (Float64, units = revolutions, 0..1)
    and moves the motor using STEP/DIR pulses via gpiod.
    Keeps internal current_steps, so every new target is absolute.
    """

    def __init__(self):
        super().__init__('stepper_driver_node')

        # Subscribe to target in revolutions
        self.sub_target = self.create_subscription(
            Float64,
            'stepper_target',
            self.target_callback,
            10
        )

        # Track current motor position (in steps)
        self.current_steps = 0

        # Optional: publisher for current position in rev
        self.pub_position = self.create_publisher(Float64, 'stepper_position', 10)

        # Initialize GPIO
        try:
            self.chip = gpiod.Chip(CHIP_PATH)
            self.request = self.chip.request_lines(
                consumer="stepper_driver",
                config={
                    PUL_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
                    DIR_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
                }
            )
            self.get_logger().info("Stepper GPIO initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            self.request = None

        self.get_logger().info("stepper_driver_node started.")

    def target_callback(self, msg: Float64):
        """
        Called whenever a new target revolution value is published.
        Blocking movement: executes the move before returning.
        """
        if self.request is None:
            self.get_logger().error("GPIO not initialized, ignoring target.")
            return

        target_rev = msg.data

        # Clamp to [0, 1] for safety (you can relax this later)
        if target_rev < 0.0:
            target_rev = 0.0
        elif target_rev > 1.0:
            target_rev = 1.0

        target_steps = int(round(target_rev * STEPS_PER_REV))
        delta_steps  = target_steps - self.current_steps

        if delta_steps == 0:
            # No movement needed
            self.get_logger().info(
                f"Target rev={target_rev:.3f} -> target_steps={target_steps} (no change)"
            )
            return

        direction = Value.ACTIVE if delta_steps > 0 else Value.INACTIVE
        steps_to_move = abs(delta_steps)

        self.get_logger().info(
            f"Move request: target_rev={target_rev:.3f}, "
            f"current_steps={self.current_steps}, target_steps={target_steps}, "
            f"delta_steps={delta_steps}"
        )

        # Perform the motion (blocking)
        self.move_steps(direction, steps_to_move)

        # Update state
        self.current_steps = target_steps

        # Publish current position in revolutions
        pos_msg = Float64()
        pos_msg.data = self.current_steps / float(STEPS_PER_REV)
        self.pub_position.publish(pos_msg)

    def move_steps(self, direction_value: Value, steps_to_move: int):
        """
        Generate step pulses with a simple acceleration ramp,
        adapted from your working stepper_test.py.
        """
        # Set direction
        self.request.set_value(DIR_PIN, direction_value)

        current_delay = START_DELAY

        for i in range(steps_to_move):
            # STEP high
            self.request.set_value(PUL_PIN, Value.ACTIVE)
            time.sleep(current_delay)

            # STEP low
            self.request.set_value(PUL_PIN, Value.INACTIVE)
            time.sleep(current_delay)

            # Ramp up speed until MIN_DELAY
            if current_delay > MIN_DELAY:
                current_delay -= 0.00001  # adjust ramp aggressiveness if needed

        # Small pause at the end of the move (optional)
        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = StepperDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
