import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PotiToMotorNode(Node):
    """
    Subscribes to /poti_value (0.0..1.0) and publishes /stepper_target (0.0..1.0 rev).
    1 knob revolution -> 1 motor revolution.
    """

    def __init__(self):
        super().__init__('poti_to_motor_node')

        # Subscribe to normalized potentiometer value
        self.sub_poti = self.create_subscription(
            Float64,
            'poti_value',
            self.poti_callback,
            10
        )

        # Publish target motor position in revolutions
        self.pub_target = self.create_publisher(
            Float64,
            'stepper_target',
            10
        )

        self.get_logger().info("poti_to_motor_node started: mapping /poti_value -> /stepper_target")

    def poti_callback(self, msg: Float64):
        """
        Called whenever a new potentiometer value arrives.
        We simply map 0..1 -> 0..1 revolutions (linear).
        """
        p_norm = msg.data

        # Optional safety clamp
        if p_norm < 0.0:
            p_norm = 0.0
        elif p_norm > 1.0:
            p_norm = 1.0

        target_rev = p_norm  # 1:1 mapping, 0..1 -> 0..1 rev

        out_msg = Float64()
        out_msg.data = target_rev
        self.pub_target.publish(out_msg)

        # Optional debug:
        # self.get_logger().info(f"poti={p_norm:.3f} -> target_rev={target_rev:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = PotiToMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
