"""Receive custom wheel encoder messages."""

import rclpy
from rclpy.node import Node

from my_robot_msgs.msg import WheelEncoders


class WheelEncoderSubscriber(Node):
    def __init__(self):
        super().__init__('wheel_encoder_subscriber')

        self.encoder_subscription = self.create_subscription(
            WheelEncoders,
            '/rover/wheel_encoders',
            self.encoder_callback,
            10,
        )
        self.get_logger().info('Waiting for wheel encoder messages.')

    def encoder_callback(self, msg: WheelEncoders):
        wheel_readings = '; '.join(
            f'{name}: angle={angle:.3f} rad, velocity={velocity:.3f} rad/s'
            for name, angle, velocity in zip(
                msg.wheel_names, msg.angle_rad, msg.velocity_rad_s
            )
        )
        self.get_logger().info(
            f'Encoders at {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} '
            f'(frame={msg.header.frame_id or "unspecified"}): {wheel_readings}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelEncoderSubscriber()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
