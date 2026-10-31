"""Convert bridged wheel joint states into custom encoder messages."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from my_robot_msgs.msg import WheelEncoders


class WheelEncoderPublisher(Node):
    def __init__(self):
        super().__init__('wheel_encoder_publisher')
        publish_rate = self.declare_parameter('publish_rate', 50.0).value
        if not math.isfinite(publish_rate) or publish_rate <= 0:
            raise ValueError('publish_rate must be finite and positive')
        self.publish_period_ns = round(1e9 / publish_rate)
        self.last_publish_stamp_ns = None

        self.encoder_publisher = self.create_publisher(
            WheelEncoders, '/rover/wheel_encoders', 10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/rover/wheel_joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info('Waiting for bridged wheel joint states.')

    def joint_state_callback(self, msg: JointState):
        wheel_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        # JointState arrays share indices, but incoming joint order may vary.
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            self.get_logger().warning(
                'Skipping joint states: position and velocity must match name length.',
                throttle_duration_sec=5.0,
            )
            return

        if any(msg.name.count(name) != 1 for name in wheel_names):
            self.get_logger().warning(
                'Skipping joint states: each wheel joint must appear exactly once.',
                throttle_duration_sec=5.0,
            )
            return

        indices = [msg.name.index(name) for name in wheel_names]
        angles = [msg.position[index] for index in indices]
        velocities = [msg.velocity[index] for index in indices]
        if not all(math.isfinite(value) for value in angles + velocities):
            self.get_logger().warning(
                'Skipping joint states: wheel measurements must be finite.',
                throttle_duration_sec=5.0,
            )
            return

        # Older Harmonic releases publish joint states every physics iteration.
        # Throttle by measurement time so pausing and slow rendering are safe.
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        last_stamp = self.last_publish_stamp_ns
        period = self.publish_period_ns
        if last_stamp is not None and 0 <= stamp_ns - last_stamp < period:
            return

        encoders = WheelEncoders()
        encoders.header = msg.header
        encoders.wheel_names = wheel_names
        encoders.angle_rad = angles
        encoders.velocity_rad_s = velocities
        self.encoder_publisher.publish(encoders)
        self.last_publish_stamp_ns = stamp_ns


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelEncoderPublisher()
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
