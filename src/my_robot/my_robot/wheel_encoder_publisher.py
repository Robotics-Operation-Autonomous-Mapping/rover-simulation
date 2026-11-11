"""Convert bridged wheel joint states into custom encoder messages."""

import math
import random

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from my_robot_msgs.msg import WheelEncoders


class EncoderNoise:
    """Independent, zero-mean Gaussian measurement noise per wheel and sample."""

    def __init__(self, angle_stddev=0.001, velocity_stddev=0.01, seed=-1):
        for name, value in (
            ('angle_noise_stddev', angle_stddev),
            ('velocity_noise_stddev', velocity_stddev),
        ):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')
        if seed < -1:
            raise ValueError('noise_seed must be -1 (random) or a nonnegative integer.')
        self.angle_stddev = angle_stddev
        self.velocity_stddev = velocity_stddev
        self.random = random.Random(None if seed == -1 else seed)

    def apply(self, angles, velocities):
        # Zero standard deviation gives exact passthrough without drawing noise.
        return (
            [a + self.random.gauss(0.0, self.angle_stddev) for a in angles]
            if self.angle_stddev else list(angles),
            [v + self.random.gauss(0.0, self.velocity_stddev) for v in velocities]
            if self.velocity_stddev else list(velocities),
        )


class WheelEncoderPublisher(Node):
    def __init__(self):
        super().__init__('wheel_encoder_publisher')

        descriptor = ParameterDescriptor(read_only=True)
        angle_stddev = self.declare_parameter(
            'angle_noise_stddev', 0.001, descriptor
        ).value
        velocity_stddev = self.declare_parameter(
            'velocity_noise_stddev', 0.01, descriptor
        ).value
        seed = self.declare_parameter('noise_seed', -1, descriptor).value
        self.noise = EncoderNoise(angle_stddev, velocity_stddev, seed)

        self.encoder_publisher = self.create_publisher(
            WheelEncoders, '/rover/wheel_encoders', 10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/rover/wheel_joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Waiting for bridged wheel joint states. Gaussian noise stddev: '
            f'angle={angle_stddev} rad, velocity={velocity_stddev} rad/s; seed={seed}.'
        )

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

        angles, velocities = self.noise.apply(angles, velocities)
        encoders = WheelEncoders()
        encoders.header = msg.header
        encoders.wheel_names = wheel_names
        encoders.angle_rad = angles
        encoders.velocity_rad_s = velocities
        self.encoder_publisher.publish(encoders)


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
