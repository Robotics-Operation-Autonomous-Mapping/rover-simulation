"""Publish assembly-defined static sensor TF and dynamic rover truth TF."""

import math
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster



def transform_from_pose(parent, child, pose):
    x, y, z, roll, pitch, yaw = map(float, pose.split())
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    rotation = transform.transform.rotation
    rotation.w = cr * cp * cy + sr * sp * sy
    rotation.x = sr * cp * cy - cr * sp * sy
    rotation.y = cr * sp * cy + sr * cp * sy
    rotation.z = cr * cp * sy - sr * sp * cy
    return transform



class RoverState(Node):
    """Publish only TF; sensor processing lives with sensor implementations."""
    def __init__(self):
        super().__init__('rover_sensor_state')
        import json
        directory = self.declare_parameter('assembly_dir', '').value
        if not directory:
            raise ValueError('assembly_dir is required; launch the assembled simulation')
        resolved = json.loads((Path(directory)/'resolved.json').read_text())
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = [transform_from_pose(*t) for t in resolved['transforms']]
        for transform in transforms:
            transform.header.stamp = self.get_clock().now().to_msg()
        if transforms:
            self.static_tf.sendTransform(transforms)
        self.dynamic_tf = TransformBroadcaster(self)
        truth = resolved['rover'].get('truth')
        self.truth_subscription = None
        if truth:
            self.truth_subscription = self.create_subscription(
                Odometry, truth['topic'], self.on_truth, qos_profile_sensor_data)

    def on_truth(self, truth):
        transform = TransformStamped()
        transform.header = truth.header
        transform.child_frame_id = truth.child_frame_id
        position = truth.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = truth.pose.pose.orientation
        self.dynamic_tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoverState()
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
