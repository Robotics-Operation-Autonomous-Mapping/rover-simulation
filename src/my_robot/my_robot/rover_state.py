"""Publish sensor TF and a synthetic INS stream from simulated rover truth."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

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


def sensor_transforms(model_file):
    """Read mounts and sensor origins from the same SDF Gazebo loads."""
    model = ET.fromstring(Path(model_file).read_text().lstrip()).find('model')
    transforms = []
    for joint in model.findall("joint[@type='fixed']"):
        parent, child = joint.findtext('parent'), joint.findtext('child')
        link = model.find(f"link[@name='{child}']")
        pose = link.find('pose')
        if pose is not None and pose.get('relative_to', parent) != parent:
            raise ValueError(f'{child}: sensor mount must be relative to {parent}')
        transforms.append(transform_from_pose(
            parent, child, pose.text if pose is not None else '0 0 0 0 0 0'
        ))
        for sensor in link.findall('sensor'):
            frame = {'camera': 'camera_frame', 'gpu_lidar': 'lidar_frame',
                     'imu': 'imu_frame', 'navsat': 'gnss_frame'}[sensor.get('type')]
            transforms.append(transform_from_pose(
                child, frame, sensor.findtext('pose', '0 0 0 0 0 0')
            ))
            if sensor.get('type') == 'camera':
                transforms.append(transform_from_pose(
                    frame, sensor.findtext('camera/optical_frame_id'),
                    f'0 0 0 {-math.pi / 2} 0 {-math.pi / 2}'
                ))
    return transforms


def registry_transforms(registry):
    return [transform_from_pose(*transform) for transform in registry.transforms()]


def synthetic_ins(truth, position_stddev, sample_noise=random.gauss):
    """Compatibility helper; implementation is owned by the synthetic INS sensor."""
    from my_robot.components import load_implementation
    return load_implementation('ins/synthetic/adapter.py').synthetic_ins(truth, position_stddev, sample_noise)


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
