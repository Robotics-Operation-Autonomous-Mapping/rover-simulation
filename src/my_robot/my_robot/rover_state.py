"""Publish sensor TF and a synthetic INS stream from simulated rover truth."""

from copy import deepcopy
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
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


def synthetic_ins(truth, position_stddev, sample_noise=random.gauss):
    """Keep the testbed's position noise; orientation and twist remain truth."""
    ins = deepcopy(truth)
    for axis in ('x', 'y', 'z'):
        setattr(ins.pose.pose.position, axis,
                getattr(ins.pose.pose.position, axis) + sample_noise(0, position_stddev))
    for index in (0, 7, 14):
        ins.pose.covariance[index] = position_stddev ** 2
    return ins


class RoverState(Node):
    def __init__(self):
        super().__init__('rover_sensor_state')
        default_model = str(Path(get_package_share_directory('my_robot')) /
                            'models/roam_rover/model.sdf')
        model_file = self.declare_parameter('model_file', default_model).value
        self.position_stddev = self.declare_parameter('ins_position_stddev', 0.005).value
        if not math.isfinite(self.position_stddev) or self.position_stddev < 0:
            raise ValueError('ins_position_stddev must be finite and nonnegative')
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = sensor_transforms(model_file)
        for transform in transforms:
            transform.header.stamp = self.get_clock().now().to_msg()
        self.static_tf.sendTransform(transforms)
        self.dynamic_tf = TransformBroadcaster(self)
        self.ins_publisher = self.create_publisher(Odometry, '/ins/odom', 10)
        self.truth_subscription = self.create_subscription(
            Odometry, '/rover/ground_truth/odometry', self.on_truth,
            qos_profile_sensor_data
        )
        self.get_logger().info('Sensor TF and synthetic INS follow rover ground truth.')

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
        self.ins_publisher.publish(synthetic_ins(truth, self.position_stddev))


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
