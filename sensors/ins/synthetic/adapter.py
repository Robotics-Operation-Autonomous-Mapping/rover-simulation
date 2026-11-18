"""Synthetic INS approximation: truth with independent position noise."""
from copy import deepcopy
import random
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def synthetic_ins(truth, position_stddev, sample_noise=random.gauss):
    result = deepcopy(truth)
    for axis in ('x','y','z'):
        setattr(result.pose.pose.position, axis, getattr(truth.pose.pose.position, axis) + sample_noise(0,position_stddev))
    for index in (0,7,14): result.pose.covariance[index] = position_stddev**2
    return result


def create_node(instance, resolved):
    node = Node(instance['id']+'_adapter')
    publisher = node.create_publisher(Odometry, instance['topics']['output'], 10)
    noise = float(instance['settings']['position_stddev'])
    node.create_subscription(Odometry, resolved['rover']['truth']['topic'],
                             lambda message: publisher.publish(synthetic_ins(message,noise)), qos_profile_sensor_data)
    return node
