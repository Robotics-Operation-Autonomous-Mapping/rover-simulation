"""Interval-average antenna velocity in world ENU, not receiver Doppler processing."""
import random
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from my_robot.kinematics import antenna_position, VelocitySampler


def create_node(instance, resolved):
    node = Node(instance['id']+'_adapter')
    source = next(i for i in resolved['instances'] if i['id']==instance['source_instance'])
    offset = tuple(map(float, source['calibration']['mount_pose'].split()[:3]))
    sampler = VelocitySampler(float(instance['settings']['publish_rate']))
    noise = float(instance['settings']['velocity_stddev'])
    publisher = node.create_publisher(TwistWithCovarianceStamped, instance['topics']['output'], qos_profile_sensor_data)

    def receive(truth):
        stamp = truth.header.stamp
        p,q = truth.pose.pose.position,truth.pose.pose.orientation
        position = antenna_position((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),offset)
        measurement = sampler.sample(stamp.sec*1_000_000_000+stamp.nanosec,position)
        if measurement is None: return
        message = TwistWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = instance['frame']
        for index,axis in enumerate(('x','y','z')):
            setattr(message.twist.twist.linear,axis,measurement[index]+random.gauss(0,noise))
            message.twist.covariance[index*7] = noise**2
        for index in (21,28,35): message.twist.covariance[index]=1e6
        publisher.publish(message)
    node.create_subscription(Odometry,resolved['rover']['truth']['topic'],receive,qos_profile_sensor_data)
    return node
