#!/usr/bin/env python3
"""Fake GNSS + INS: Gazebo's true cube position plus a little random noise."""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

LAT0, LON0, ALT0 = 51.0447, -114.0719, 1045.0   # where (0,0) is on Earth
M_LAT = 111320.0                                # metres per degree of latitude
M_LON = 111320.0 * math.cos(math.radians(LAT0)) # metres per degree of longitude


class FakeGnssIns(Node):
    def __init__(self):
        super().__init__('gnss_ins_sim')
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.ins_pub = self.create_publisher(Odometry, '/ins/odom', 10)
        self.create_subscription(Odometry, '/model/cube/odometry', self.on_truth, 10)
        self.count = 0

    def on_truth(self, msg):          # runs 200 times per second
        self.count += 1
        p = msg.pose.pose.position

        if self.count % 2 == 0:       # every 2nd message -> 100 Hz INS
            ins = Odometry()
            ins.header.stamp = msg.header.stamp
            ins.header.frame_id = 'world'
            ins.child_frame_id = 'cube'
            ins.pose.pose.position.x = p.x + random.gauss(0, 0.005)
            ins.pose.pose.position.y = p.y + random.gauss(0, 0.005)
            ins.pose.pose.position.z = p.z + random.gauss(0, 0.005)
            ins.pose.pose.orientation = msg.pose.pose.orientation
            ins.twist.twist = msg.twist.twist
            self.ins_pub.publish(ins)

        if self.count % 10 == 0:      # every 10th message -> 20 Hz GNSS
            fix = NavSatFix()
            fix.header.stamp = msg.header.stamp
            fix.header.frame_id = 'gnss'
            fix.status.status = 2     # 2 = high-accuracy (RTK-like) fix
            fix.latitude = LAT0 + (p.y + random.gauss(0, 0.02)) / M_LAT
            fix.longitude = LON0 + (p.x + random.gauss(0, 0.02)) / M_LON
            fix.altitude = ALT0 + p.z + random.gauss(0, 0.04)
            fix.position_covariance = [0.0004, 0.0, 0.0,
                                       0.0, 0.0004, 0.0,
                                       0.0, 0.0, 0.0016]
            fix.position_covariance_type = 2   # 2 = diagonal covariance known
            self.gnss_pub.publish(fix)


def main():
    rclpy.init()
    node = FakeGnssIns()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()