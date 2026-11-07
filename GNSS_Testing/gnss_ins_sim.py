#!/usr/bin/env python3
"""Fake GNSS: Gazebo's true cube position plus a little random noise."""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

LAT0, LON0, ALT0 = 51.0447, -114.0719, 1045.0   # where (0,0) is on Earth
M_LAT = 111320.0                                # metres per degree of latitude
M_LON = 111320.0 * math.cos(math.radians(LAT0)) # metres per degree of longitude


class FakeGnss(Node):
    def __init__(self):
        super().__init__('gnss_sim')
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.create_subscription(Odometry, '/model/cube/odometry', self.on_truth, 10)
        self.count = 0

    def on_truth(self, msg):          # runs 200 times per second
        self.count += 1
        if self.count % 10 != 0:      # keep every 10th message -> 20 Hz GNSS
            return

        p = msg.pose.pose.position
        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = 'gnss'
        fix.status.status = 2         # 2 = high-accuracy (RTK-like) fix
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
    node = FakeGnss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()