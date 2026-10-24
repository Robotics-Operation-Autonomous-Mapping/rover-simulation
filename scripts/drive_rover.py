"""Keep a timed velocity publisher alive through its final stop commands.

Invoked by drive_rover.sh after input validation and sourcing ROS 2 Jazzy.
Uses only installed ROS packages; no workspace build is needed.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions


def main():
    linear, angular, duration = map(float, sys.argv[1:])
    bridge_executable = (
        Path(get_package_prefix('ros_gz_bridge')) / 'lib/ros_gz_bridge/parameter_bridge'
    )
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('rover_timed_drive')
    # Gazebo partitions do not isolate ROS topics. Keep this bridge's ROS input
    # private to this invocation, including when testing a different partition.
    drive_topic = f'/rover/drive_helper/session_{os.getpid()}/cmd_vel'
    publisher = node.create_publisher(Twist, drive_topic, 10)
    bridge = None

    def interrupt(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def publish_for(message, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if bridge.poll() is not None:
                raise RuntimeError('The drive bridge exited before the command completed.')
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    try:
        # Keep the bridge alive on Ctrl+C so the finally block can send zeros.
        bridge = subprocess.Popen([
            str(bridge_executable),
            '/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '--ros-args', '-r', f'/rover/cmd_vel:={drive_topic}',
            '--log-level', 'warn',
        ], start_new_session=True)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            if bridge.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Could not connect to the drive bridge.')
            rclpy.spin_once(node, timeout_sec=0.05)

        # Establish Gazebo discovery while stopped before starting the timer.
        publish_for(Twist(), 1.0)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        publish_for(command, duration)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if bridge is not None and bridge.poll() is None:
                publish_for(Twist(), 0.5)
        finally:
            if bridge is not None and bridge.poll() is None:
                bridge.terminate()
                try:
                    bridge.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    bridge.kill()
                    bridge.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
