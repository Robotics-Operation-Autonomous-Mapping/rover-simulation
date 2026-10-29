#!/usr/bin/env python3
"""Continuous ROAM skid-steer keyboard teleop using ROS 2 -> Gazebo bridge.

Requires Ubuntu desktop, ROS 2 Jazzy and ros_gz_bridge; python3-tk for GUI.
The window must have keyboard focus. UI speeds are commanded wheel-side speeds,
not measured encoder feedback.
"""
import math
import subprocess
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist

RATE_HZ = 30
DT = 1.0 / RATE_HZ
WHEEL_SEPARATION = 0.456
MAX_SIDE_SPEED = 1.0     # m/s at tire circumference
ACCEL = 0.6              # m/s^2 speed ramp while driving
BRAKE = 1.4              # m/s^2 speed ramp while stopping
TURN = 0.55              # m/s difference added/subtracted by steering


def approach(current, target, delta):
    return max(current - delta, min(current + delta, target))


class Teleop:
    def __init__(self, root):
        self.root = root
        self.held = set()
        self.left = self.right = 0.0
        self.limit = 0.65  # adjustable max side speed
        self.running = True

        self.node = rclpy.create_node("roam_keyboard_teleop")
        self.pub = self.node.create_publisher(Twist, "/rover/cmd_vel", 10)
        self.bridge = subprocess.Popen([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        root.title("ROAM | Skid-Steer Drive")
        root.geometry("530x340")
        root.resizable(False, False)
        tk.Label(root, text="ROAM 4WD SKID-STEER", font=("Arial", 18, "bold")).pack(pady=(15, 5))
        tk.Label(root, text="Hold W / ↑ forward, S / ↓ reverse, A / ← left, D / → right").pack()
        tk.Label(root, text="W+A or W+D for smooth curves | + / - change speed | Space brake | Q quit").pack(pady=6)
        self.label = tk.Label(root, font=("Consolas", 13), justify="left")
        self.label.pack(pady=12)
        tk.Button(root, text="STOP", command=self.stop).pack()
        tk.Label(root, text="Click this window for keyboard focus. Release keys to decelerate.").pack(pady=10)
        root.bind("<KeyPress>", self.down)
        root.bind("<KeyRelease>", self.up)
        root.bind("<FocusOut>", lambda _: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.focus_force()
        self.tick()

    @staticmethod
    def key(event):
        return event.keysym.lower()

    def down(self, event):
        k = self.key(event)
        if k in ("q", "escape"):
            self.close()
            return
        if k in ("plus", "equal", "kp_add"):
            if k not in self.held:
                self.limit = min(MAX_SIDE_SPEED, self.limit + 0.1)
        elif k in ("minus", "underscore", "kp_subtract"):
            if k not in self.held:
                self.limit = max(0.1, self.limit - 0.1)
        elif k == "space":
            self.stop()
        else:
            self.held.add(k)

    def up(self, event):
        self.held.discard(self.key(event))

    def stop(self):
        self.held.clear()

    def publish(self):
        msg = Twist()
        # Differential-drive conversion from independent side speeds.
        msg.linear.x = (self.left + self.right) / 2.0
        msg.angular.z = (self.right - self.left) / WHEEL_SEPARATION
        # Gazebo plugin's angular limit is 1.5 rad/s; limit commands here too.
        msg.angular.z = max(-1.5, min(1.5, msg.angular.z))
        self.pub.publish(msg)

    def tick(self):
        if not self.running:
            return
        forward = int(bool(self.held & {"w", "up"})) - int(bool(self.held & {"s", "down"}))
        steer = int(bool(self.held & {"a", "left"})) - int(bool(self.held & {"d", "right"}))
        # Positive steering: right faster than left -> left turn.
        target_l = forward * self.limit - steer * TURN / 2
        target_r = forward * self.limit + steer * TURN / 2
        scale = max(1, abs(target_l) / MAX_SIDE_SPEED, abs(target_r) / MAX_SIDE_SPEED)
        target_l /= scale
        target_r /= scale
        self.left = approach(self.left, target_l, (BRAKE if not forward and not steer else ACCEL) * DT)
        self.right = approach(self.right, target_r, (BRAKE if not forward and not steer else ACCEL) * DT)
        self.publish()
        self.label.configure(text=(
            f"Left wheel side:  {self.left:+.2f} m/s\n"
            f"Right wheel side: {self.right:+.2f} m/s\n"
            f"Speed setting:    {self.limit:.2f} m/s\n"
            f"Command v:        {(self.left + self.right)/2:+.2f} m/s\n"
            f"Command turn:     {max(-1.5, min(1.5,(self.right-self.left)/WHEEL_SEPARATION)):+.2f} rad/s"
        ))
        self.root.after(round(1000 / RATE_HZ), self.tick)

    def close(self):
        if not self.running:
            return
        self.running = False
        self.left = self.right = 0
        for _ in range(3):
            self.publish()
        self.bridge.terminate()
        try:
            self.bridge.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.bridge.kill()
        self.node.destroy_node()
        self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    try:
        Teleop(root)
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()