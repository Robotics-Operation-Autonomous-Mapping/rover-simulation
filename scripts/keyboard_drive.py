
#!/usr/bin/env python3

"""Independent left/right skid-steer throttle for ROAM."""

import argparse
import subprocess
import signal
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist

RATE_HZ = 30
DT = 1.0 / RATE_HZ

WHEEL_SEPARATION = 0.456
MAX_SPEED = 1.0
SPEED_STEP = 0.1

ACCELERATION = 0.6
BRAKING = 1.4


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def approach(current, target, step):
    if current < target:
        return min(current + step, target)
    return max(current - step, target)


class Teleop:
    def __init__(self, root, bridge_commands=True):
        self.root = root
        self.running = True

        self.target_left = 0.0
        self.target_right = 0.0

        self.left = 0.0
        self.right = 0.0

        self.node = rclpy.create_node("roam_keyboard_teleop")
        self.pub = self.node.create_publisher(
            Twist, "/rover/cmd_vel", 10
        )

        self.bridge = None
        if bridge_commands:
            self.bridge = subprocess.Popen([
                "ros2", "run", "ros_gz_bridge",
                "parameter_bridge",
                "/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        root.title("ROAM | Independent Skid-Steer")
        root.geometry("550x400")

        tk.Label(
            root,
            text="ROAM 4WD SKID-STEER",
            font=("Arial", 18, "bold")
        ).pack(pady=15)

        tk.Label(
            root,
            text="W/S: Left wheels     ↑/↓: Right wheels"
        ).pack()

        tk.Label(
            root,
            text="Space: Brake     R: Reset     Q: Quit"
        ).pack(pady=5)

        self.label = tk.Label(
            root,
            font=("Consolas", 13),
            justify="left"
        )
        self.label.pack(pady=18)

        tk.Button(
            root,
            text="BRAKE",
            command=self.brake
        ).pack()

        tk.Label(
            root,
            text="Click the window to control the rover."
        ).pack(pady=10)

        root.bind("<KeyPress>", self.key_pressed)
        root.bind("<FocusOut>", lambda _: self.brake())
        root.protocol("WM_DELETE_WINDOW", self.close)

        root.focus_force()
        self.tick()

    def key_pressed(self, event):
        key = event.keysym.lower()

        if key == "w":
            self.target_left += SPEED_STEP

        elif key == "s":
            self.target_left -= SPEED_STEP

        elif key == "up":
            self.target_right += SPEED_STEP

        elif key == "down":
            self.target_right -= SPEED_STEP

        elif key == "space":
            self.brake()

        elif key == "r":
            self.reset()

        elif key in ("q", "escape"):
            self.close()
            return

        self.target_left = clamp(
            self.target_left, -MAX_SPEED, MAX_SPEED
        )
        self.target_right = clamp(
            self.target_right, -MAX_SPEED, MAX_SPEED
        )

    def brake(self):
        self.target_left = 0.0
        self.target_right = 0.0

    def reset(self):
        self.brake()
        self.left = 0.0
        self.right = 0.0
        self.publish()

    def publish(self):
        msg = Twist()

        msg.linear.x = (self.left + self.right) / 2.0
        msg.angular.z = (
            (self.right - self.left) / WHEEL_SEPARATION
        )

        self.pub.publish(msg)

    def tick(self):
        if not self.running:
            return

        left_rate = (
            BRAKING if self.target_left == 0 else ACCELERATION
        )
        right_rate = (
            BRAKING if self.target_right == 0 else ACCELERATION
        )

        self.left = approach(
            self.left,
            self.target_left,
            left_rate * DT
        )

        self.right = approach(
            self.right,
            self.target_right,
            right_rate * DT
        )

        self.publish()

        self.label.configure(text=(
            f"LEFT SIDE\n"
            f"  Target: {self.target_left:+.2f} m/s\n"
            f"  Actual command: {self.left:+.2f} m/s\n\n"
            f"RIGHT SIDE\n"
            f"  Target: {self.target_right:+.2f} m/s\n"
            f"  Actual command: {self.right:+.2f} m/s\n\n"
            f"Linear velocity: {(self.left+self.right)/2:+.2f} m/s\n"
            f"Angular velocity: {(self.right-self.left)/WHEEL_SEPARATION:+.2f} rad/s"
        ))

        self.root.after(
            round(1000 / RATE_HZ),
            self.tick
        )

    def close(self):
        if not self.running:
            return

        self.running = False
        self.reset()

        if self.bridge is not None:
            self.bridge.terminate()
            try:
                self.bridge.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.bridge.kill()
                self.bridge.wait()

        self.node.destroy_node()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-bridge', action='store_true',
                        help='Use the command bridge started by the sensor launcher.')
    args = parser.parse_args()
    rclpy.init()
    root = tk.Tk()

    try:
        teleop = Teleop(root, bridge_commands=not args.no_bridge)
        signal.signal(signal.SIGINT, lambda *_: teleop.close())
        signal.signal(signal.SIGTERM, lambda *_: teleop.close())
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
