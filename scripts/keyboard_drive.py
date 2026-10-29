
#!/usr/bin/env python3

import curses
import subprocess

TOPIC = "/rover/cmd_vel"
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.7


def drive(linear, angular):
    message = (
        f"linear: {{x: {linear}}}, "
        f"angular: {{z: {angular}}}"
    )

    subprocess.run(
        [
            "gz", "topic",
            "-t", TOPIC,
            "-m", "gz.msgs.Twist",
            "-p", message
        ],
        check=True
    )


def main(screen):
    curses.curs_set(0)
    screen.keypad(True)

    screen.addstr(0, 0, "ROAM Keyboard Controller")
    screen.addstr(2, 0, "UP     - Forward")
    screen.addstr(3, 0, "DOWN   - Backward")
    screen.addstr(4, 0, "LEFT   - Rotate left")
    screen.addstr(5, 0, "RIGHT  - Rotate right")
    screen.addstr(6, 0, "SPACE  - Stop")
    screen.addstr(7, 0, "Q      - Quit")
    screen.addstr(9, 0, "Press a key to set movement.")

    try:
        while True:
            key = screen.getch()

            if key == curses.KEY_UP:
                drive(LINEAR_SPEED, 0.0)
                action = "FORWARD"

            elif key == curses.KEY_DOWN:
                drive(-LINEAR_SPEED, 0.0)
                action = "BACKWARD"

            elif key == curses.KEY_LEFT:
                drive(0.0, ANGULAR_SPEED)
                action = "LEFT"

            elif key == curses.KEY_RIGHT:
                drive(0.0, -ANGULAR_SPEED)
                action = "RIGHT"

            elif key == ord(" "):
                drive(0.0, 0.0)
                action = "STOPPED"

            elif key in (ord("q"), ord("Q")):
                break

            else:
                continue

            screen.move(11, 0)
            screen.clrtoeol()
            screen.addstr(11, 0, f"Movement: {action}")
            screen.refresh()

    finally:
        drive(0.0, 0.0)


if __name__ == "__main__":
    curses.wrapper(main)
