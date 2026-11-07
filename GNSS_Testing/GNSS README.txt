Hello, its THE goat Agastya Parashar, this is a basic doc explaining exactly what this test was, what the point of it was, etc.# Gazebo & ROS 2 GNSS/INS Simulation Testbed

A high-fidelity simulation testbed built in **Gazebo Sim** and **ROS 2 Jazzy** designed for validating **GNSS (20 Hz)** and **INS (100 Hz)** sensor streams. This environment focuses on maintaining precise, unthrottled sensor output frequencies and real-time factor (RTF) stability.

---

## Prerequisites

OS: Ubuntu 24.04 (recommended for Jazzy)
Middleware: ROS 2 Jazzy
Simulator: Gazebo Sim (Harmonic)
Packages:`ros_gz_bridge`, `ros_gz_sim`

---

## Environment Structure

`cube_world.sdf`**: The main simulation world file. It configures a static 10 kg test cube equipped with high-rate IMU/INS and odometry publisher plugins, set within a WGS84 geographic frame (`latitude: 51.0447`, `longitude: -114.0719`).
Physics Engine**: Configured with a `1ms` max step size and locked to a stable **1.0 Real-Time Factor (RTF)**.
---

## Quick Start

To set up the test itself:
```bash

In terminal 1 run: gz sim -r cube_world.sdf
This runs the world with the cube in it.

In terminal 2 run: 
ros2 run ros_gz_bridge parameter_bridge \
  '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/model/cube/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'

First line is for gnss, second is for driving the cube, for some reason the cube doesn't move, so the third line has no affect for now

In terminal 3 run: python3 gnss_ins_sim.py
Handles the generation and publishing of the GNSS for this test, this terminal wont display anything

Once both terminals are running. You then have the freedom to do a few things. Note: Each command must be ran in a seperate terminal

To see the rate at which the gnss messages are being sent: (Gnss: 20hz)

Run:ros2 topic hz /gnss/fix

To see the gnss data in each message:

Run: ros2 topic echo /gnss/fix


You can compare the gnss data with the ground truth. 
Note: The data will only change if u manually move it

Extra:

This sim is still far too ideal. 
It’s always a perfect RTK fix. A real receiver switches between fixed, float, and single-point modes, with errors from centimeters to meters.
Noise is independent each sample. Real GNSS errors drift slowly and are correlated over time, so consecutive fixes are not independent random draws.
No latency or dropouts. Real messages arrive slightly late, and signals disappear near obstacles.
Message format. The sim publishes plain NavSatFix. The real driver publishes that too, plus its own richer messages, so check what the driver outputs when the unit arrives. 

We modeled 20 Hz, whThese are placeholders for ideal conditions, so we’ll confirm against the real unit.
ich is the receiver’s maximum position rate, with noise in the range of the datasheet’s RTK numbers. 
