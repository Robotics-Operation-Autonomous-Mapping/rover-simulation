# 3D LiDAR & IMU Gazebo Simulation

This repository contains the configuration and workflow for simulating a **Livox Mid-360 LiDAR** and an **IMU** on a mobile robot (`vehicle_blue`) within **Gazebo Sim (SDF 1.10)** and bridging the sensor data to **ROS 2 Jazzy**.

## 🛠️ System Prerequisites

Ensure you have ROS 2 Jazzy and the ROS-Gazebo bridge installed on your system:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

To automatically source ROS 2 in every new terminal session, run:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 📂 Simulation Setup & SDF Configuration

To get started, you must **create your own SDF file from Gazebo** (or use an existing robot world) and save it to your preferred path (e.g., `/home/christopehr/roverSim2/lidarsim.sdf`). 

To make the LiDAR and IMU operational, your SDF file needs three key components: the **World-level Sensor System Plugin**, the **GPU LiDAR Sensor block**, and the **IMU Sensor block**.

### 1. World Plugin (Sensors System)

Ensure your `<world>` tag includes the Gazebo sensors system plugin so it processes all nested model sensors:

```xml
<world name="diff_drive">
  <!-- Required plugin for Gazebo Sim sensors -->
  <plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  
  <!-- Other world tags (lights, physics, etc.) go here -->
</world>
```

### 2. Sensor XML Snippets (Inside `vehicle_blue` link)

Add the following `<sensor>` blocks inside the appropriate link element (e.g., your robot's chassis link) of `vehicle_blue`:

```xml
<!-- GPU LiDAR Sensor (Livox Mid-360 simulation) -->
<sensor name="gpu_lidar" type="gpu_lidar">
  <pose>0 0 0.5 0 0 0</pose> <!-- Adjust position relative to chassis -->
  <topic>lidar_points</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.523599</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <visualize>true</visualize>
</sensor>

<!-- IMU Sensor -->
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.5 0 0 0</pose>
  <topic>imu</topic>
  <update_rate>100</update_rate>
  <always_on>1</always_on>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### Sensor Topic Mappings

| Sensor Type | Gazebo Internal Topic | ROS 2 Bridged Topic | ROS 2 Message Type |
| :--- | :--- | :--- | :--- |
| **GPU LiDAR** | `/lidar_points` | `/lidar_points` | `sensor_msgs/msg/PointCloud2` |
| **IMU Sensor** | `/imu` | `/imu` | `sensor_msgs/msg/Imu` |

## 🚀 Step-by-Step Execution Guide

### Step 1: Launch the Gazebo Simulation

Open a terminal and start the Gazebo Sim environment with your SDF file path (the `-r` flag forces physics simulation to run immediately on start):

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="<path_to_sdf_file> -r"
```

### Step 2: Start the ROS-Gazebo Bridge

Open a **second terminal** to translate the internal Gazebo sensor topics into standard ROS 2 topics:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar_points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
```

> **Note:** The `[` character specifies a unidirectional flow from **Gazebo Sim -> ROS 2**.

## 🔍 Verification & Debugging

Open a **third terminal** to confirm that the sensor data streams are flowing natively into ROS 2:

1. **List all active topics:**
   ```bash
   ros2 topic list
   ```
   *(Verify that `/lidar_points` and `/imu` are displayed).*

2. **Verify live IMU data stream:**
   ```bash
   ros2 topic echo /imu
   ```

3. **Verify live PointCloud data stream:**
   ```bash
   ros2 topic echo /lidar_points
   ```

4. **Inspect Gazebo internal topics directly (optional):**
   ```bash
   gz topic -e -t /lidar_points
   ```
