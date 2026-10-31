# All arrays use the same order: front left, front right, rear left, rear right.
# Joint names: front_left_wheel_joint, front_right_wheel_joint,
# rear_left_wheel_joint, rear_right_wheel_joint.
std_msgs/Header header       # Measurement timestamp and coordinate frame name
string[4] wheel_names         # Four wheel joint names
float64[4] angle_rad          # Four wheel angles, in radians
float64[4] velocity_rad_s     # Four wheel speeds, in radians/second
