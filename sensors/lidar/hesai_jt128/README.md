# lidar/hesai_jt128

Reusable Gazebo sensor implementation. `sensor.sdf` supplies housing physics
and sensor structure. `definition.json` declares default settings, ROS/Gazebo
message mappings, QoS and required world systems.

Calibration files select this implementation and provide instance names,
extrinsic poses, topics, frames, enabled state and optional settings overrides.
They do not modify these shared component files.

See [assembly architecture](../../../docs/REGISTRY_README.md).

This is a generic 360x128 GPU-grid approximation, without JT128-specific firing patterns or point timing.
