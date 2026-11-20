# camera/pinhole_rgb

Reusable Gazebo sensor implementation. `sensor.sdf` supplies housing physics
and sensor structure. `definition.json` declares default settings, ROS/Gazebo
message mappings, QoS and required world systems.

Calibration files select this implementation and provide instance names,
extrinsic poses, topics, frames, enabled state and optional settings overrides.
They do not modify these shared component files.

See [assembly architecture](../../../docs/ARCHITECTURE.md).
