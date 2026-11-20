# Synthetic INS test signal

`adapter.py:create_node` subscribes to the rover truth odometry interface.
`synthetic_ins` copies truth and adds independent Gaussian position noise;
orientation, velocity, timestamp and frames remain from truth. It is not sensor
fusion and has no sensor-local Gazebo plugin. The rover supplies truth.

Configured in `calibration/legacy.json` and `calibration/localization.json`.
See [architecture and fidelity](../../../docs/ARCHITECTURE.md).
