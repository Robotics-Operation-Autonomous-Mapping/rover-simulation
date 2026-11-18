"""Scope of the simple antenna-velocity approximation."""
from my_robot.registry import pose_values


def validate(instance, registry):
    source = next((i for i in registry.instances if i['id'] == instance['source_instance']), None)
    if source is None or source['definition'] != 'gnss' or not registry.active(source):
        raise ValueError('GNSS velocity requires an enabled GNSS source')
    if source['parent'] != registry.data['base_frame'] or any(pose_values(source['calibration']['sensor_pose'])):
        raise ValueError('This velocity adapter requires a base mount and zero sensor offset')
    if instance['frame'] != registry.rover['truth']['frame']:
        raise ValueError('GNSS velocity must use the truth ENU frame')
