"""Requirements of this four-wheel custom-message adapter."""
import xml.etree.ElementTree as ET


def validate(instance, registry):
    model = ET.parse(registry.rover_dir/registry.rover['model']).getroot().find('model')
    joints = {joint.get('name') for joint in model.findall('joint')}
    names = instance['settings']['wheel_names']
    if len(names) != 4 or len(set(names)) != 4 or not set(names) <= joints:
        raise ValueError('The custom WheelEncoders adapter requires four existing unique joints')
