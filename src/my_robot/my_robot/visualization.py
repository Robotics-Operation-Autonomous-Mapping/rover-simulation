"""RViz presentation is cross-cutting; sensor topics come from the registry."""

import json
from pathlib import Path


def write_rviz(registry, path):
    displays = [
        {'Class': 'rviz_default_plugins/Grid', 'Name': 'Ground grid', 'Enabled': True,
         'Cell Size': 1, 'Plane Cell Count': 40, 'Reference Frame': registry.rover.get('truth', {}).get('frame', registry.data['base_frame']), 'Value': True},
        {'Class': 'rviz_default_plugins/TF', 'Name': 'Rover and sensor frames', 'Enabled': True,
         'Show Names': True, 'Marker Scale': 0.25, 'Value': True},
    ]
    for instance in registry.instances:
        if not registry.active(instance):
            continue
        kind = instance['definition']
        if kind not in ('camera', 'lidar', 'ins'):
            continue
        if kind == 'ins' and not instance['settings'].get('publish_ins', True):
            continue
        display = {'Enabled': True, 'Value': True}
        topic = instance['topics']['output'] if kind == 'ins' else instance['topics']['data']['ros']
        display['Topic'] = {'Value': topic, 'Reliability Policy': 'Reliable' if kind == 'ins' else 'Best Effort',
                            'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                            'Depth': 2 if kind == 'camera' else 5}
        if kind == 'camera':
            display.update({'Class': 'rviz_default_plugins/Image', 'Name': instance['id'], 'Transport Hint': 'raw'})
        elif kind == 'lidar':
            display.update({'Class': 'rviz_default_plugins/PointCloud2', 'Name': '3D LiDAR',
                            'Position Transformer': 'XYZ', 'Color Transformer': 'AxisColor',
                            'Axis': 'Z', 'Style': 'Points', 'Size (Pixels)': 2, 'Decay Time': 0.2})
        else:
            display.update({'Class': 'rviz_default_plugins/Odometry', 'Name': 'Synthetic INS',
                            'Keep': 100, 'Position Tolerance': 0.1, 'Angle Tolerance': 0.1})
        displays.append(display)
    document = {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'}, {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {
            'Class': '', 'Displays': displays, 'Enabled': True,
            'Global Options': {'Fixed Frame': registry.rover.get('truth', {}).get('frame', registry.data['base_frame']), 'Frame Rate': 20, 'Background Color': '40; 40; 40'},
            'Tools': [{'Class': 'rviz_default_plugins/' + name} for name in ('Interact', 'MoveCamera', 'Select', 'FocusCamera')],
            'Views': {'Current': {'Class': 'rviz_default_plugins/Orbit', 'Distance': 12, 'Pitch': 0.55,
                                  'Yaw': 0.8, 'Target Frame': registry.data['base_frame'],
                                  'Focal Point': {'X': 0, 'Y': 0, 'Z': 0}}},
        },
        'Window Geometry': {'Width': 1280, 'Height': 800},
    }
    Path(path).write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
