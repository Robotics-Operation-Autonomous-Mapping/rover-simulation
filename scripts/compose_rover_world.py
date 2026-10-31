#!/usr/bin/env python3
"""Compose a temporary world with the instrumented ROAM rover."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


SPAWN_POSES = {
    'road': '0 0 0.28 0 0 0',
    'maze': '-4.5 -4.5 0.23 0 0 0',
    'forest': '0 0 0.49 0 -0.12 0',
    'roam_test': '0 0 0.23 0 0 0',
}


def compose_world(workspace, world_name):
    root = ET.fromstring(
        (Path(workspace) / 'worlds' / f'{world_name}.sdf').read_text().lstrip()
    )
    world = root.find('world')
    if world is None:
        raise ValueError('Source SDF has no world')

    rovers = [include for include in world.findall('include')
              if include.findtext('uri') == 'model://roam_rover']
    if len(rovers) > 1:
        raise ValueError('World contains multiple ROAM rovers')
    rover = rovers[0] if rovers else ET.SubElement(world, 'include')
    for tag, value in [('uri', 'model://roam_rover'), ('name', 'roam_rover'),
                       ('pose', SPAWN_POSES[world_name])]:
        element = rover.find(tag)
        if element is None:
            element = ET.SubElement(rover, tag)
        element.text = value

    for suffix, system in [
        ('physics', 'Physics'), ('user-commands', 'UserCommands'),
        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'),
        ('imu', 'Imu'), ('navsat', 'NavSat'),
    ]:
        name = f'gz::sim::systems::{system}'
        plugin = next((p for p in world.findall('plugin')
                       if p.get('name') == name), None)
        if plugin is None:
            plugin = ET.SubElement(
                world, 'plugin', filename=f'gz-sim-{suffix}-system', name=name
            )
        if system == 'Sensors':
            engine = plugin.find('render_engine')
            if engine is None:
                engine = ET.SubElement(plugin, 'render_engine')
            # GPU lidar requires Ogre 2, independently of the GUI renderer.
            engine.text = 'ogre2'

    if world.find('spherical_coordinates') is None:
        coordinates = ET.SubElement(world, 'spherical_coordinates')
        for tag, value in [
            ('surface_model', 'EARTH_WGS84'), ('world_frame_orientation', 'ENU'),
            ('latitude_deg', '51.0447'), ('longitude_deg', '-114.0719'),
            ('elevation', '1045'), ('heading_deg', '0'),
        ]:
            ET.SubElement(coordinates, tag).text = value
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', choices=SPAWN_POSES)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = compose_world(Path(__file__).resolve().parent.parent, args.world)
    ET.ElementTree(root).write(args.output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
