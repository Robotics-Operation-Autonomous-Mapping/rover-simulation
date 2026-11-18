#!/usr/bin/env python3
"""Generate reviewable model/bridge/view artifacts; the launcher generates at runtime."""

import argparse
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/my_robot'))
from my_robot.registry import Registry, write_model, write_bridge
from my_robot.visualization import write_rviz
from my_robot.assembly import assemble


def generate(registry, directory):
    directory.mkdir(parents=True, exist_ok=True)
    model = registry.model()
    for uri in model.findall('.//mesh/uri'):
        if uri.text.startswith('meshes/'):
            uri.text = 'model://roam_rover/' + uri.text
    write_model(model, directory / 'model.sdf')
    write_bridge(registry, directory / 'bridge.yaml')
    write_rviz(registry, directory / 'sensors.rviz')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='default')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--output', type=Path)
    group.add_argument('--update-default', action='store_true')
    group.add_argument('--check', action='store_true')
    args = parser.parse_args()
    registry = Registry(ROOT / 'src/my_robot', args.profile)
    if args.output:
        assemble(registry, "road", args.output)
        return
    if args.profile != 'default':
        parser.error('--check and --update-default apply only to the default compatibility artifacts')
    targets = {'model.sdf': ROOT / 'models/roam_rover/model.sdf',
               'bridge.yaml': ROOT / 'src/my_robot/config/rover_bridge.yaml',
               'sensors.rviz': ROOT / 'src/my_robot/config/rover_sensors.rviz'}
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        generate(registry, directory)
        for name, target in targets.items():
            content = (directory / name).read_bytes()
            if args.update_default:
                target.write_bytes(content)
            elif target.read_bytes().replace(b'\r\n', b'\n') != content.replace(b'\r\n', b'\n'):
                raise SystemExit(f'Stale generated artifact: {target}; run --update-default')
    print('Default generated artifacts are current.')


if __name__ == '__main__':
    main()
