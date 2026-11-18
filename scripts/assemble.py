#!/usr/bin/env python3
"""Assemble reusable components without requiring ROS or launching Gazebo."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/my_robot'))
from my_robot.registry import Registry
from my_robot.assembly import assemble


def arguments(parser):
    parser.add_argument('--rover', default='atlas')
    parser.add_argument('--world', default='road')
    parser.add_argument('--calibration', type=Path, default=ROOT/'calibration/atlas.json')
    parser.add_argument('--enable', action='append', default=[], metavar='INSTANCE')
    parser.add_argument('--disable', action='append', default=[], metavar='INSTANCE')
    parser.add_argument('--spawn-pose', help='Quoted x y z roll pitch yaw')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    arguments(parser)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        registry = Registry(ROOT, rover=args.rover, calibration=args.calibration, enable=args.enable, disable=args.disable)
        assemble(registry, args.world, args.output, args.spawn_pose)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, f'Assembly failed: {error}\n')
    print(f'Assembled {args.rover} in {args.world}: {args.output.resolve()}')


if __name__ == '__main__': main()
