#!/usr/bin/env python3
"""Validate a calibration/world or inspect a generated assembly; optionally check live ROS."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
from assemble import ROOT, arguments
from my_robot.registry import Registry
from my_robot.assembly import compose_world


def validate_bundle(directory):
    directory=Path(directory).resolve()
    resolved=json.loads((directory/'resolved.json').read_text())
    if resolved['version']!=1: raise ValueError('Unsupported assembly version')
    if json.loads((directory/'bridge.yaml').read_text())!=resolved['bridges']:
        raise ValueError('Bridge configuration differs from resolved assembly')
    if json.loads((directory/'tf.json').read_text())!=resolved['transforms']:
        raise ValueError('TF configuration differs from resolved assembly')
    for name in ('model.sdf','world.sdf'): ET.parse(directory/name)
    for adapter in resolved['adapters']:
        path=(directory/adapter['path']).resolve()
        if not path.is_relative_to(directory) or not path.is_file(): raise ValueError('Missing adapter')
    return resolved


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    arguments(parser)
    parser.add_argument('--assembly',type=Path)
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--duration',type=float,default=20)
    args=parser.parse_args()
    try:
        if args.assembly:
            validate_bundle(args.assembly)
        else:
            registry=Registry(ROOT,rover=args.rover,calibration=args.calibration,enable=args.enable,disable=args.disable)
            compose_world(registry,args.world,args.spawn_pose)
        print('PASS: configuration and assembly structure')
        if args.live:
            command=[sys.executable,str(ROOT/'scripts/validate_sensors.py'),'--duration',str(args.duration)]
            if args.assembly: command+=['--assembly',str(args.assembly)]
            else:
                command+=['--calibration',str(args.calibration),'--rover',args.rover]
                for name in args.enable: command+=['--enable',name]
                for name in args.disable: command+=['--disable',name]
            return subprocess.call(command)
    except (ValueError,KeyError,OSError,ET.ParseError) as error:
        parser.exit(2,f'FAIL: {error}\n')
    return 0


if __name__=='__main__': raise SystemExit(main())
