#!/usr/bin/env python3
"""Compatibility API for the shared calibration-driven world assembler."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/my_robot'))
from my_robot.registry import Registry, read_json
from my_robot.assembly import compose_world as compose, assemble
SPAWN_POSES={name:read_json(ROOT/'worlds'/f'{name}.json')['spawn_pose'] for name in ('road','maze','forest','roam_test')}

def compose_world(workspace,world_name,profile='default'):
    return compose(Registry(workspace,profile),world_name)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world',choices=SPAWN_POSES)
    parser.add_argument('output',type=Path)
    parser.add_argument('--profile',default='default')
    args=parser.parse_args()
    bundle=args.output.with_suffix('.bundle')
    assemble(Registry(ROOT,args.profile),args.world,bundle)
    args.output.write_bytes((bundle/'world.sdf').read_bytes())
if __name__=='__main__':main()
