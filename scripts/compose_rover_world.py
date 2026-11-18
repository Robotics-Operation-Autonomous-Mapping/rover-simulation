#!/usr/bin/env python3
"""Compatibility API and CLI for world composition."""
from pathlib import Path
from runpy import run_path

_implementation = run_path(str(Path(__file__).resolve().parent / 'launch/compose_rover_world.py'))
compose_world = _implementation['compose_world']
SPAWN_POSES = _implementation['SPAWN_POSES']

if __name__ == '__main__':
    _implementation['main']()
