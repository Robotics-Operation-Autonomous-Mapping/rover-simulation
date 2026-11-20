#!/usr/bin/env python3
"""Assemble a simulation, then delegate process lifetime to ROS launch."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
from assemble import ROOT, arguments
from my_robot.registry import Registry
from my_robot.assembly import assemble


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    arguments(parser)
    parser.add_argument('--control', action='store_true')
    parser.add_argument('--no-ros', dest='no_ros', action='store_true')
    parser.add_argument('--no-view', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--render-engine', choices=['ogre','ogre2'])
    parser.add_argument('--output', type=Path, help='Keep generated artifacts in an empty directory')
    parser.add_argument('--assemble-only', action='store_true')
    args = parser.parse_args()
    if args.assemble_only and args.output is None: parser.error('--assemble-only requires --output')
    try:
        registry = Registry(ROOT,rover=args.rover,calibration=args.calibration,enable=args.enable,disable=args.disable)
        with tempfile.TemporaryDirectory(prefix='roam-run-') as temporary:
            directory = args.output or Path(temporary)
            assemble(registry,args.world,directory,args.spawn_pose)
            print(f'Assembly: {directory.resolve()}',flush=True)
            if args.assemble_only: return 0
            renderer = args.render_engine or 'ogre2'
            if not args.render_engine:
                try:
                    if subprocess.run(['systemd-detect-virt','-q'],capture_output=True).returncode == 0: renderer='ogre'
                except FileNotFoundError: pass
            environment = os.environ.copy()
            environment.setdefault('GZ_PARTITION','roam_rover_sensors')
            if environment.get('XDG_SESSION_TYPE')=='wayland': environment['QT_QPA_PLATFORM']='xcb'
            command = ['ros2','launch','my_robot','simulation.launch.py',f'assembly_dir:={directory.resolve()}',
                       f'ros:={str(not args.no_ros).lower()}',f'control:={str(args.control and not args.headless).lower()}',
                       f'view:={str(not args.no_view).lower()}',f'headless:={str(args.headless).lower()}',f'renderer:={renderer}']
            process = subprocess.Popen(command,env=environment)
            try:
                return process.wait()
            except KeyboardInterrupt:
                # ROS launch receives the same terminal SIGINT. Keep bundle assets
                # alive until it has finished shutting down its child processes.
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                return 130
    except (ValueError,KeyError,OSError) as error:
        parser.exit(2,f'Unable to run: {error}\nSource ROS Jazzy and install/setup.bash before running.\n')


if __name__=='__main__': raise SystemExit(main())
