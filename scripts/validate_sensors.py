#!/usr/bin/env python3
"""Check a running simulation's measurement types, frames, stamps, data and rates."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/my_robot'))
from my_robot.registry import Registry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='atlas')
    parser.add_argument('--rover', default='atlas')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--assembly', type=Path)
    parser.add_argument('--enable', action='append', default=[])
    parser.add_argument('--disable', action='append', default=[])
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--min-rate-ratio', type=float, default=0.5)
    parser.add_argument('--print-topics', action='store_true', help='Print bag topics without requiring ROS')
    args = parser.parse_args()
    if args.assembly:
        resolved = json.loads((args.assembly/'resolved.json').read_text())
        contracts, instances = resolved['outputs'], resolved['instances']
        base_frame = resolved['rover']['base_frame']
    else:
        registry = Registry(ROOT, args.profile, rover=args.rover, calibration=args.calibration,
                            enable=args.enable, disable=args.disable)
        contracts, instances = registry.outputs(), registry.instances
        base_frame = registry.data['base_frame']
    if args.print_topics:
        print('\n'.join(['/clock', '/tf', '/tf_static'] + [o['topic'] for o in contracts]))
        return
    if args.duration <= 0 or not 0 < args.min_rate_ratio <= 1:
        parser.error('Duration must be positive and rate ratio must be in (0, 1]')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('roam_sensor_validation', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {o['topic']: [] for o in contracts}
    errors = set()
    notes = set()

    def receive(contract, message):
        topic = contract['topic']
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        stamps = samples[topic]
        if stamp <= 0 or (stamps and stamp < stamps[-1]):
            errors.add(f'{topic}: zero/backwards measurement timestamp (restart validation after a world reset)')
        if message.header.frame_id != contract['frame']:
            errors.add(f"{topic}: frame {message.header.frame_id!r}, expected {contract['frame']!r}")
        stamps.append(stamp)
        kind = contract['type'].rsplit('/', 1)[-1]
        if kind == 'PointCloud2':
            fields = {f.name for f in message.fields}
            if not {'x', 'y', 'z'} <= fields or not message.data:
                errors.add(f'{topic}: empty cloud or missing XYZ fields')
            if not {'intensity'} <= fields:
                notes.add(f'{topic}: intensity field unavailable')
            if not fields.intersection({'time', 't', 'timestamp'}):
                notes.add(f'{topic}: no per-point timing; unsuitable for hardware-equivalent deskew validation')
        elif kind == 'Image' and (not message.data or not message.width or not message.height):
            errors.add(f'{topic}: empty image')
        elif kind == 'CameraInfo' and (message.k[0] <= 0 or message.k[4] <= 0):
            errors.add(f'{topic}: invalid camera intrinsics')
        elif kind == 'NavSatFix' and not all(math.isfinite(v) for v in (message.latitude, message.longitude, message.altitude)):
            errors.add(f'{topic}: non-finite GNSS fix')
        elif kind == 'JointState' and (len(message.name) != 4 or len(message.position) != 4 or len(message.velocity) != 4):
            errors.add(f'{topic}: expected four wheel joint measurements')

    subscriptions = [node.create_subscription(get_message(o['type']), o['topic'],
                     lambda msg, contract=o: receive(contract, msg), qos_profile_sensor_data) for o in contracts]
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        advertised = dict(node.get_topic_names_and_types())
        for output in contracts:
            topic = output['topic']
            stamps = samples[topic]
            if output['type'] not in advertised.get(topic, []):
                errors.add(f'{topic}: required message type not advertised')
            if len(stamps) < 2 or stamps[-1] <= stamps[0]:
                errors.add(f'{topic}: insufficient advancing samples')
                continue
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
            print(f"{topic}: {rate:.1f} Hz simulation time; configured {output['rate']:g} Hz; {len(stamps)} samples")
            if rate < output['rate'] * args.min_rate_ratio:
                errors.add(f'{topic}: below configured rate tolerance')
        for instance in instances:
            if instance['enabled'] and 'calibration' in instance:
                frame = instance.get('optical_frame', instance['frame'])
                if not buffer.can_transform(base_frame, frame, rclpy.time.Time()):
                    errors.add(f'Missing TF: {base_frame} -> {frame}')
        if node.get_clock().now().nanoseconds <= 0:
            errors.add('Simulation clock did not advance')
        for note in sorted(notes):
            print('LIMITATION:', note)
        for error in sorted(errors):
            print('FAIL:', error)
        print(f"{'FAIL' if errors else 'PASS'}: {len(contracts)} measurement topic contracts")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
