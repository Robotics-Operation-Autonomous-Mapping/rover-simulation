"""Check sensor wiring, frame conventions, and synthetic INS behavior."""

from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
import pytest
import yaml

from my_robot.rover_state import sensor_transforms, synthetic_ins


WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / 'models/roam_rover/model.sdf'


def test_every_sensor_bridge_has_a_connected_frame_and_a_matching_gazebo_topic():
    model = ET.fromstring(MODEL.read_text().lstrip()).find('model')
    sensors = model.findall('link/sensor')
    transforms = sensor_transforms(MODEL)
    parents = {t.child_frame_id: t.header.frame_id for t in transforms}
    assert len(parents) == len(transforms), 'Duplicate TF child frames'
    bridge = yaml.safe_load((WORKSPACE / 'src/my_robot/config/rover_bridge.yaml').read_text())
    for sensor in sensors:
        topic = sensor.findtext('topic')
        if sensor.get('type') == 'gpu_lidar':
            topic += '/points'
        mapping = next(b for b in bridge
                       if b.get('gz_topic_name', b.get('topic_name')) == topic)
        frame = mapping['frame_id']
        visited = set()
        while frame != 'base_link':
            assert frame not in visited, 'Cycle in sensor TF tree'
            visited.add(frame)
            frame = parents[frame]
    camera = next(t for t in transforms if t.child_frame_id == 'camera_frame')
    lidar = next(t for t in transforms if t.child_frame_id == 'lidar_frame')
    assert camera.transform.translation.x == pytest.approx(0.045)
    assert lidar.transform.translation.z == pytest.approx(0.03)


def test_camera_optical_axes_are_right_down_forward():
    transform = next(t for t in sensor_transforms(MODEL)
                     if t.child_frame_id == 'camera_optical_frame')
    q = transform.transform.rotation
    # Third column of the quaternion rotation matrix: optical +Z -> body +X.
    assert 2 * (q.x * q.z + q.w * q.y) == pytest.approx(1)
    assert 2 * (q.y * q.z - q.w * q.x) == pytest.approx(0)
    assert 1 - 2 * (q.x * q.x + q.y * q.y) == pytest.approx(0)
    assert math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) == pytest.approx(1)


def test_ins_keeps_timestamp_frames_and_motion_without_modifying_truth():
    truth = Odometry()
    truth.header.frame_id = 'world'
    truth.child_frame_id = 'base_link'
    truth.header.stamp.sec = 123
    truth.header.stamp.nanosec = 456
    truth.pose.pose.position.x = 12.0
    truth.pose.pose.orientation.w = 1.0
    truth.twist.twist.linear.x = 0.4
    ins = synthetic_ins(truth, 0.005, sample_noise=lambda mean, stddev: stddev)
    assert ins.header == truth.header
    assert ins.child_frame_id == truth.child_frame_id
    assert ins.pose.pose.position.x == pytest.approx(12.005)
    assert truth.pose.pose.position.x == 12.0
    assert truth.pose.covariance[0] == 0
    assert ins.pose.covariance[0] == pytest.approx(0.005**2)
    assert ins.pose.pose.orientation == truth.pose.pose.orientation
    assert ins.twist == truth.twist


@pytest.mark.parametrize('world_name', ['road', 'maze', 'forest', 'roam_test'])
def test_world_composition_preserves_terrain_and_installs_required_sensor_systems(world_name):
    path = WORKSPACE / 'scripts/compose_rover_world.py'
    spec = spec_from_file_location('compose_rover_world', path)
    composer = module_from_spec(spec)
    spec.loader.exec_module(composer)
    source_path = WORKSPACE / 'worlds' / f'{world_name}.sdf'
    original = source_path.read_bytes()
    source = ET.fromstring(original.decode().lstrip()).find('world')
    world = composer.compose_world(WORKSPACE, world_name).find('world')
    assert source_path.read_bytes() == original
    assert [ET.tostring(m) for m in world.findall('model')] == [
        ET.tostring(m) for m in source.findall('model')]
    assert len([i for i in world.findall('include')
                if i.findtext('uri') == 'model://roam_rover']) == 1
    systems = {p.get('name') for p in world.findall('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::Sensors',
            'gz::sim::systems::Imu', 'gz::sim::systems::NavSat'} <= systems
    assert world.findtext('spherical_coordinates/world_frame_orientation') == 'ENU'
