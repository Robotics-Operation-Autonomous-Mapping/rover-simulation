"""Check the encoder's boundary with bridged JointState messages."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from sensor_msgs.msg import JointState

from my_robot.wheel_encoder_publisher import WheelEncoderPublisher


WHEELS = [
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
]


def convert(msg):
    # Exercise the callback without starting DDS or needing a running simulator.
    publisher = Mock()
    logger = Mock()
    node = SimpleNamespace(encoder_publisher=publisher, get_logger=lambda: logger,
                           last_publish_stamp_ns=None, publish_period_ns=20_000_000)
    WheelEncoderPublisher.joint_state_callback(node, msg)
    return publisher, logger


def test_reorders_wheels_and_preserves_simulation_header():
    msg = JointState()
    msg.name = [WHEELS[2], 'extra_joint', WHEELS[0], WHEELS[3], WHEELS[1]]
    msg.position = [3.0, 99.0, 1.0, 4.0, 2.0]
    msg.velocity = [30.0, 99.0, 10.0, 40.0, 20.0]
    msg.header.stamp.sec = 42
    msg.header.stamp.nanosec = 123
    msg.header.frame_id = 'basic_rover'

    publisher, logger = convert(msg)

    publisher.publish.assert_called_once()
    encoders = publisher.publish.call_args.args[0]
    assert encoders.wheel_names == WHEELS
    assert list(encoders.angle_rad) == [1.0, 2.0, 3.0, 4.0]
    assert list(encoders.velocity_rad_s) == [10.0, 20.0, 30.0, 40.0]
    assert encoders.header == msg.header
    logger.warning.assert_not_called()


@pytest.mark.parametrize('case', [
    'missing_position', 'missing_velocity', 'missing_wheel', 'duplicate_wheel',
    'nan_angle', 'infinite_velocity',
])
def test_rejects_incomplete_or_invalid_measurements(case):
    msg = JointState()
    msg.name = WHEELS.copy()
    msg.position = [0.0] * 4
    msg.velocity = [0.0] * 4
    if case == 'missing_position':
        msg.position = []
    elif case == 'missing_velocity':
        msg.velocity = []
    elif case == 'missing_wheel':
        msg.name[0] = 'other_joint'
    elif case == 'duplicate_wheel':
        msg.name.append(WHEELS[0])
        msg.position.append(0.0)
        msg.velocity.append(0.0)
    elif case == 'nan_angle':
        msg.position[0] = float('nan')
    elif case == 'infinite_velocity':
        msg.velocity[0] = float('inf')

    publisher, logger = convert(msg)

    publisher.publish.assert_not_called()
    logger.warning.assert_called_once()


def test_throttles_by_simulation_timestamp_and_recovers_after_reset():
    publisher = Mock()
    node = SimpleNamespace(
        encoder_publisher=publisher, get_logger=Mock(),
        publish_period_ns=20_000_000, last_publish_stamp_ns=None,
    )
    msg = JointState()
    msg.name = WHEELS.copy()
    msg.position = [0.0] * 4
    msg.velocity = [1.0] * 4
    for stamp in [1_000_000, 2_000_000, 20_000_000, 21_000_000]:
        msg.header.stamp.nanosec = stamp
        WheelEncoderPublisher.joint_state_callback(node, msg)
    assert publisher.publish.call_count == 2
    # A world reset sends time backwards; output should resume immediately.
    msg.header.stamp.nanosec = 0
    WheelEncoderPublisher.joint_state_callback(node, msg)
    assert publisher.publish.call_count == 3
