"""Check the encoder's boundary with bridged JointState messages."""

from types import SimpleNamespace
from unittest.mock import Mock
import statistics

import pytest
from sensor_msgs.msg import JointState

from my_robot.wheel_encoder_publisher import EncoderNoise, WheelEncoderPublisher


WHEELS = [
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
]


def convert(msg, noise=None):
    # Exercise the callback without starting DDS or needing a running simulator.
    publisher = Mock()
    logger = Mock()
    node = SimpleNamespace(
        encoder_publisher=publisher, get_logger=lambda: logger,
        noise=noise if noise is not None else EncoderNoise(0.0, 0.0),
    )
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


def test_noise_is_repeatable_without_modifying_input():
    angles, velocities = [1.0] * 4, [2.0] * 4
    first = EncoderNoise(seed=42)
    second = EncoderNoise(seed=42)
    sequence = [first.apply(angles, velocities) for _ in range(3)]
    assert sequence == [second.apply(angles, velocities) for _ in range(3)]
    assert sequence[0] != sequence[1]
    assert sequence[0] != EncoderNoise(seed=43).apply(angles, velocities)
    assert angles == [1.0] * 4
    assert velocities == [2.0] * 4


@pytest.mark.parametrize('angle_stddev,velocity_stddev', [(0.0, 0.1), (0.1, 0.0)])
def test_each_noise_channel_can_be_disabled(angle_stddev, velocity_stddev):
    angles, velocities = EncoderNoise(angle_stddev, velocity_stddev, 42).apply(
        [1.0] * 4, [2.0] * 4,
    )
    assert (angles == [1.0] * 4) == (angle_stddev == 0.0)
    assert (velocities == [2.0] * 4) == (velocity_stddev == 0.0)


def test_noise_has_configured_statistics():
    noise = EncoderNoise(0.02, 0.2, 42)
    angles, velocities = noise.apply([0.0] * 20000, [0.0] * 20000)
    for samples, stddev in ((angles, 0.02), (velocities, 0.2)):
        assert abs(statistics.mean(samples)) < stddev * 0.03
        assert statistics.stdev(samples) == pytest.approx(stddev, rel=0.03)


@pytest.mark.parametrize('value', [-0.1, float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('parameter', ['angle_stddev', 'velocity_stddev'])
def test_rejects_invalid_noise_standard_deviation(parameter, value):
    with pytest.raises(ValueError, match='finite and nonnegative'):
        EncoderNoise(**{parameter: value})


def test_rejects_invalid_seed():
    with pytest.raises(ValueError, match='noise_seed'):
        EncoderNoise(seed=-2)


def test_callback_adds_noise_and_preserves_measurement_metadata():
    msg = JointState()
    msg.name = list(reversed(WHEELS))
    msg.position = [4.0, 3.0, 2.0, 1.0]
    msg.velocity = [40.0, 30.0, 20.0, 10.0]
    msg.header.stamp.sec = 42
    msg.header.frame_id = 'basic_rover'

    publisher, logger = convert(msg, EncoderNoise(seed=42))
    encoders = publisher.publish.call_args.args[0]
    assert encoders.header == msg.header
    assert encoders.wheel_names == WHEELS
    assert list(encoders.angle_rad) != [1.0, 2.0, 3.0, 4.0]
    assert list(encoders.velocity_rad_s) != [10.0, 20.0, 30.0, 40.0]
    # Noise is small enough that the wheel order is still identifiable.
    assert list(encoders.angle_rad) == pytest.approx([1, 2, 3, 4], abs=0.01)
    assert list(encoders.velocity_rad_s) == pytest.approx([10, 20, 30, 40], abs=0.1)
    assert list(msg.position) == [4.0, 3.0, 2.0, 1.0]
    logger.warning.assert_not_called()


def test_invalid_measurements_do_not_consume_noise_samples():
    noise = EncoderNoise(seed=42)
    msg = JointState()
    msg.name = WHEELS.copy()
    msg.position = [float('nan')] * 4
    msg.velocity = [0.0] * 4
    publisher, _ = convert(msg, noise)
    publisher.publish.assert_not_called()
    assert noise.apply([0.0] * 4, [0.0] * 4) == EncoderNoise(seed=42).apply(
        [0.0] * 4, [0.0] * 4,
    )
