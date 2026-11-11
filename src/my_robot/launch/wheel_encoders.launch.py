"""Bridge Gazebo wheel measurements and run the ROS encoder nodes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_PARTITION', os.environ.get('GZ_PARTITION') or 'roam_rover_wheels'
        ),
        DeclareLaunchArgument(
            'log_readings', default_value='true',
            description='Start the encoder subscriber to log readings.',
        ),
        DeclareLaunchArgument(
            'angle_noise_stddev', default_value='0.001',
            description='Encoder angle Gaussian noise standard deviation in rad; 0 disables.',
        ),
        DeclareLaunchArgument(
            'velocity_noise_stddev', default_value='0.01',
            description='Encoder speed Gaussian noise standard deviation in rad/s; 0 disables.',
        ),
        DeclareLaunchArgument(
            'noise_seed', default_value='-1',
            description='Random seed: -1 for random, nonnegative for repeatable samples.',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='wheel_encoder_bridge', output='screen',
            arguments=[
                '/rover/wheel_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
        ),
        Node(
            package='my_robot', executable='wheel_encoder_publisher',
            output='screen', parameters=[{
                'use_sim_time': True,
                'angle_noise_stddev': ParameterValue(
                    LaunchConfiguration('angle_noise_stddev'), value_type=float,
                ),
                'velocity_noise_stddev': ParameterValue(
                    LaunchConfiguration('velocity_noise_stddev'), value_type=float,
                ),
                'noise_seed': ParameterValue(
                    LaunchConfiguration('noise_seed'), value_type=int,
                ),
            }],
        ),
        Node(
            package='my_robot', executable='wheel_encoder_subscriber',
            output='screen', parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('log_readings')),
        ),
    ])
