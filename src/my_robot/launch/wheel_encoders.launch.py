"""Bridge Gazebo wheel measurements and run the ROS encoder nodes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_PARTITION', os.environ.get('GZ_PARTITION') or 'roam_rover_wheels'
        ),
        DeclareLaunchArgument(
            'log_readings', default_value='true',
            description='Start the encoder subscriber to log readings.',
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
            output='screen', parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='my_robot', executable='wheel_encoder_subscriber',
            output='screen', parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('log_readings')),
        ),
    ])
