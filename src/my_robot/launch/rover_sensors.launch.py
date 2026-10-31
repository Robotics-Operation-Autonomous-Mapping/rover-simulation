"""Bridge all rover sensors, publish frames and synthetic INS, and open RViz."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('my_robot')) / 'config'
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_PARTITION', os.environ.get('GZ_PARTITION') or 'roam_rover_sensors'
        ),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open the camera and LiDAR sensor view.'),
        DeclareLaunchArgument('log_readings', default_value='false',
                              description='Log wheel encoder readings.'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='rover_sensor_bridge', output='screen',
            parameters=[{'config_file': str(config / 'rover_bridge.yaml'),
                         'use_sim_time': True}],
        ),
        Node(
            package='my_robot', executable='rover_sensor_state', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='my_robot', executable='wheel_encoder_publisher', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='my_robot', executable='wheel_encoder_subscriber', output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('log_readings')),
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', str(config / 'rover_sensors.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
