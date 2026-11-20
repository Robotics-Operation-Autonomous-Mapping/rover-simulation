"""Start a previously assembled simulation; does not reload calibration."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from my_robot.runtime import actions


def start(context):
    boolean = lambda name: LaunchConfiguration(name).perform(context).lower() == 'true'
    return actions(LaunchConfiguration('assembly_dir').perform(context),
                   gazebo=boolean('gazebo'), ros=boolean('ros'), control=boolean('control'), view=boolean('view'),
                   headless=boolean('headless'), log_readings=boolean('log_readings'),
                   renderer=LaunchConfiguration('renderer').perform(context))


def generate_launch_description():
    defaults = {'gazebo':'true','ros':'true','control':'false','view':'true','headless':'false','renderer':'ogre2','log_readings':'false'}
    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION',os.environ.get('GZ_PARTITION') or 'roam_rover_sensors'),
        DeclareLaunchArgument('assembly_dir'),
        *[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()],
        OpaqueFunction(function=start),
    ])
