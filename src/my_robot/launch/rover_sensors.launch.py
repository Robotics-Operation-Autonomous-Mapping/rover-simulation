"""Compatibility ROS-only entry point; prefer a shared assembly_dir."""
import os
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from my_robot.registry import Registry
from my_robot.assembly import assemble
from my_robot.runtime import actions


def start(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    directory=value('assembly_dir')
    cleanup=[]
    if not directory:
        temporary=tempfile.TemporaryDirectory(prefix='roam-ros-')
        directory=temporary.name
        registry=Registry(value('registry_share'),value('profile'),rover=value('rover'),calibration=value('calibration') or None)
        assemble(registry,'road',directory)
        def release(context):
            temporary.cleanup()
            return []
        cleanup=[RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=release)]))]
    return cleanup+actions(directory,gazebo=False,view=value('rviz').lower()=='true',log_readings=value('log_readings').lower()=='true')


def generate_launch_description():
    defaults={'profile':'default','rover':'atlas','calibration':'','assembly_dir':'',
              'registry_share':get_package_share_directory('my_robot'),'rviz':'true','log_readings':'false'}
    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION',os.environ.get('GZ_PARTITION') or 'roam_rover_sensors'),
        *[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()],
        OpaqueFunction(function=start),
    ])
