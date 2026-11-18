"""ROS launch actions consuming one immutable assembly result."""
import json
from pathlib import Path
import sys


def actions(directory, *, gazebo=True, ros=True, control=False, view=True, headless=False, renderer='ogre2', log_readings=False):
    from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    directory = Path(directory).resolve()
    resolved = json.loads((directory/'resolved.json').read_text())
    result = []
    critical = []
    if gazebo:
        sim = ExecuteProcess(cmd=['gz','sim','-r',str(directory/'world.sdf'),'--render-engine',renderer]+(['-s'] if headless else []),output='screen')
        result.extend([sim, RegisterEventHandler(OnProcessExit(target_action=sim,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))])
    if ros or control:
        result.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='rover_sensor_bridge',output='screen',
                           parameters=[{'config_file':str(directory/('bridge.yaml' if ros else 'command_bridge.yaml')),'use_sim_time':True}]))
        critical.append(result[-1])
    if ros:
        result.append(Node(package='my_robot',executable='rover_sensor_state',output='screen',
                           parameters=[{'assembly_dir':str(directory),'use_sim_time':True}]))
        critical.append(result[-1])
        for adapter in resolved['adapters']:
            result.append(Node(package='my_robot',executable='sensor_adapter',output='screen',
                               arguments=[str(directory),adapter['instance']],parameters=[{'use_sim_time':True}]))
            critical.append(result[-1])
        if log_readings:
            for instance in resolved['instances']:
                if instance['enabled'] and instance['definition']=='wheel_encoders':
                    result.append(Node(package='my_robot',executable='wheel_encoder_subscriber',output='screen',
                                       parameters=[{'use_sim_time':True}], remappings=[('/rover/wheel_encoders',instance['topics']['output'])]))
        if view and not headless:
            result.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(directory/'sensors.rviz')],parameters=[{'use_sim_time':True}]))
    if control and not headless:
        settings = resolved['rover'].get('control')
        if settings is None: raise ValueError('Selected rover has no keyboard control implementation')
        path = directory/'assets'/resolved['rover_name']/'control/keyboard_drive.py'
        result.append(ExecuteProcess(cmd=[sys.executable,str(path),'--no-bridge','--command-topic',settings['command_topic'],
                                          '--wheel-separation',str(settings['wheel_separation']),'--max-speed',str(settings['max_speed'])],output='screen'))
    for process in critical:
        result.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[
            EmitEvent(event=Shutdown(reason='Required ROS process exited'))])))
    return result
