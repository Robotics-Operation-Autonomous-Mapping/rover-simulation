"""Run a sensor-owned ROS adapter from a resolved assembly bundle."""
import importlib.util
import json
from pathlib import Path
import sys


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 2: raise ValueError('Expected assembly directory and instance ID')
    directory = Path(args[0]).resolve()
    resolved = json.loads((directory/'resolved.json').read_text())
    record = next(a for a in resolved['adapters'] if a['instance'] == args[1])
    path = (directory/record['path']).resolve()
    if not path.is_relative_to(directory): raise ValueError('Adapter must be inside the assembly')
    spec = importlib.util.spec_from_file_location('roam_sensor_adapter', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = next(i for i in resolved['instances'] if i['id'] == args[1])
    rclpy.init()
    node = None
    try:
        node = module.create_node(instance, resolved)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
