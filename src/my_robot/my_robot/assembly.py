"""Generate an inspectable runtime bundle before starting any processes."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from my_robot.registry import Registry, read_json, component_path, pose_values, write_model
from my_robot.visualization import write_rviz

SYSTEMS = {'Physics':'physics', 'UserCommands':'user-commands', 'SceneBroadcaster':'scene-broadcaster',
           'Sensors':'sensors', 'Imu':'imu', 'NavSat':'navsat', 'Magnetometer':'magnetometer'}


def compose_world(registry, world_name, spawn_pose=None):
    path = component_path(registry.share/'worlds', world_name)
    source = path.with_suffix('.sdf')
    root = ET.fromstring(source.read_text(encoding='utf-8').lstrip())
    world = root.find('world')
    if world is None: raise ValueError('SDF must contain a world')
    metadata = read_json(path.with_suffix('.json')) if path.with_suffix('.json').exists() else {}
    pose = spawn_pose or metadata.get('spawn_pose')
    if pose is None: raise ValueError('Supply --spawn-pose or world spawn metadata')
    pose_values(pose)
    model = registry.model().find('model')
    name = model.get('name')
    if any(m.get('name') == name for m in world.findall('model')):
        raise ValueError(f'World already contains a model named {name}')
    ET.SubElement(model, 'pose').text = pose
    world.append(model)
    systems = {'Physics', 'UserCommands', 'SceneBroadcaster'}
    for instance in registry.instances:
        if registry.active(instance): systems.update(registry.definition(instance).get('required_systems', []))
    for system in sorted(systems):
        if system not in SYSTEMS: raise ValueError(f'Unsupported world system: {system}')
        name = 'gz::sim::systems::'+system
        plugin = next((p for p in world.findall('plugin') if p.get('name') == name), None)
        if plugin is None:
            plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{SYSTEMS[system]}-system', name=name)
        if system == 'Sensors':
            engine = plugin.find('render_engine')
            if engine is None: engine = ET.SubElement(plugin, 'render_engine')
            engine.text = 'ogre2'
    if world.find('spherical_coordinates') is None and 'spherical_coordinates' in metadata:
        coordinates = ET.SubElement(world, 'spherical_coordinates')
        for key, value in metadata['spherical_coordinates'].items(): ET.SubElement(coordinates, key).text = str(value)
    if world.find('magnetic_field') is None and 'magnetic_field' in metadata:
        ET.SubElement(world, 'magnetic_field').text = metadata['magnetic_field']
    return root


def assemble(registry, world_name, output, spawn_pose=None):
    output = Path(output).resolve()
    # Reject nonempty output directories to avoid stale or overwritten run artifacts.
    if output.exists() and any(output.iterdir()):
        raise ValueError(f'Assembly output must be empty: {output}')
    world = compose_world(registry, world_name, spawn_pose)
    output.mkdir(parents=True, exist_ok=True)
    rover_assets = output/'assets'/registry.rover_name
    shutil.copytree(registry.rover_dir, rover_assets)
    model = world.find(f"world/model[@name='{registry.model().find('model').get('name')}']")
    # Resolve component-local meshes into copied assets. Global model:// URIs are retained.
    for link in model.findall('link'):
        instance = next((i for i in registry.instances if i.get('link') == link.get('name')), None)
        asset_dir = rover_assets
        if instance:
            source = component_path(registry.share/'sensors', instance['implementation'])
            asset_dir = output/'assets'/'sensors'/instance['id']
            shutil.copytree(source, asset_dir)
        for uri in link.findall('.//mesh/uri'):
            if '://' not in uri.text:
                asset = (asset_dir/uri.text).resolve()
                if not asset.is_relative_to(asset_dir.resolve()) or not asset.is_file():
                    raise ValueError(f'Missing or escaping mesh: {uri.text}')
                uri.text = asset.as_uri()
    adapters = []
    for instance in registry.instances:
        definition = registry.definition(instance)
        if not registry.active(instance) or 'adapter' not in definition: continue
        source = component_path(registry.share/'sensors', instance['implementation'])
        destination = output/'adapters'/instance['id']
        shutil.copytree(source, destination)
        adapter = (destination/definition['adapter']).resolve()
        if not adapter.is_relative_to(destination.resolve()) or not adapter.is_file():
            raise ValueError('Adapter must be a file within its sensor component')
        adapters.append({'instance':instance['id'], 'path':str(adapter.relative_to(output))})
    resolved = {'version':1, 'rover':registry.rover, 'rover_name':registry.rover_name,
                'calibration_source':str(registry.calibration_path), 'world':world_name,
                'instances':registry.instances, 'transforms':registry.transforms(),
                'bridges':registry.bridges(), 'outputs':registry.outputs(), 'adapters':adapters}
    for name, data in [('resolved.json',resolved), ('bridge.yaml',resolved['bridges']), ('tf.json',resolved['transforms'])]:
        (output/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    commands = [m for m in resolved['bridges'] if m.get('direction') == 'ROS_TO_GZ']
    (output/'command_bridge.yaml').write_text(json.dumps(commands,indent=2)+'\n',encoding='utf-8')
    write_model(world, output/'world.sdf')
    single = ET.Element('sdf',version='1.10')
    single.append(deepcopy(model))
    write_model(single, output/'model.sdf')
    write_rviz(registry, output/'sensors.rviz')
    return resolved
