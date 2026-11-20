"""Resolve reusable sensor defaults and instance calibration for assembly."""
from copy import deepcopy
import json
import importlib.util
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def read_json(path):
    path = Path(path)
    text = path.read_text(encoding='utf-8')
    if path.suffix.lower() in ('.yaml', '.yml'):
        import yaml
        return yaml.safe_load(text)
    return json.loads(text)


def component_path(root, name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]+(?:/[A-Za-z0-9_-]+)*', name):
        raise ValueError(f'Invalid component name: {name}')
    path = (Path(root) / name).resolve()
    if not path.is_relative_to(Path(root).resolve()):
        raise ValueError('Component must remain inside its asset directory')
    return path


def set_text(root, path, value):
    element = root.find(path)
    if element is None:
        raise ValueError(f'Unknown SDF setting: {path}')
    element.text = str(value)


def field(data, path):
    for key in path.split('.'):
        data = data[key]
    return data


def pose_values(text):
    values = tuple(map(float, text.split()))
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError(f'Expected six finite pose values: {text}')
    return values


class Registry:
    """One resolved configuration; no ROS imports and no runtime processes."""

    def __init__(self, share, *, rover='atlas', calibration=None, enable=(), disable=()):
        share = Path(share).resolve()
        self.share = share
        self.rover_name = rover
        self.rover_dir = component_path(share / 'rover', rover)
        self.rover = read_json(self.rover_dir / 'definition.json')
        if 'control' in self.rover:
            model = ET.parse(self.rover_dir / self.rover['model']).getroot().find('model')
            for name, path in self.rover['control'].pop('bindings', {}).items():
                value = model.findtext(path)
                if value is None: raise ValueError(f'Missing rover control binding: {path}')
                self.rover['control'][name] = value
        if calibration is None:
            calibration = share / 'calibration/atlas.json'
        calibration = Path(calibration).resolve()
        self.calibration_path = calibration
        self.data = read_json(calibration)
        if self.data.get('version') != 1:
            raise ValueError('Unsupported calibration version')
        self.data['base_frame'] = self.rover['base_frame']
        self.instances = self.data['instances']
        self.definitions = {}
        for instance in self.instances:
            name = instance['implementation']
            definition = read_json(component_path(share / 'sensors', name) / 'definition.json')
            self.definitions[name] = definition
            instance['definition'] = definition['family']
            instance['settings'] = dict(definition.get('settings', {}), **instance.get('settings', {}))
            instance.setdefault('enabled', True)
        ids = {i['id'] for i in self.instances}
        if not set(enable).union(disable) <= ids:
            raise ValueError('Enable/disable refers to an unknown sensor instance')
        if set(enable).intersection(disable):
            raise ValueError('An instance cannot be both enabled and disabled')
        for instance in self.instances:
            if instance['id'] in enable: instance['enabled'] = True
            if instance['id'] in disable: instance['enabled'] = False
        self.enabled = {i['id'] for i in self.instances if i['enabled']}
        self.validate()

    def definition(self, instance):
        return self.definitions[instance['implementation']]

    def by_definition(self, family):
        return [i for i in self.instances if i['definition'] == family]

    def active(self, instance):
        return instance['enabled']

    @staticmethod
    def instance_transforms(instance):
        calibration = instance['calibration']
        result = [(instance['parent'], instance['link'], calibration['mount_pose']),
                  (instance['link'], instance['frame'], calibration['sensor_pose'])]
        if 'optical_frame' in instance:
            result.append((instance['frame'], instance['optical_frame'], calibration['optical_pose']))
        return result

    def transforms(self):
        return [tuple(t) for t in self.rover.get('static_transforms', [])] + [
            t for i in self.instances if self.definition(i)['kind'] == 'gazebo'
            for t in self.instance_transforms(i)]

    def validate(self):
        ids = [i['id'] for i in self.instances]
        if len(ids) != len(set(ids)):
            raise ValueError('Duplicate sensor instance IDs')
        base = ET.parse(self.rover_dir / self.rover['model']).getroot().find('model')
        links = {e.get('name') for e in base.findall('link')}
        joints = {e.get('name') for e in base.findall('joint')}
        if self.data['base_frame'] not in links:
            raise ValueError('Rover base frame must identify a model link')
        frames = set(links)
        for instance in self.instances:
            if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', instance['id']):
                raise ValueError('Instance IDs must be simple identifiers')
            if not isinstance(instance['enabled'], bool):
                raise ValueError('enabled must be a boolean')
            definition = self.definition(instance)
            for key, value in instance['settings'].items():
                if key not in definition.get('settings', {}):
                    raise ValueError(f'Unknown setting {key} in {instance["id"]}')
                if key.endswith('update_rate') or key in ('publish_rate',):
                    if not math.isfinite(float(value)) or float(value) <= 0:
                        raise ValueError(f'Invalid positive rate: {key}')
                if key.endswith('stddev'):
                    if not math.isfinite(float(value)) or float(value) < 0:
                        raise ValueError(f'Invalid noise: {key}')
            if definition.get('requires_truth') and self.active(instance) and not self.rover.get('truth'):
                raise ValueError(f'{instance["id"]} requires a rover truth interface')
            if definition['kind'] == 'gazebo':
                if instance['joint'] in joints:
                    raise ValueError('Duplicate joint name')
                joints.add(instance['joint'])
                for key in ('link', 'frame', 'optical_frame'):
                    if key in instance:
                        if instance[key] in frames:
                            raise ValueError(f'Duplicate frame/link: {instance[key]}')
                        frames.add(instance[key])
                for _, _, pose in self.instance_transforms(instance): pose_values(pose)
                expected = instance['topics']['sensor'] + definition.get('gazebo_output_suffix', '')
                if instance['topics']['data']['gz'] != expected:
                    raise ValueError('Gazebo sensor and output topics disagree')
        parents = {child: parent for parent, child, _ in self.transforms()}
        if len(parents) != len(self.transforms()):
            raise ValueError('Duplicate TF children')
        for child in parents:
            seen = set()
            while child not in links:
                if child in seen or child not in parents:
                    raise ValueError('Disconnected or cyclic sensor TF')
                seen.add(child)
                child = parents[child]
        for instance in self.instances:
            definition = self.definition(instance)
            if self.active(instance) and 'validator' in definition:
                folder = component_path(self.share/'sensors', instance['implementation'])
                path = (folder/definition['validator']).resolve()
                if not path.is_relative_to(folder): raise ValueError('Validator must be inside the component')
                spec = importlib.util.spec_from_file_location('sensor_validation', path)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                module.validate(instance, self)
        for key in ('ros_topic_name', 'gz_topic_name'):
            names = set()
            for mapping in self.bridges():
                topic = mapping.get(key, mapping.get('topic_name'))
                if not isinstance(topic, str) or not topic.startswith('/') or topic in names:
                    raise ValueError(f'Invalid or duplicate topic: {topic}')
                names.add(topic)
        self.model()

    def bridges(self):
        result = deepcopy(self.rover.get('bridges', []))
        for instance in self.instances:
            if not self.active(instance): continue
            definition = self.definition(instance)
            for output, mapping in definition.get('outputs', {}).items():
                topics = instance['topics'][output]
                result.append(dict(mapping, ros_topic_name=topics['ros'], gz_topic_name=topics['gz'],
                                   frame_id=instance.get('optical_frame', instance['frame'])))
            if 'input_bridge' in definition:
                mapping = dict(definition['input_bridge'], topic_name=instance['topics']['input'])
                if 'frame' in instance: mapping['frame_id'] = instance['frame']
                result.append(mapping)
            for extra in definition.get('extra_bridges', []):
                topics = instance['topics'][extra['topics_key']]
                mapping = {k:v for k,v in extra.items() if k not in ('topics_key', 'frame', 'rate')}
                result.append(dict(mapping, ros_topic_name=topics['ros'], gz_topic_name=topics['gz']))
        return result

    def model(self):
        root = ET.parse(self.rover_dir / self.rover['model']).getroot()
        model = root.find('model')
        for instance in self.instances:
            definition = self.definition(instance)
            folder = component_path(self.share / 'sensors', instance['implementation'])
            if definition['kind'] == 'gazebo':
                link = ET.parse(folder / 'sensor.sdf').getroot()
                link.set('name', instance['link'])
                ET.SubElement(link, 'pose', relative_to=instance['parent']).text = instance['calibration']['mount_pose']
                sensor = link.find('sensor')
                sensor.set('name', instance['sensor_name'])
                ET.SubElement(sensor, 'pose').text = instance['calibration']['sensor_pose']
                set_text(sensor, 'topic', instance['topics']['sensor'])
                for path, value in instance['settings'].items(): set_text(sensor, path, value)
                if 'optical_frame' in instance:
                    set_text(sensor, 'camera/optical_frame_id', instance['optical_frame'])
                    set_text(sensor, 'camera/camera_info_topic', instance['topics']['camera_info']['gz'])
                if any(not len(e) and e.text is None for e in sensor.iter()):
                    raise ValueError(f'Incomplete SDF settings: {instance["id"]}')
                if not self.active(instance): link.remove(sensor)
                model.append(link)
                joint = ET.SubElement(model, 'joint', name=instance['joint'], type='fixed')
                ET.SubElement(joint, 'parent').text = instance['parent']
                ET.SubElement(joint, 'child').text = instance['link']
            elif self.active(instance) and (folder / 'plugin.sdf').exists():
                plugin = ET.parse(folder / 'plugin.sdf').getroot()
                for path, source in definition.get('plugin_bindings', {}).items():
                    set_text(plugin, path, field(instance, source))
                for path, source in definition.get('plugin_lists', {}).items():
                    for old in list(plugin.findall(path)): plugin.remove(old)
                    for value in field(instance, source): ET.SubElement(plugin, path).text = value
                model.append(plugin)
        return root

    def outputs(self):
        outputs = []
        for i in self.instances:
            if not self.active(i): continue
            d = self.definition(i)
            for name, mapping in d.get('outputs', {}).items():
                outputs.append({'topic':i['topics'][name]['ros'], 'type':mapping['ros_type_name'],
                                'frame':i.get('optical_frame', i['frame']), 'rate':float(i['settings']['update_rate'])})
            if 'input_bridge' in d:
                outputs.append({'topic':i['topics']['input'], 'type':d['input_bridge']['ros_type_name'],
                                'frame':i['frame'], 'rate':float(i['settings']['update_rate'])})
            for extra in d.get('extra_bridges', []):
                outputs.append({'topic':i['topics'][extra['topics_key']]['ros'], 'type':extra['ros_type_name'],
                                'frame':extra['frame'], 'rate':extra['rate']})
            for output in d.get('derived_outputs', []):
                context = dict(i, rover=self.rover)
                outputs.append({'topic':field(context, output['topic']), 'type':output['ros_type_name'],
                                'frame':field(context, output['frame']), 'rate':float(field(context, output['rate']))})
        return outputs


def write_model(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
