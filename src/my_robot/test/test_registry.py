"""Portable regression and modular assembly tests; no ROS or Gazebo required."""
from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

SHARE=Path(__file__).resolve().parents[1]
ROOT=SHARE.parents[1]
sys.path.insert(0,str(SHARE))
from my_robot.registry import Registry
from my_robot.assembly import assemble, compose_world
from my_robot.asset_install import asset_data_files
from my_robot.kinematics import antenna_position, VelocitySampler
from my_robot.visualization import write_rviz


def canonical(element):
    text=(element.text or '').strip()
    if element.tag=='uri' and '/meshes/' in text: text='meshes/'+text.split('/meshes/')[-1]
    children=[canonical(child) for child in element if not (
        element.tag=='sensor' and child.tag=='pose' and all(float(v)==0 for v in child.text.split()))]
    return element.tag,tuple(sorted(element.attrib.items())),text,tuple(sorted(children))


def normalize(mappings):
    result=[]
    for mapping in mappings:
        mapping=mapping.copy()
        if 'topic_name' in mapping:
            mapping['ros_topic_name']=mapping['gz_topic_name']=mapping.pop('topic_name')
        result.append(mapping)
    return sorted(result,key=lambda m:m['ros_topic_name'])


class RegistryTests(unittest.TestCase):
    def test_original_model_and_bridge_are_preserved(self):
        before=ET.fromstring((SHARE/'test/fixtures/original_rover.sdf').read_text().lstrip())
        registry=Registry(ROOT)
        self.assertEqual(canonical(before),canonical(registry.model()))
        bridge=json.loads((SHARE/'test/fixtures/original_bridge.json').read_text())
        self.assertEqual(normalize(bridge),normalize(registry.bridges()))

    def test_atlas_topic_contract(self):
        registry=Registry(ROOT,'atlas')
        actual={o['topic']:o['type'] for o in registry.outputs()}
        expected={'/rover/lidar/points':'sensor_msgs/msg/PointCloud2',
                  '/rover/lidar/imu':'sensor_msgs/msg/Imu', '/rover/imu/data':'sensor_msgs/msg/Imu',
                  '/rover/imu/mag':'sensor_msgs/msg/MagneticField','/rover/gnss/fix':'sensor_msgs/msg/NavSatFix',
                  '/rover/gnss/velocity':'geometry_msgs/msg/TwistWithCovarianceStamped',
                  '/rover/wheel/joint_states':'sensor_msgs/msg/JointState','/rover/wheel/odometry':'nav_msgs/msg/Odometry'}
        for name in ('front_left','front_right','left','right','rear'):
            expected[f'/rover/camera/{name}/image_raw']='sensor_msgs/msg/Image'
            expected[f'/rover/camera/{name}/camera_info']='sensor_msgs/msg/CameraInfo'
        self.assertEqual(expected,actual)
        self.assertFalse(registry.by_definition('ins'))
        self.assertEqual(len(registry.by_definition('camera')),5)

    def test_every_profile_and_world_preserves_terrain(self):
        for profile in ('default','atlas','camera','lidar','localization'):
            registry=Registry(ROOT,profile)
            for name in ('road','forest','maze','roam_test'):
                with self.subTest(profile=profile,world=name):
                    path=ROOT/'worlds'/f'{name}.sdf'
                    before=path.read_bytes()
                    source=ET.fromstring(before.decode().lstrip()).find('world')
                    world=compose_world(registry,name).find('world')
                    self.assertEqual(before,path.read_bytes())
                    self.assertEqual([canonical(m) for m in source.findall('model')],
                                     [canonical(m) for m in world.findall('model') if m.get('name')!='roam_rover'])
                    self.assertEqual(len(world.findall("model[@name='roam_rover']")),1)

    def test_disable_keeps_mass_but_removes_measurements_and_view(self):
        registry=Registry(ROOT,'atlas',disable=['camera_front_left'])
        model=registry.model().find('model')
        link=model.find("link[@name='camera_front_left_link']")
        self.assertIsNotNone(link.find('inertial'))
        self.assertIsNone(link.find('sensor'))
        self.assertNotIn('/rover/camera/front_left/image_raw',[o['topic'] for o in registry.outputs()])
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'view.rviz'
            write_rviz(registry,path)
            self.assertNotIn('/rover/camera/front_left/image_raw',path.read_text())

    def test_calibration_overrides_do_not_modify_sensor_defaults(self):
        data=json.loads((ROOT/'calibration/legacy.json').read_text())
        data['instances'][0]['settings']['update_rate']='15'
        before=(ROOT/'sensors/camera/pinhole_rgb/definition.json').read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'calibration.json'; path.write_text(json.dumps(data))
            registry=Registry(ROOT,calibration=path)
            self.assertEqual(registry.model().findtext("model/link[@name='camera_link']/sensor/update_rate"),'15')
        self.assertEqual(before,(ROOT/'sensors/camera/pinhole_rgb/definition.json').read_bytes())
        self.assertEqual(Registry(ROOT).instances[0]['settings']['update_rate'],'30')

    def test_duplicate_camera_uses_same_reusable_definition(self):
        data=json.loads((ROOT/'calibration/legacy.json').read_text())
        camera=deepcopy(data['instances'][0])
        for key in ('id','link','joint','sensor_name','frame','optical_frame'): camera[key]+='_second'
        camera['topics']['sensor']+='_second'
        for key in ('data','camera_info'):
            camera['topics'][key]={k:v+'_second' for k,v in camera['topics'][key].items()}
        camera['calibration']['mount_pose']='0.3 0.2 0.18 0 0 0'
        data['instances'].append(camera)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'calibration.json'; path.write_text(json.dumps(data))
            registry=Registry(ROOT,calibration=path)
            self.assertEqual(len(registry.by_definition('camera')),2)
            self.assertIn(('base_link','camera_link_second',camera['calibration']['mount_pose']),registry.transforms())

    def test_bad_calibration_is_rejected(self):
        for failure in ('duplicate','cycle','nan','rate','topic','unknown_setting','enabled','implementation'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as temp:
                data=json.loads((ROOT/'calibration/legacy.json').read_text())
                first,second=data['instances'][:2]
                if failure=='duplicate': second['frame']=first['frame']
                if failure=='cycle': first['parent']=first['frame']
                if failure=='nan': first['calibration']['mount_pose']='nan 0 0 0 0 0'
                if failure=='rate': first['settings']['update_rate']='-1'
                if failure=='topic': first['topics']['data']['gz']='/wrong'
                if failure=='unknown_setting': first['settings']['typo']='123'
                if failure=='enabled': first['enabled']='yes'
                if failure=='implementation': first['implementation']='../../other'
                path=Path(temp)/'calibration.json'; path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): Registry(ROOT,calibration=path)

    def test_no_mandatory_ins_or_encoders_and_another_rover(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            shutil.copytree(ROOT/'sensors',root/'sensors')
            rover=root/'rover/test'; rover.mkdir(parents=True)
            (rover/'model.sdf').write_text('<sdf version="1.10"><model name="other"><link name="body"/><link name="mast"><pose relative_to="body">0 0 1 0 0 0</pose></link><joint name="mast_mount" type="fixed"><parent>body</parent><child>mast</child></joint></model></sdf>')
            (rover/'definition.json').write_text(json.dumps({'base_frame':'body','model':'model.sdf','bridges':[],
                'static_transforms':[['body','mast','0 0 1 0 0 0']]}))
            data=json.loads((ROOT/'calibration/legacy.json').read_text())
            data['instances']=data['instances'][:1]
            data['instances'][0]['parent']='mast'
            path=root/'calibration.json'; path.write_text(json.dumps(data))
            registry=Registry(root,rover='test',calibration=path)
            self.assertEqual(registry.model().find('model').get('name'),'other')
            self.assertIn(('body','mast','0 0 1 0 0 0'),registry.transforms())
            self.assertFalse(registry.by_definition('ins'))
            self.assertFalse(registry.by_definition('wheel_encoders'))
            data['instances']=[]; path.write_text(json.dumps(data))
            self.assertEqual(Registry(root,rover='test',calibration=path).instances,[])

    def test_resolved_bundle_has_assets_adapters_and_consistent_tf(self):
        registry=Registry(ROOT,'atlas')
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'assembled'
            resolved=assemble(registry,'forest',output)
            self.assertEqual(json.loads((output/'tf.json').read_text()),[list(t) for t in registry.transforms()])
            self.assertEqual(json.loads((output/'bridge.yaml').read_text()),resolved['bridges'])
            self.assertEqual({a['instance'] for a in resolved['adapters']},{'wheel_encoders','gnss_velocity'})
            for adapter in resolved['adapters']: self.assertTrue((output/adapter['path']).is_file())
            self.assertTrue((output/'assets/atlas/meshes/chassis.dae').is_file())
            for uri in ET.parse(output/'model.sdf').findall('.//mesh/uri'):
                self.assertTrue(uri.text.startswith('file:'))
            with self.assertRaises(ValueError): assemble(registry,'forest',output)

    def test_installed_asset_layout_is_self_contained(self):
        with tempfile.TemporaryDirectory() as temp:
            prefix=Path(temp)
            for target,files in asset_data_files(ROOT):
                destination=prefix/target; destination.mkdir(parents=True,exist_ok=True)
                for source in files: shutil.copy2(source,destination/Path(source).name)
            installed=prefix/'share/my_robot'
            registry=Registry(installed,'atlas')
            resolved=assemble(registry,'road',prefix/'bundle')
            self.assertEqual(len(resolved['outputs']),18)
            self.assertTrue((prefix/'bundle/assets/atlas/control/keyboard_drive.py').exists())

    def test_enable_disable_dependency_validation(self):
        with self.assertRaises(ValueError): Registry(ROOT,'atlas',disable=['gnss'])
        registry=Registry(ROOT,'atlas',disable=['gnss','gnss_velocity'])
        self.assertFalse(any(o['topic']=='/rover/gnss/velocity' for o in registry.outputs()))
        with self.assertRaises(ValueError): Registry(ROOT,disable=['unknown'])
        with self.assertRaises(ValueError): Registry(ROOT,enable=['camera'],disable=['camera'])

    def test_antenna_rotation_and_velocity_reset(self):
        result=antenna_position((1,2,3),(0,0,math.sqrt(0.5),math.sqrt(0.5)),(0.5,0,0))
        for value,expected in zip(result,(1,2.5,3)): self.assertAlmostEqual(value,expected)
        sampler=VelocitySampler(20.0)
        self.assertIsNone(sampler.sample(0,(0,0,0)))
        self.assertIsNone(sampler.sample(10_000_000,(0.01,0,0)))
        self.assertAlmostEqual(sampler.sample(50_000_000,(0.05,0,0))[0],1)
        self.assertIsNone(sampler.sample(0,(10,10,10)))
        self.assertEqual(sampler.sample(50_000_000,(10,10,10)),(0,0,0))


if __name__=='__main__': unittest.main()
