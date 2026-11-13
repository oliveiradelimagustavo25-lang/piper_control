#!/usr/bin/env python3
"""类别场景、缺失引用、模型尺寸和实际 launch 优先级；不启动 ROS 节点。"""
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import warnings
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT/'src/my_robot_perception/config/object_classes.yaml'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scene = load('scene_config', ROOT/'script/build/scene_config.py')


class SceneConfigCheck(unittest.TestCase):
    def test_counts_and_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            for counts in [(1,1,1),(0,4,0),(3,1,1),(7,2,5),(0,0,0)]:
                config = {cid:[[0.32,i*0.03,0.03] for i in range(n)] for cid,n in enumerate(counts,1)}
                scene.generate(config,directory,ROOT/'src/my_robot_world',CATALOG)
                bodies=ET.parse(Path(directory)/'objects.xml').getroot().findall('body')
                self.assertEqual(len(bodies),sum(counts))
                self.assertEqual(len({b.get('name') for b in bodies}),len(bodies))
            for config,expected in [(None,3),({},0),({1:[[0.32,0,0.03]]},1)]:
                scene.generate(config,directory,ROOT/'src/my_robot_world',CATALOG)
                self.assertEqual(len(ET.parse(Path(directory)/'objects.xml').getroot()),expected)

    def test_missing_class_and_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            catalog=yaml.safe_load(CATALOG.read_text())
            cylinder=copy.deepcopy(catalog['classes'][0])
            cylinder.update(id=4,name='red_cylinder',geometry={'type':'cylinder','dimensions':[0.02,0.02,0.04]})
            cylinder.pop('simulation',None)
            cylinder.pop('cv',None)
            catalog['classes'].append(cylinder)
            path=root/'classes.yaml'
            path.write_text(yaml.safe_dump(catalog))
            with warnings.catch_warnings(record=True) as notes:
                scene.generate({1:[[0.32,0,0.03]],4:[[0.32,0.1,0.03]],10:[[0.3,-0.1,0.03]]},root,ROOT/'src/my_robot_world',path)
            report=json.loads((root/'scene_report.json').read_text())
            self.assertEqual(report['requested'],{'1':1,'4':1,'10':1})
            self.assertEqual(report['generated'],{'1':1,'4':1})
            self.assertTrue(notes)
            cylinder=ET.parse(root/'objects.xml').getroot().findall('body')[1].find('geom')
            self.assertEqual(cylinder.get('type'),'cylinder')
            self.assertEqual(list(map(float,cylinder.get('size').split())),[0.01,0.02])
            self.assertEqual(report['skipped'][0]['class_id'],10)
            import mujoco
            bodies=ET.parse(root/'objects.xml').getroot()
            xml='<mujoco><worldbody>'+''.join(ET.tostring(b,encoding='unicode') for b in bodies)+'</worldbody></mujoco>'
            model=mujoco.MjModel.from_xml_string(xml)
            self.assertEqual(model.ngeom,2)

    def test_scene_and_catalog_precedence(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        launch=load('mujoco_launch',ROOT/'src/my_robot_bringup/launch/mujoco.launch.py')
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            catalog=yaml.safe_load(CATALOG.read_text())
            catalog['classes'][0]['geometry']['dimensions']=[0.02,0.02,0.02]
            custom=root/'classes.yaml'
            custom.write_text(yaml.safe_dump(catalog))
            path=root/'launch.yaml'
            path.write_text(yaml.safe_dump({'scenes':{1:[[0.32,0,0.03]]},'class_config':'missing.yaml'}))
            context=LaunchContext()
            context.launch_configurations.update(launch_config=str(path),scene_file='explicit.xml')
            DeclareLaunchArgument('scene_file',default_value=launch.ConfigDefault('scene_file','default.xml')).execute(context)
            self.assertNotIn('_piper_generate_scene',context.launch_configurations)
            self.assertEqual(context.launch_configurations['scene_file'],'explicit.xml')
            context=LaunchContext()
            context.launch_configurations.update(launch_config=str(path),class_config=str(custom),dry_run='true')
            for action in launch.generate_launch_description().entities:
                if isinstance(action,DeclareLaunchArgument):
                    action.execute(context)
            try:
                launch.launch_setup(context)
                scene_path=Path(context.launch_configurations['scene_file'])
                geom=ET.parse(scene_path.parent/'objects.xml').getroot().find('body/geom')
                self.assertEqual(geom.get('size'),'0.01 0.01 0.01')
                self.assertEqual(context.launch_configurations['class_config'],str(custom))
            finally:
                if context.launch_configurations.get('_piper_scene_runtime'):
                    shutil.rmtree(context.launch_configurations['_piper_scene_runtime'])


if __name__=='__main__':
    unittest.main()
