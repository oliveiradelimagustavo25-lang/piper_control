#!/usr/bin/env python3
"""离线检查总配置 scenes、启动参数优先级和 XML 生成，不启动 ROS 节点。"""
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scene = load('scene_config', ROOT/'script/build/scene_config.py')


class SceneConfigCheck(unittest.TestCase):
    def test_counts_and_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            for red,green,blue in [(1,1,1),(0,4,0),(3,1,1),(7,2,5),(0,0,0)]:
                config = {c:[[0.32,i*0.03,0.03] for i in range(n)]
                          for c,n in zip(['red','green','blue'],[red,green,blue])}
                scene.generate(config,directory,ROOT/'src/my_robot_world')
                bodies=ET.parse(Path(directory)/'objects.xml').getroot().findall('body')
                self.assertEqual(len(bodies),red+green+blue)
                self.assertEqual(len({b.get('name') for b in bodies}),len(bodies))
            scene.generate({'red':[[0.32,-0.1,0.03],[0.38,-0.1,0.03]]},directory,ROOT/'src/my_robot_world')
            self.assertEqual(len(ET.parse(Path(directory)/'objects.xml').getroot()),4)
            for config in [None,{}]:
                scene.generate(config,directory,ROOT/'src/my_robot_world')
                self.assertEqual(len(ET.parse(Path(directory)/'objects.xml').getroot()),3)

    def test_scene_precedence(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        launch = load('mujoco_launch', ROOT/'src/my_robot_bringup/launch/mujoco.launch.py')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            path.write_text('scenes:\n  red: []\n  green: []\n  blue: []\n')
            context=LaunchContext()
            context.launch_configurations['launch_config']=str(path)
            context.launch_configurations['scene_file']='explicit.xml'
            DeclareLaunchArgument('scene_file',default_value=launch.ConfigDefault('scene_file','default.xml')).execute(context)
            self.assertEqual(context.launch_configurations['scene_file'],'explicit.xml')
            self.assertNotIn('_piper_scene_runtime',context.launch_configurations)
            del context.launch_configurations['scene_file']
            filename=launch.ConfigDefault('scene_file','default.xml').perform(context)
            try:
                self.assertEqual(len(ET.parse(Path(filename).parent/'objects.xml').getroot()),0)
            finally:
                shutil.rmtree(context.launch_configurations['_piper_scene_runtime'])


if __name__=='__main__':
    unittest.main()
