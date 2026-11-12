#!/usr/bin/env python3
"""离线验证放置规则、实际 launch 参数优先级和无运动 dry-run。"""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rules = load('placement_config', ROOT/'script/build/placement_config.py')
POSITIONS = {c:[-0.05,-0.01,0.0005] for c in rules.COLORS}


class PlacementConfigCheck(unittest.TestCase):
    def test_validation(self):
        for value in [None, {}, {'red':None,'green':[0,0,0],'blue':[0,0,0]},
                      {**POSITIONS,'blue':[0,float('nan'),0]}, {**POSITIONS,'red':[False,0,0]}]:
            with self.assertRaises(ValueError):
                rules.parameters({'place_positions':value})
        for value in [None,[],['relative']*2,['relative','absolute','invalid']]:
            with self.assertRaises(ValueError):
                rules.parameters({'place_position_modes':value})

    def test_no_implicit_legacy_offsets(self):
        old={'red.place_position':[0.26,-0.06,0.005]}
        self.assertEqual(rules.resolve(old),old)
        with self.assertRaises(ValueError):
            rules.resolve(old,'[relative, relative, relative]')
        with self.assertRaises(ValueError):
            rules.resolve({'place_positions':None})

    def test_effective_override_can_replace_lower_null(self):
        result=rules.resolve({'place_positions.red':None, 'place_position_modes':None},
                             '[relative, absolute, absolute]',yaml.safe_dump(POSITIONS))
        self.assertEqual(result['place_positions.red'],POSITIONS['red'])
        self.assertEqual(result['place_position_modes'],['relative','absolute','absolute'])

    def test_actual_launch_priority(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        launch=load('mujoco_launch',ROOT/'src/my_robot_bringup/launch/mujoco.launch.py')
        motion=load('motion_launch',ROOT/'src/my_robot_bringup/launch/motion_control.launch.py')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            path.write_text(yaml.safe_dump({'place_position_modes':['relative']*3,'place_positions':POSITIONS}))
            context=LaunchContext()
            context.launch_configurations.update(launch_config=str(path),place_position_modes='[relative, absolute, absolute]')
            for action in launch.generate_launch_description().entities:
                if isinstance(action,DeclareLaunchArgument):
                    action.execute(context)
            base={'place_position_modes':['absolute']*3,
                  **{'place_positions.'+c:[10.,20.,30.] for c in rules.COLORS}}
            result=motion.resolve_placement(base,context.launch_configurations['place_position_modes'],
                                           context.launch_configurations['place_positions'])
            self.assertEqual(result['place_position_modes'],['relative','absolute','absolute'])
            self.assertEqual(result['place_positions.red'],POSITIONS['red'])
            explicit={c:[0.1,0.2,0.3] for c in rules.COLORS}
            result=motion.resolve_placement(result,'',yaml.safe_dump(explicit))
            self.assertEqual(result['place_positions.red'],explicit['red'])

    def test_dry_run_through_script(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            path.write_text(yaml.safe_dump({'place_position_modes':['relative']*3,'place_positions':POSITIONS,
                'planner_config':str(ROOT/'src/my_robot_planner/config/task_planner.yaml'),
                'manipulation_config':str(ROOT/'src/my_robot_commander/config/manipulation.yaml')}))
            positions={c:[-0.24,-0.06,-0.12] for c in rules.COLORS}
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path),'--',
                'place_position_modes:=[relative, absolute, absolute]',
                'place_positions:='+yaml.safe_dump(positions,default_flow_style=True)],
                capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('red object center = [initial_x -0.24, -0.06, -0.12]',result.stdout)
            self.assertNotIn('process started',result.stdout)
            alternate=Path(directory)/'planner.yaml'
            alternate.write_text(yaml.safe_dump({'task_planner':{'ros__parameters':{
                'place_position_modes':['absolute']*3,
                **{'place_positions.'+c:[999.,999.,999.] for c in rules.COLORS}}}}))
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path),
                                   '--planner-config',str(alternate)],capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('red object center = [initial_x -0.05, initial_y -0.01, initial_z +0.0005]',result.stdout)
            self.assertNotIn('999',result.stdout)
            path.write_text('place_positions: null\n')
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path)],
                                  capture_output=True,text=True,timeout=20)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('null',result.stdout+result.stderr)


if __name__=='__main__':
    unittest.main()
