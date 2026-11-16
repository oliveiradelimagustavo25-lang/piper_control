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
POSITIONS = {c:[-0.05,-0.01,0.0005] for c in (1,2,3)}


class PlacementConfigCheck(unittest.TestCase):
    def test_validation(self):
        import warnings
        for value in [None, {1:None}, {1:[0,float('nan'),0]}, {1:[False,0,0]}]:
            with warnings.catch_warnings(record=True) as notes:
                result=rules.parameters({'place_positions':value})
            self.assertTrue(result['use_position_rules'])
            self.assertNotIn('place_positions.1',result)
            self.assertTrue(notes)
        result=rules.parameters({'place_positions':{1:[1,2,3]}})
        self.assertEqual(result['place_positions.1'],[1,2,3])
        self.assertNotIn('place_positions.2',result)
        for value in [None,[],['relative']*2,['relative','absolute','invalid']]:
            with self.assertRaises(ValueError):
                rules.parameters({'place_position_modes':value})

    def test_region_rules(self):
        import warnings
        region={4:[0.33,0.10,0.39,0.14,0.015]}
        result=rules.parameters({'place_regions':region})
        self.assertTrue(result['use_position_rules'])
        self.assertEqual(result['place_regions.4'],region[4])
        # 单行/单列区域允许：min == max 仍可生成一维候选。
        line=rules.parameters({'place_regions':{4:[0.26,-0.06,0.26,-0.06,0.005]}})
        self.assertIn('place_regions.4',line)
        for value in [None,{4:None},{4:[0,0,1,1]},{4:[0,0,float('nan'),1,0]},
                      {4:[1,0,0,1,0]},{4:[0,0,1,1,'x']}]:
            with warnings.catch_warnings(record=True) as notes:
                result=rules.parameters({'place_regions':value})
            self.assertTrue(result['use_position_rules'])
            self.assertNotIn('place_regions.4',result)
            self.assertTrue(notes)
        # 同类点+区域属于配置二义性：直接失败启动。
        with self.assertRaises(ValueError):
            rules.parameters({'place_positions':{4:[0,0,0]},'place_regions':region})
        overridden=rules.resolve({'place_regions.4':[1,2,3,4,5]},
                                 yaml.safe_dump(['absolute']*3),'',yaml.safe_dump(region))
        self.assertEqual(overridden['place_regions.4'],region[4])

    def test_region_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            path.write_text(yaml.safe_dump({'place_position_modes':['absolute']*3,
                'place_regions':{4:[0.33,0.10,0.39,0.14,0.015]},
                'planner_config':str(ROOT/'src/my_robot_planner/config/task_planner.yaml'),
                'manipulation_config':str(ROOT/'src/my_robot_commander/config/manipulation.yaml')}))
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path)],
                                  capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('class 4 region = x[0.33, 0.39] y[0.1, 0.14] z=0.015',result.stdout)

    def test_layer_override_across_kinds(self):
        region={4:[0.30,0.10,0.39,0.14,0.015]}
        point={4:[0.10,0.20,0.30]}
        modes=yaml.safe_dump(['absolute']*3)
        # 高层区域整体替换低层点规则，不残留成冲突。
        result=rules.resolve({'place_positions.4':point[4]},modes,'',yaml.safe_dump(region))
        self.assertIn('place_regions.4',result)
        self.assertNotIn('place_positions.4',result)
        # 高层点规则整体替换低层区域。
        result=rules.resolve({'place_regions.4':region[4]},modes,yaml.safe_dump(point),'')
        self.assertIn('place_positions.4',result)
        self.assertNotIn('place_regions.4',result)
        # 同一层同时给出点与区域仍是配置错误。
        with self.assertRaises(ValueError):
            rules.resolve({},modes,yaml.safe_dump(point),yaml.safe_dump(region))

    def test_no_implicit_legacy_offsets(self):
        old={'classes.1.place_position':[0.26,-0.06,0.005]}
        self.assertEqual(rules.resolve(old),old)
        with self.assertRaises(ValueError):
            rules.resolve(old,'[relative, relative, relative]')
        self.assertTrue(rules.resolve({'place_positions':{}})['use_position_rules'])
        empty=rules.resolve({},'[relative, relative, relative]','{}')
        self.assertTrue(rules.resolve(empty)['use_position_rules'])
        import warnings
        with warnings.catch_warnings(record=True):
            result=rules.resolve({'place_positions':None,'place_positions.1':[1.,2.,3.]})
        self.assertNotIn('place_positions.1',result)

    def test_effective_override_can_replace_lower_null(self):
        result=rules.resolve({'place_positions.1':None, 'place_position_modes':None},
                             '[relative, absolute, absolute]',yaml.safe_dump(POSITIONS))
        self.assertEqual(result['place_positions.1'],POSITIONS[1])
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
            launch.placement_defaults(context,launch.load_launch_config(str(path)))
            base={'place_position_modes':['absolute']*3,
                  **{'place_positions.'+str(c):[10.,20.,30.] for c in (1,2,3)}}
            result=motion.resolve_placement(base,context.launch_configurations['place_position_modes'],
                                           context.launch_configurations['place_positions'])
            self.assertEqual(result['place_position_modes'],['relative','absolute','absolute'])
            self.assertEqual(result['place_positions.1'],POSITIONS[1])
            explicit={c:[0.1,0.2,0.3] for c in (1,2,3)}
            result=motion.resolve_placement(result,'',yaml.safe_dump(explicit))
            self.assertEqual(result['place_positions.1'],explicit[1])

    def test_cli_overrides_yaml_across_kinds(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            point={4:[0.10,0.20,0.30]}
            region={4:[0.30,0.10,0.39,0.14,0.015]}

            def dry_run(config, **arguments):
                path.write_text(yaml.safe_dump(config))
                return subprocess.run(
                    [str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path),'--',
                     *[name+':='+yaml.safe_dump(value,default_flow_style=True).strip()
                       for name,value in arguments.items()]],
                    capture_output=True,text=True,timeout=20)

            for lower, higher, expected, absent in [
                ({'place_positions':point}, {'place_regions':region},
                 'class 4 region =', 'class 4 object center ='),
                ({'place_regions':region}, {'place_positions':point},
                 'class 4 object center =', 'class 4 region ='),
            ]:
                with self.subTest(lower=lower):
                    result=dry_run(lower,**higher)
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                    self.assertIn(expected,result.stdout)
                    self.assertNotIn(absent,result.stdout)
                    self.assertNotIn('process started',result.stdout)
            # An explicit empty rule set also replaces YAML rules.
            result=dry_run({'place_positions':point},place_regions={})
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertNotIn('class 4 object center =',result.stdout)
            # Changing only coordinate modes retains the YAML placement rules.
            result=dry_run({'place_positions':point},place_position_modes=['relative']*3)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('initial_x +0.1',result.stdout)
            # Conflicts within either source layer remain errors.
            for config,args in [
                ({},{'place_positions':point,'place_regions':region}),
                ({'place_positions':point,'place_regions':region},{}),
            ]:
                result=dry_run(config,**args)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('defines both',result.stdout+result.stderr)

    def test_dry_run_through_script(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'launch.yaml'
            path.write_text(yaml.safe_dump({'place_position_modes':['relative']*3,'place_positions':POSITIONS,
                'planner_config':str(ROOT/'src/my_robot_planner/config/task_planner.yaml'),
                'manipulation_config':str(ROOT/'src/my_robot_commander/config/manipulation.yaml')}))
            positions={c:[-0.24,-0.06,-0.12] for c in (1,2,3)}
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path),'--',
                'place_position_modes:=[relative, absolute, absolute]',
                'place_positions:='+yaml.safe_dump(positions,default_flow_style=True)],
                capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('class 1 object center = [initial_x -0.24, -0.06, -0.12]',result.stdout)
            self.assertNotIn('process started',result.stdout)
            alternate=Path(directory)/'planner.yaml'
            alternate.write_text(yaml.safe_dump({'task_planner':{'ros__parameters':{
                'place_position_modes':['absolute']*3,
                **{'place_positions.'+str(c):[999.,999.,999.] for c in (1,2,3)}}}}))
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path),
                                   '--planner-config',str(alternate)],capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('class 1 object center = [initial_x -0.05, initial_y -0.01, initial_z +0.0005]',result.stdout)
            self.assertNotIn('999',result.stdout)
            path.write_text('place_positions: {1: [0,0,0], 241: [0,0,0]}\n')
            result=subprocess.run([str(ROOT/'script/pick_three_colors.sh'),'--dry-run','--config',str(path)],
                                  capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('Ignored placement class 241',result.stdout)


if __name__=='__main__':
    unittest.main()
