"""直接调用 C++ MotionController 的运动接口探针。"""

import json
import subprocess

import yaml

from test.support.runtime import ROOT


def check_motion(session, processes, robot_xml):
    source = ROOT / 'test/probes/motion_controller'
    build = ROOT / 'build/motion_controller_probe'
    with (session.output / 'probe_build.log').open('w') as log:
        subprocess.run(['cmake', '-S', str(source), '-B', str(build)],
                       check=True, stdout=log, stderr=subprocess.STDOUT, timeout=60)
        subprocess.run(['cmake', '--build', str(build), '-j2'],
                       check=True, stdout=log, stderr=subprocess.STDOUT, timeout=180)

    config_dir = ROOT / 'src/my_robot_moveit_pkg/config'
    params = {'/**': {'ros__parameters': {
        'robot_description': robot_xml,
        'robot_description_semantic': (config_dir / 'piper.srdf').read_text(),
        'robot_description_kinematics': yaml.safe_load((config_dir / 'kinematics.yaml').read_text()),
        'use_sim_time': True,
        'report_file': str(session.output / 'motion.json'),
    }}}
    config = session.output / 'probe_params.yaml'
    config.write_text(yaml.safe_dump(params))
    probe = processes.start([str(build / 'motion_controller_probe'),
                             '--ros-args', '--params-file', str(config)], 'motion')
    session.waiter.require(lambda: probe.poll() is not None, 180, '运动探针超时')
    session.report['motion'] = json.loads((session.output / 'motion.json').read_text())
    checks = session.report['motion']
    print(f"运动接口：{sum(check['passed'] for check in checks)}/{len(checks)}", flush=True)
    return probe.returncode == 0 and bool(checks) and all(check['passed'] for check in checks)
