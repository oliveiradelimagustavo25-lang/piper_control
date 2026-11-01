#!/usr/bin/env python3
"""任务与感知回归入口；模拟执行端，不驱动机械臂。"""

import argparse
from pathlib import Path
import sys
import types
import unittest


def load_suite(args, output):
    # 调用前必须配置中间件环境；--help 不需要加载 ROS 类型支持。
    from test.stages.planner_checks import PlannerChecks
    from test.stages.perception_checks import PerceptionChecks

    classes = {'planner': PlannerChecks, 'perception': PerceptionChecks}
    for case in classes.values():
        case.output = output
    suite = unittest.TestSuite()
    loader = unittest.defaultTestLoader
    if args.checks:
        cases = types.SimpleNamespace(**{case.__name__: case for case in classes.values()})
        suite.addTests(loader.loadTestsFromNames(args.checks, cases))
    else:
        selected = classes.values() if args.stage == 'all' else [classes[args.stage]]
        for case in selected:
            suite.addTests(loader.loadTestsFromTestCase(case))
    return suite, list(classes.values())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['all', 'planner', 'perception'], default='all')
    parser.add_argument('checks', nargs='*', help='可选：PlannerChecks.test_… 或 PerceptionChecks.test_…')
    args = parser.parse_args(argv)

    from test.support.runtime import configure_ros, interrupt_on_sigterm, output_directory, write_report

    output = output_directory('planner_validation')
    configure_ros(output, domain=94, middleware='rmw_fastrtps_cpp')
    interrupt_on_sigterm()
    print('输出目录：' + str(output), flush=True)
    report = {'stage': args.stage, 'passed': False}
    classes = []
    try:
        suite, classes = load_suite(args, output)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        report.update(tests_run=result.testsRun, passed=result.wasSuccessful(),
                      failures=[(str(case), detail) for case, detail in result.failures],
                      errors=[(str(case), detail) for case, detail in result.errors])
    except KeyboardInterrupt:
        report['error'] = '测试被中断'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        # unittest 中途被中断时也释放夹具；正常执行后清理队列已为空。
        for case in classes:
            case.doClassCleanups()
        write_report(output, report)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
