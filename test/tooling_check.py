#!/usr/bin/env python3
"""验证测试工具的导入与资源清理，不启动 ROS 或仿真。"""

import importlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from test.support.runtime import Processes


class ToolingChecks(unittest.TestCase):
    def test_entry_imports_have_no_runtime_side_effects(self):
        environment = dict(os.environ)
        with patch.object(Path, 'mkdir') as mkdir, patch('subprocess.Popen') as popen:
            for name in ['test.planner_check', 'test.executor_check', 'test.sorting_run']:
                importlib.reload(importlib.import_module(name))
            mkdir.assert_not_called()
            popen.assert_not_called()
        self.assertEqual(dict(os.environ), environment)

    def test_cleanup_on_failure_does_not_stop_an_unrelated_process(self):
        # 两个独立进程组模拟“本次测试”和旁边已运行的程序。
        sibling = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                   start_new_session=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaisesRegex(RuntimeError, 'test failure'):
                    with Processes(Path(directory)) as processes:
                        child = processes.start(
                            [sys.executable, '-c', 'import time; time.sleep(60)'], 'child')
                        raise RuntimeError('test failure')
                self.assertIsNotNone(child.poll())
                self.assertIsNone(sibling.poll())
        finally:
            sibling.terminate()
            sibling.wait(timeout=5)

    def test_cleanup_after_parent_exit_stops_its_child(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            pid_file = output / 'child.pid'
            code = (
                'import pathlib, subprocess, sys; '
                'child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); '
                f'pathlib.Path({str(pid_file)!r}).write_text(str(child.pid))'
            )
            with Processes(output) as processes:
                parent = processes.start([sys.executable, '-c', code], 'parent')
                parent.wait(timeout=5)
                pid = int(pid_file.read_text())
            # 被回收前的僵尸进程也已停止；不能再有运行中的孤儿进程。
            status = Path(f'/proc/{pid}/stat')
            deadline = time.monotonic() + 2
            while status.exists() and status.read_text().split()[2] != 'Z':
                if time.monotonic() >= deadline:
                    self.fail('退出后仍有运行中的子进程')
                time.sleep(0.01)


if __name__ == '__main__':
    unittest.main(verbosity=2)
