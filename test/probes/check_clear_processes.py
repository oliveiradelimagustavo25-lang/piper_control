#!/usr/bin/env python3
"""离线验证进程选择边界，不创建或终止任何 ROS/系统进程。"""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('clear_processes', ROOT/'script/build/clear_processes.py')
clear = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clear)


def process(pid, parent, kind='node', orphan=False, domain=227, cwd=ROOT):
    return clear.Process(pid,parent,str(pid),kind,domain,cwd,'test',orphan)


class ClearSelectionCheck(unittest.TestCase):
    def test_orphan_launch_and_children(self):
        chain=[process(10,1,'launch',True),process(11,10),process(12,10,'rviz')]
        targets,leaders=clear.select_targets(chain,ROOT,227)
        self.assertEqual({p.pid for p in targets},{10,11,12})
        self.assertEqual({p.pid for p in leaders},{10})

    def test_active_launch_protected(self):
        chain=[process(10,2,'launch'),process(11,10)]
        self.assertEqual(clear.select_targets(chain,ROOT,227),([],[]))

    def test_domain_workspace_and_standalone(self):
        chain=[process(10,1,'launch',True,228),process(11,10,domain=228),
               process(20,1,'launch',True,cwd=Path('/tmp/other_workspace')),
               process(21,20,cwd=Path('/tmp/other_workspace')),
               process(30,1,orphan=True),process(31,1,'rviz',True)]
        targets,leaders=clear.select_targets(chain,ROOT,227)
        self.assertEqual({p.pid for p in targets},{30})
        self.assertEqual({p.pid for p in leaders},{30})


if __name__=='__main__':
    unittest.main()
