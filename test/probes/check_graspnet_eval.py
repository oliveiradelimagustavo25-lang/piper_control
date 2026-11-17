#!/usr/bin/env python3
"""CPU-only regression checks for staged GraspNet evaluation; no ROS or GPU."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


predict = load('grasp_predict_check', ROOT/'tools/graspnet/scripts/predict.py')
evaluate = load('grasp_eval_check', ROOT/'test/graspnet_eval.py')


def grasp(width, x=0.3, angle=0.):
    row = np.zeros(17, dtype=np.float32)
    row[0:4] = [0.9, width, .02, .03]
    c, s = np.cos(angle), np.sin(angle)
    row[4:13] = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]).ravel()
    row[13:16] = [x, 0., .1]
    row[16] = -1
    return row


class EvaluationCheck(unittest.TestCase):
    def test_nms_return_value_is_used_before_width_filter(self):
        class Group:
            def __init__(self, array):
                self.grasp_group_array = array

            def __len__(self):
                return len(self.grasp_group_array)

            def __getitem__(self, mask):
                return Group(self.grasp_group_array[mask])

            def nms(self):
                # Match the real API's return-new-object contract.
                return Group(self.grasp_group_array[[0, 2]])

            def sort_by_score(self):
                pass

        class NoCollision:
            def __init__(self, scene, **kwargs):
                pass

            def detect(self, group, **kwargs):
                return np.zeros(len(group), dtype=bool)

        array = np.array([grasp(.09), grasp(.09, x=.301), grasp(.04, x=.5)])
        stages = predict.postprocess(Group(array), np.zeros((1, 3)),
                                     dict(collision_voxel=.01, approach_dist=.05,
                                          collision_thresh=.01, width_max=.07), NoCollision)
        self.assertEqual([len(stages[k]) for k in
                          ('raw', 'after_collision', 'after_nms', 'width_filtered')], [3, 3, 2, 1])
        np.testing.assert_allclose(stages['width_filtered'][:, 1], [.04])

    def test_width_changes_attribution_and_preserves_roll(self):
        array = np.array([grasp(.09), grasp(.04, x=.5, angle=.7)])
        targets = [dict(body='cube', center=[.3, 0., .1])]
        before = predict.evaluate_stage(array, np.eye(4), targets, .02)
        after = predict.evaluate_stage(array[array[:, 1] <= .07], np.eye(4), targets, .02)
        self.assertTrue(before['attribution']['cube']['top1'])
        self.assertFalse(after['attribution']['cube']['top1'])
        candidate = after['candidates_top'][0]
        np.testing.assert_allclose(candidate['reference_rotation_base'], array[1, 4:13].reshape(3, 3))
        self.assertNotIn('tcp_base', candidate)
        self.assertEqual(candidate['depth'], float(array[1, 3]))

    def test_empty_candidates_are_valid_zero_hits(self):
        result = predict.evaluate_stage(np.empty((0, 17)), np.eye(4),
                                        [dict(body='cube', center=[.3, 0., .1])], .02)
        self.assertFalse(result['attribution']['cube']['top5'])
        self.assertEqual(result['candidates_top'], [])
        self.assertIsNone(result['width_mm']['med'])

    def test_errors_remain_in_denominator(self):
        stage = predict.evaluate_stage(np.array([grasp(.04)]), np.eye(4),
                                       [dict(body='cube', center=[.3, 0., .1])], .02)
        ok = dict(root='batch', scenario='scene', stages={s: stage for s in predict.STAGES},
                  counts=dict(raw=1, collision_blocked=0, after_nms=1, width_pass=1),
                  timings_ms=dict(cloud=1, forward=2, post=3))
        error = dict(root='batch', scenario='scene', error='bad depth')
        rows = evaluate.aggregate(dict(meta=dict(schema_version=2), results=[ok, error]), 'masked_random')
        for row in rows:
            self.assertEqual((row['n_items'], row['n_valid'], row['n_errors']), (2, 1, 1))
            self.assertEqual(row['top1_rate'], .5)
            self.assertEqual(row['top5_coverage'], .5)
        all_bad = evaluate.aggregate(dict(meta=dict(schema_version=2), results=[error]), 'masked_random')
        self.assertEqual(all_bad[0]['top1_rate'], 0.)
        self.assertIsNone(all_bad[0]['forward_ms'])

    def test_sampling_not_tied_to_item_order(self):
        seed = predict.item_seed(42, 'scene/frame/class1')
        self.assertEqual(seed, predict.item_seed(42, 'scene/frame/class1'))
        self.assertNotEqual(seed, predict.item_seed(42, 'scene/frame/class2'))
        points = np.arange(300).reshape(100, 3)
        np.testing.assert_array_equal(predict.sample(points, 20, np.random.default_rng(seed)),
                                      predict.sample(points, 20, np.random.default_rng(seed)))

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            predict.build_cloud(np.ones((3, 3)), dict(fx=0., fy=1., cx=0., cy=0.))
        with self.assertRaises(ValueError):
            evaluate.aggregate(dict(meta={}, results=[]), 'masked_random')


if __name__ == '__main__':
    unittest.main()
