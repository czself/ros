#!/usr/bin/env python3
import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from route_geometry import PaintGeometry, load_contract, preflight
from audit_calibrated_run import audit


class CalibratedRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.points = load_contract(ROOT / 'navigation/inner_route.yaml')
        cls.geometry = PaintGeometry(ROOT / 'models/competition_ground/materials/textures/map.png')

    def test_real_texture_preflight(self):
        self.assertEqual(preflight(self.geometry, self.points), [])

    def test_calibrated_topology_and_same_birth(self):
        np.testing.assert_array_equal(self.points[0], self.points[-1])
        actual = np.sign(np.diff(self.points, axis=0)).astype(int).tolist()
        # inner_route.yaml includes tangent cut-ins at the corners. Keep an
        # explicit topology expectation so an accidental route edit is caught.
        self.assertEqual(actual, [
            [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1],
            [-1, -1], [-1, 0], [-1, -1], [0, -1],
            [1, -1], [1, 0], [1, -1], [0, -1],
        ])

    def test_midsegment_paint_is_rejected(self):
        # Put one prohibited texture pixel at an otherwise clear segment's
        # midpoint: endpoints remain clear but swept preflight must fail.
        p = (self.points[1] + self.points[2]) / 2
        row, col = self.geometry.pixel(*p)
        original = self.geometry.paint[row, col]
        try:
            self.geometry.paint[row, col] = 1
            failures = preflight(self.geometry, self.points)
            self.assertTrue(any(f[0] == 1 and f[3] == 'translation' for f in failures))
        finally:
            self.geometry.paint[row, col] = original

    def sample_record(self):
        samples = []
        t = 0.
        for i, (a, b) in enumerate(zip(self.points[:-1], self.points[1:])):
            yaw = math.atan2(*(b-a)[::-1])
            if samples:
                previous_yaw = samples[-1]['pose'][2]
                angle = math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw))
                for fraction in np.linspace(0, 1, math.ceil(abs(angle)/.025)+1):
                    samples.append({'t': t, 'pose': [*a, previous_yaw+fraction*angle],
                                    'segment': i, 'ages': [.01, .02, .03],
                                    'speed': 0., 'yaw_rate': .5})
                    t += .05
            for fraction in np.linspace(0, 1, math.ceil(np.linalg.norm(b-a)/.008)+1):
                p = a + fraction * (b-a)
                samples.append({'t': t, 'pose': [*p, yaw], 'segment': i,
                                'ages': [.01, .02, .03], 'speed': .16, 'yaw_rate': 0.})
                t += .05
        for _ in range(20):
            s = copy.deepcopy(samples[-1]); s.update(t=t, speed=0.)
            samples.append(s); t += .05
        return {'result': 'COMPLETE', 'planned': self.points.tolist(),
                'completed_segments': list(range(len(self.points) - 1)),
                'samples': samples}

    def test_claimed_completion_without_trajectory_is_rejected(self):
        self.assertFalse(audit({'result': 'COMPLETE'}, self.geometry, self.points)['accepted'])

    def test_point_goal_count_does_not_override_wrong_trajectory(self):
        record = self.sample_record()
        record['samples'][20]['pose'][0] -= .4
        result = audit(record, self.geometry, self.points)
        self.assertFalse(result['accepted'])
        self.assertIn('measured corridor violation', result['failures'])

    def test_teleported_turn_is_not_accepted(self):
        record = self.sample_record()
        record['samples'][10]['pose'][2] += math.pi/2
        report = audit(record, self.geometry, self.points)
        self.assertFalse(report['accepted'])
        self.assertIn('pose discontinuity or excessive motion', report['failures'])

    def test_complete_measured_route_passes(self):
        report = audit(self.sample_record(), self.geometry, self.points)
        self.assertTrue(report['accepted'], report)

    def test_truncated_completion_manifest_is_rejected(self):
        record = self.sample_record()
        record['completed_segments'] = record['completed_segments'][:7]
        report = audit(record, self.geometry, self.points)
        self.assertFalse(report['accepted'])
        self.assertIn('completion manifest missing ordered segments',
                      report['failures'])

    def test_stale_sensor_evidence_is_rejected(self):
        record = self.sample_record()
        record['samples'][10]['ages'][2] = 1.5
        self.assertIn('missing or stale pose/laser/depth evidence',
                      audit(record, self.geometry, self.points)['failures'])

    def test_final_motion_is_rejected_even_at_home(self):
        record = self.sample_record()
        record['samples'][-3]['speed'] = .1
        self.assertIn('no sustained measured stationary finish',
                      audit(record, self.geometry, self.points)['failures'])


if __name__ == '__main__':
    unittest.main()
