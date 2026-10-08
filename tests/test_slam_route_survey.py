import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
tree=ast.parse((ROOT/'scripts/slam_route_survey.py').read_text())
namespace={'math':math,'np':np,'DEFAULT_FOOTPRINT':((.042,.085),(.042,-.085),(-.147,-.085),(-.147,.085))}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'<survey helpers>','exec'),namespace)


def scan(obstacle=None):
    ranges=np.full(360,3.)
    minimum=-math.pi
    increment=math.pi/180
    if obstacle is not None:
        x,y=obstacle
        angle=math.atan2(y,x+.0875)
        index=int(round((angle-minimum)/increment))%360
        ranges[index]=math.hypot(x+.0875,y)
    return SimpleNamespace(ranges=ranges,angle_min=minimum,
                           angle_increment=increment,range_min=.08,range_max=8.)


class SlamSurveyHelpersTest(unittest.TestCase):
    def test_anchor_rotates_relative_route_without_truth_pose(self):
        goals=namespace['anchored_route']([(1,2),(1,3),(1,2)],math.pi/2,(5,6,0))
        np.testing.assert_allclose(goals,[(6,6),(5,6)],atol=1e-12)

    def test_clear_scan_allows_forward_and_rotation(self):
        for speed,turn in [(.25,0),(0,.6),(.2,.3)]:
            self.assertTrue(namespace['laser_motion_clear'](scan(),(-.0875,0),speed,turn))

    def test_forward_swept_footprint_stops_before_obstacle(self):
        self.assertFalse(namespace['laser_motion_clear'](scan((.20,0)),(-.0875,0),.25,0))

    def test_current_footprint_collision_rejects_motion(self):
        self.assertFalse(namespace['laser_motion_clear'](scan((0,.10)),(-.0875,0),0,.6))

    def test_rotation_sweep_checks_side_obstacle(self):
        self.assertFalse(namespace['laser_motion_clear'](scan((.10,.09)),(-.0875,0),0,-.6))

    def test_invalid_scan_fails_closed(self):
        message=scan()
        message.ranges[:]=float('nan')
        self.assertFalse(namespace['laser_motion_clear'](message,(-.0875,0),.25,0))

    def test_mode_switch_stops_the_survey_controller(self):
        runtime=(ROOT/'scripts/runtime_control.py').read_text()
        self.assertIn("'slam_route_survey'",runtime)
        self.assertIn('|slam_route_survey)',runtime)


if __name__=='__main__':
    unittest.main()
