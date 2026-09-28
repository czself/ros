import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from person_reporting import PersonCounter, foreground_position, calibrated_intrinsics


class PersonReportingTest(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((Path(__file__).resolve().parents[1]/
            'navigation/person_reporting.json').read_text())

    def observation(self, x, y, category='resident'):
        return {'world_xy': [x, y], 'street': 'A', 'class': category,
                'confidence': .9, 'box': [0, 0, 10, 20], 'image_path': 'photo.png'}

    def test_replayed_source_frame_never_counts_twice(self):
        counter=PersonCounter(self.config)
        stamp={'secs':1,'nsecs':0};obs=[self.observation(.9,.8)]
        counter.add_view('POINT_2',stamp,obs)
        self.assertEqual(counter.add_view('POINT_2',stamp,obs),[])
        self.assertEqual(counter.report()['counts']['total'],1)

    def test_same_person_at_another_view_deduplicates(self):
        counter=PersonCounter(self.config)
        counter.add_view('POINT_2',{'secs':1,'nsecs':0},[self.observation(.9,.8)])
        counter.add_view('POINT_3',{'secs':2,'nsecs':0},[self.observation(.915,.79)])
        report=counter.report()
        self.assertEqual(report['counts']['total'],1)
        self.assertEqual(report['duplicate_observations'],1)

    def test_close_different_people_remain_distinct_in_every_frame(self):
        counter=PersonCounter(self.config)
        # Both observations are closer than the association radius, but
        # a one-to-one assignment keeps distinct people in the same frame.
        obs=[self.observation(.9,.8),self.observation(.92,.8)]
        counter.add_view('POINT_2',{'secs':1,'nsecs':0},obs)
        counter.add_view('POINT_3',{'secs':2,'nsecs':0},list(reversed(obs)))
        self.assertEqual(counter.report()['counts']['total'],2)
        self.assertEqual([len(t['observations']) for t in counter.tracks],[2,2])

    def test_class_conflict_is_reported_not_hidden(self):
        counter=PersonCounter(self.config)
        counter.add_view('POINT_2',{'secs':1,'nsecs':0},[self.observation(.9,.8)])
        counter.add_view('POINT_3',{'secs':2,'nsecs':0},[self.observation(.9,.8,'stranger')])
        self.assertFalse(counter.report()['complete'])
        self.assertIn('CLASS_CONFLICT',counter.report()['issues'][0]['reason'])

    def test_inventory_is_validation_not_a_source_of_counts(self):
        counter=PersonCounter(self.config)
        report=counter.report()
        self.assertEqual(report['counts']['total'],0)
        self.assertFalse(report['complete'])

    def test_missing_second_outsider_fails_validation(self):
        counter=PersonCounter(self.config)
        counter.add_view('POINT_2',{'secs':1,'nsecs':0},[self.observation(.9,.8,'stranger')])
        self.assertFalse(counter.report()['checks']['stranger'])

    def test_foreground_depth_does_not_use_background_median(self):
        depth=np.full((100,100),3.0,dtype=np.float32)
        depth[30:70,40:60]=.35
        item={'box':[20,10,80,90]}
        pose={'x':0.,'y':0.,'z':0.,'quaternion':[0,0,0,1]}
        position,measurement=foreground_position(item,depth,
            [[100,0,50],[0,100,50],[0,0,1]],pose,(100,100,3))
        self.assertAlmostEqual(measurement['foreground_depth_m'],.35,places=5)
        self.assertEqual(position,[0.,0.])

    def test_invalid_near_depth_never_substitutes_floor(self):
        depth=np.full((100,100),np.nan,dtype=np.float32)
        depth[80:100]=1.0
        with self.assertRaisesRegex(ValueError,'NO_TORSO'):
            foreground_position({'box':[20,10,80,90]},depth,
                [[100,0,50],[0,100,50],[0,0,1]],
                {'x':0,'y':0,'z':0,'quaternion':[0,0,0,1]},(100,100,3))


if __name__=='__main__':
    unittest.main()
