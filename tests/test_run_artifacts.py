import ast
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_artifacts import write_run_summary

ROOT = Path(__file__).resolve().parents[1]


def production_finish():
    # Exercise the production finalizer without importing unavailable host ROS.
    tree = ast.parse((ROOT / 'scripts/route_executor.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RouteExecutor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'finish')
    namespace = dict(os=os, json=json, write_run_summary=write_run_summary,
                     rospy=SimpleNamespace(get_param=lambda *a: '', logerr=Mock()),
                     Twist=Mock, String=lambda **kw: kw, BEST_PT_SHA256='checkpoint',
                     save_report=Mock(return_value={'complete': True}))
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<production finish>', 'exec'), namespace)
    return namespace['finish'], namespace


def executor(directory, photos=False):
    return SimpleNamespace(client=SimpleNamespace(cancel_all_goals=Mock()),
                           cmd_pub=SimpleNamespace(publish=Mock()),
                           status_pub=SimpleNamespace(publish=Mock()),
                           capture_photos=photos, person_counter=None,
                           photo_dir=directory, goal_events=[{'name': 'POINT_10', 'result': 'NO_ROUTE_PROGRESS'}],
                           mission_error=None, strict_acceptance=True,
                           accepted_photo_records=[], hd_capture=None,
                           ocr_enabled=False, ocr_results=[], route=[('POINT_1', 0, 0, 0)])


class RunArtifactsTest(unittest.TestCase):
    def test_atomic_summary_contains_complete_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_run_summary(directory, {'route_status': 'FAILED', 'detail': '失败'})
            self.assertEqual(json.loads(Path(path).read_text())['detail'], '失败')
            self.assertEqual(os.listdir(directory), ['run_summary.json'])

    def test_serialization_failure_preserves_previous_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_run_summary(directory, {'route_status': 'FAILED'})
            previous = Path(path).read_bytes()
            with self.assertRaises(ValueError):
                write_run_summary(directory, {'value': float('nan')})
            self.assertEqual(Path(path).read_bytes(), previous)
            self.assertEqual(os.listdir(directory), ['run_summary.json'])

    def test_replace_failure_preserves_previous_summary_and_removes_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_run_summary(directory, {'route_status': 'FAILED'})
            previous = Path(path).read_bytes()
            with patch('run_artifacts.os.replace', side_effect=OSError('disk failure')):
                with self.assertRaises(OSError):
                    write_run_summary(directory, {'route_status': 'COMPLETE_PARKED'})
            self.assertEqual(Path(path).read_bytes(), previous)
            self.assertEqual(os.listdir(directory), ['run_summary.json'])

    def test_diagnostic_success_without_photos_is_saved(self):
        finish, _ = production_finish()
        with tempfile.TemporaryDirectory() as directory:
            node = executor(directory)
            self.assertTrue(finish(node, True))
            report = json.loads((Path(directory)/'run_summary.json').read_text())
            self.assertEqual(report['route_status'], 'COMPLETE_PARKED')
            self.assertFalse(report['capture_photos'])
            node.client.cancel_all_goals.assert_called_once()
            node.cmd_pub.publish.assert_called_once()
            node.status_pub.publish.assert_called_once_with('COMPLETE_PARKED')

    def test_failed_run_preserves_exception_and_goal_reason(self):
        finish, _ = production_finish()
        with tempfile.TemporaryDirectory() as directory:
            node = executor(directory)
            node.mission_error = {'type': 'ValueError', 'detail': 'bad image'}
            self.assertFalse(finish(node, False))
            report = json.loads((Path(directory)/'run_summary.json').read_text())
            self.assertEqual(report['route_status'], 'FAILED')
            self.assertEqual(report['mission_error'], node.mission_error)
            self.assertEqual(report['goals'][0]['result'], 'NO_ROUTE_PROGRESS')

    def test_person_report_failure_does_not_prevent_failed_summary(self):
        finish, namespace = production_finish()
        namespace['save_report'].side_effect = OSError('report disk failure')
        with tempfile.TemporaryDirectory() as directory:
            node = executor(directory, photos=True)
            node.person_counter = object()
            self.assertFalse(finish(node, True))
            report = json.loads((Path(directory)/'run_summary.json').read_text())
            self.assertEqual(report['route_status'], 'FAILED')
            self.assertEqual(report['finalization_errors'][0]['operation'], 'person_report')
            node.status_pub.publish.assert_called_once_with('FAILED:MISSION')

    def test_cancellation_failure_still_attempts_stop_and_saves_failure(self):
        finish, _ = production_finish()
        with tempfile.TemporaryDirectory() as directory:
            node = executor(directory)
            node.client.cancel_all_goals.side_effect = RuntimeError('action unavailable')
            self.assertFalse(finish(node, True))
            node.cmd_pub.publish.assert_called_once()
            report = json.loads((Path(directory)/'run_summary.json').read_text())
            self.assertEqual(report['finalization_errors'][0]['operation'], 'cancel_goals')

    def test_summary_write_failure_never_reports_success(self):
        finish, namespace = production_finish()
        namespace['write_run_summary'] = Mock(side_effect=OSError('read-only'))
        with tempfile.TemporaryDirectory() as directory:
            node = executor(directory)
            self.assertFalse(finish(node, True))
            node.status_pub.publish.assert_called_once_with('FAILED:SUMMARY_WRITE')


if __name__ == '__main__':
    unittest.main()
