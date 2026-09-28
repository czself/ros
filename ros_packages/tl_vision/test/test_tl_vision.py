"""tl_vision 单元测试。

只覆盖不依赖 ROS、不依赖模型权重的部分 —— 目的是让队友 clone 下来
``catkin_make run_tests`` 就能验证环境, 不需要先备好 best.pt。

需要模型的集成测试请直接用离线工具:
    python3 -m tl_vision.offline -i <图片> -w best.pt
"""
import unittest

from tl_vision.config import load_config
from tl_vision.detector import (
    DEFAULT_CLASS_NAMES,
    Detection,
    ForceRule,
    InferenceResult,
    _sorted_by_group,
    parse_force_rules,
)


def _det(name, conf=0.9):
    return Detection(
        class_id=DEFAULT_CLASS_NAMES.index(name),
        class_name=name,
        confidence=conf,
        x1=1.0,
        y1=2.0,
        x2=11.0,
        y2=22.0,
    )


class TestDetection(unittest.TestCase):
    def test_geometry(self):
        d = _det("red_on")
        self.assertEqual(d.width, 10.0)
        self.assertEqual(d.height, 20.0)
        self.assertEqual(d.area, 200.0)
        self.assertEqual(d.xyxy, (1.0, 2.0, 11.0, 22.0))

    def test_semantic_flags(self):
        self.assertTrue(_det("green_on").is_traffic_light)
        self.assertTrue(_det("stranger").is_person)
        self.assertTrue(_det("license_plate").is_plate)
        self.assertFalse(_det("resident").is_plate)

    def test_light_state_groups_on_off(self):
        self.assertEqual(_det("red_on").light_state, "red")
        self.assertEqual(_det("red_off").light_state, "red")
        self.assertEqual(_det("yellow_on").light_state, "yellow")
        self.assertIsNone(_det("license_plate").light_state)


class TestInferenceResult(unittest.TestCase):
    def setUp(self):
        self.res = InferenceResult(
            detections=[
                _det("license_plate", 0.6),
                _det("red_on", 0.8),
                _det("resident", 0.7),
                _det("stranger", 0.95),
                _det("green_off", 0.55),
            ],
            inference_ms=12.5,
            frame_width=640,
            frame_height=480,
        )

    def test_grouping_queries(self):
        self.assertEqual(len(self.res.plates), 1)
        self.assertEqual(len(self.res.traffic_lights), 2)
        self.assertEqual(len(self.res.people), 2)
        self.assertEqual(len(self.res), 5)

    def test_dominant_light_picks_highest_conf(self):
        self.assertEqual(self.res.dominant_light().class_name, "red_on")

    def test_dominant_light_none_when_absent(self):
        self.assertIsNone(InferenceResult(detections=[_det("resident")]).dominant_light())

    def test_to_dict_is_json_friendly(self):
        import json

        json.dumps(self.res.plates[0].to_dict())  # 不应抛异常


class TestSorting(unittest.TestCase):
    def test_order_is_people_then_lights_then_plates(self):
        dets = [
            _det("license_plate", 0.99),
            _det("red_on", 0.10),
            _det("resident", 0.20),
        ]
        got = [d.class_name for d in _sorted_by_group(dets)]
        self.assertEqual(got, ["resident", "red_on", "license_plate"])

    def test_within_group_sorted_by_confidence_desc(self):
        dets = [_det("red_on", 0.3), _det("green_on", 0.9), _det("red_off", 0.6)]
        got = [d.confidence for d in _sorted_by_group(dets)]
        self.assertEqual(got, [0.9, 0.6, 0.3])


class TestForceRules(unittest.TestCase):
    def test_parses_valid_rules(self):
        rules = parse_force_rules(
            [{"match": "红灯亮", "class_name": "red_on"}]
        )
        self.assertEqual(len(rules), 1)
        self.assertIsInstance(rules[0], ForceRule)

    def test_drops_unknown_class_name(self):
        rules = parse_force_rules([{"match": "x", "class_name": "not_a_class"}])
        self.assertEqual(rules, [])

    def test_drops_incomplete_rule(self):
        self.assertEqual(parse_force_rules([{"match": "x"}]), [])
        self.assertEqual(parse_force_rules([{"class_name": "red_on"}]), [])

    def test_none_is_safe(self):
        self.assertEqual(parse_force_rules(None), [])


class TestConfig(unittest.TestCase):
    def test_shipped_config_is_valid(self):
        """随包发布的 config/vision_config.yaml 必须能加载且通过校验。"""
        cfg = load_config()
        self.assertGreaterEqual(cfg.inference.conf_threshold, 0.0)
        self.assertLessEqual(cfg.inference.conf_threshold, 1.0)

    def test_shipped_rules_parsed(self):
        cfg = load_config()
        names = {r["class_name"] for r in cfg.debug.force_full_image_rules}
        self.assertTrue(names <= set(DEFAULT_CLASS_NAMES))

    def test_override_preserves_sibling_keys(self):
        """深层覆盖不能把同级未提及的键冲掉 (曾出过 bug)。"""
        cfg = load_config(overrides={"debug": {"force_full_image_boxes": True}})
        self.assertTrue(cfg.debug.force_full_image_boxes)
        self.assertTrue(
            cfg.debug.force_full_image_rules,
            "覆盖单个 debug 字段不应丢掉 force_full_image_rules",
        )

    def test_rejects_out_of_range_thresholds(self):
        with self.assertRaises(ValueError):
            load_config(overrides={"inference": {"conf_threshold": 1.5}})
        with self.assertRaises(ValueError):
            load_config(overrides={"inference": {"iou_threshold": -0.2}})

    def test_rejects_bad_imgsz(self):
        with self.assertRaises(ValueError):
            load_config(overrides={"model": {"imgsz": 100}})  # 非 32 倍数

    def test_rejects_out_of_range_class_id(self):
        with self.assertRaises(ValueError):
            load_config(overrides={"inference": {"class_filter": [42]}})

    def test_missing_weights_raises_actionable_error(self):
        import os

        cfg = load_config(overrides={"model": {"weights": ""}})
        os.environ.pop("TL_VISION_WEIGHTS", None)
        with self.assertRaises(FileNotFoundError) as ctx:
            cfg.resolve_weights()
        self.assertIn("TL_VISION_WEIGHTS", str(ctx.exception))


class TestDatasetLabels(unittest.TestCase):
    """确认 msg 里的 class_id 语义与 data.yaml 不会走偏。"""

    def test_class_count_is_nine(self):
        self.assertEqual(len(DEFAULT_CLASS_NAMES), 9)

    def test_names_unique(self):
        self.assertEqual(len(set(DEFAULT_CLASS_NAMES)), len(DEFAULT_CLASS_NAMES))

    def test_shipped_data_yaml_matches_detector_order(self):
        import yaml

        from tl_vision.config import PKG_ROOT

        raw = yaml.safe_load(
            (PKG_ROOT / "config" / "data.yaml").read_text(encoding="utf-8-sig")
        )
        names = raw["names"]
        ordered = tuple(names[k] for k in sorted(names)) if isinstance(names, dict) else tuple(names)
        self.assertEqual(
            list(ordered),
            list(DEFAULT_CLASS_NAMES),
            "data.yaml 类别顺序必须与 detector.DEFAULT_CLASS_NAMES 逐位一致",
        )


if __name__ == "__main__":
    unittest.main()
