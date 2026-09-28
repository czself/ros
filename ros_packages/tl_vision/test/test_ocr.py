"""车牌文本 + OCR 适配层的单元测试。

不依赖 paddleocr: 引擎用 stub, Paddle 的两代返回结构则直接喂给解析函数。
所以没装 Paddle 的机器 (CI、开发机) 也能跑。
"""
import unittest

from tl_vision.ocr import _detect_paddle_api, _parse_v2, _parse_v3
from tl_vision.plate_text import (
    build,
    format_for_file,
    is_new_energy,
    normalize,
    split_display,
    validate,
)


class TestNormalize(unittest.TestCase):
    def test_strips_separator(self):
        self.assertEqual(normalize("苏A·B8Q62"), "苏AB8Q62")

    def test_fullwidth_to_halfwidth(self):
        self.assertEqual(normalize("苏Ａ·B8Q62"), "苏AB8Q62")
        self.assertEqual(normalize("粤Ｂ１２３４５"), "粤B12345")

    def test_strips_surrounding_whitespace(self):
        self.assertEqual(normalize("  粤B 12345  "), "粤B12345")

    def test_uppercases_letters(self):
        self.assertEqual(normalize("sua12345"), "SUA12345")

    def test_empty(self):
        self.assertEqual(normalize(""), "")
        self.assertEqual(normalize("   "), "")

    def test_io_mapped_to_digits(self):
        """GA 16735 禁用 I/O, 出现即判为误识。"""
        self.assertEqual(normalize("京AD8I234"), "京AD81234")
        self.assertEqual(normalize("京AD8O234"), "京AD80234")

    def test_letter_l_survives(self):
        self.assertEqual(normalize("苏A·PL12A"), "苏APL12A")
        self.assertEqual(normalize("京A·LL1234"), "京ALL1234")


class TestSplitDisplay(unittest.TestCase):
    def test_inserts_separator(self):
        self.assertEqual(split_display("苏AB8Q62"), "苏A·B8Q62")
        self.assertEqual(split_display("粤B12345"), "粤B·12345")

    def test_too_short_returned_as_is(self):
        self.assertEqual(split_display("苏"), "苏")
        self.assertEqual(split_display(""), "")


class TestValidate(unittest.TestCase):
    def test_accepts_standard(self):
        self.assertIsNone(validate("苏AB8Q62"))
        self.assertIsNone(validate("鄂D7B5Q2"))
        self.assertIsNone(validate("粤B12345"))

    def test_accepts_new_energy_8_char(self):
        self.assertIsNone(validate("苏AD08080"))

    def test_rejects_empty(self):
        self.assertEqual(validate(""), "空结果")

    def test_rejects_bad_length(self):
        self.assertIn("长度", validate("苏AB8Q6"))
        self.assertIn("长度", validate("苏AB8Q623456"))

    def test_rejects_bad_province(self):
        self.assertIn("省份", validate("XY12345"))
        self.assertIn("省份", validate("ABCDEFG"))

    def test_rejects_bad_city_letter(self):
        self.assertIn("机关", validate("苏112345"))

    def test_flags_io_as_suspect(self):
        # normalize 已把 I/O 映走, 这里直接喂 validate 验证该规则仍在
        self.assertIn("I/O", validate("苏AB8I62X"))

    def test_special_plate_warns(self):
        self.assertIn("特殊号牌", validate("苏A"))


class TestIsNewEnergy(unittest.TestCase):
    def test_true_for_digit_heavy_8(self):
        self.assertTrue(is_new_energy("苏AD08080"))

    def test_false_for_7_char(self):
        self.assertFalse(is_new_energy("苏AB8Q62"))

    def test_false_for_mostly_alpha_8(self):
        self.assertFalse(is_new_energy("苏AAB8Q62X"))


class TestBuild(unittest.TestCase):
    def test_valid_plates(self):
        p = build("苏A·B8Q62", confidence=0.98)
        self.assertTrue(p.is_valid)
        self.assertIsNone(p.issue)
        self.assertEqual(p.text, "苏AB8Q62")
        self.assertEqual(p.display, "苏A·B8Q62")
        self.assertAlmostEqual(p.confidence, 0.98)

    def test_keeps_raw(self):
        p = build("  苏Ａ·B8Q62  ")
        self.assertEqual(p.raw, "  苏Ａ·B8Q62  ")

    def test_invalid_keeps_text(self):
        p = build("XYZ12345")
        self.assertFalse(p.is_valid)
        self.assertIsNotNone(p.issue)
        self.assertEqual(p.text, "XYZ12345")  # 非 strict: 保留但标脏

    def test_strict_clears_invalid_text(self):
        p = build("XYZ12345", strict=True)
        self.assertFalse(p.is_valid)
        self.assertEqual(p.text, "")

    def test_strict_also_clears_display(self):
        """display 是给人看的字段, 且 __str__ 优先取它 —— 只清 text 会漏脏数据。"""
        p = build("XYZ12345", strict=True)
        self.assertEqual(p.display, "")
        self.assertEqual(str(p), "XYZ12345")  # 只剩 raw 可供排查
        self.assertEqual(p.raw, "XYZ12345")

    def test_strict_keeps_valid_result(self):
        p = build("苏A·B8Q62", strict=True)
        self.assertTrue(p.is_valid)
        self.assertEqual(p.text, "苏AB8Q62")
        self.assertEqual(p.display, "苏A·B8Q62")

    def test_str_falls_back(self):
        self.assertEqual(str(build("苏A·B8Q62")), "苏A·B8Q62")
        self.assertEqual(str(build("")), "")

    def test_real_ocr_outputs_roundtrip(self):
        """回归基准: 旧 ocr_result.txt 里的真实输出必须原样还原。"""
        for expected in ("苏A·B8Q62", "鄂D·7B5Q2", "苏A·PL12A"):
            with self.subTest(plate=expected):
                p = build(expected)
                self.assertTrue(p.is_valid)
                self.assertEqual(p.display, expected)


class TestFormatForFile(unittest.TestCase):
    def test_strips_tabs_and_newlines(self):
        self.assertEqual(format_for_file("苏A\tB8Q62"), "苏A B8Q62")
        self.assertEqual(format_for_file("  苏A·B8Q62 \n"), "苏A·B8Q62")


class TestParseV2(unittest.TestCase):
    """PaddleOCR 2.x: [[ [box, (text, score)], ... ]]"""

    def test_single_line(self):
        # 结构: [[ [box, (text, score)], ... ], ... ]
        box = [[0, 0], [1, 0], [1, 1], [0, 1]]
        out = [[[box, ("苏A·B8Q62", 0.98)]]]
        text, conf = _parse_v2(out)
        self.assertEqual(text, "苏A·B8Q62")
        self.assertAlmostEqual(conf, 0.98)

    def test_multiple_lines_concatenated(self):
        out = [
            [[None, ("苏A", 0.9)]],
            [[None, ("B8Q62", 0.7)]],
        ]
        text, conf = _parse_v2(out)
        self.assertEqual(text, "苏AB8Q62")
        self.assertAlmostEqual(conf, 0.8)

    def test_empty_variants(self):
        for empty in (None, [], [[]]):
            self.assertEqual(_parse_v2(empty), ("", 0.0))

    def test_malformed_entries_skipped(self):
        out = [[[None, ()], [None, ["x"]], [None]]]
        self.assertEqual(_parse_v2(out)[0], "")


class _FakeResult:
    """模拟 PaddleOCR 3.x 的 Result 对象。"""

    def __init__(self, data):
        self._data = data

    @property
    def json(self):
        return self._data


class TestParseV3(unittest.TestCase):
    """PaddleOCR 3.x: Result 对象, .json 里带 rec_texts / rec_scores。"""

    def test_dict_under_res_key(self):
        # 实测 3.7.0 的真实结构
        data = {
            "input_path": "x.jpg",
            "res": {
                "rec_texts": ["苏A·B8Q62"],
                "rec_scores": [0.9984],
                "rec_polys": [],
            },
        }
        text, conf = _parse_v3([_FakeResult(data)])
        self.assertEqual(text, "苏A·B8Q62")
        self.assertAlmostEqual(conf, 0.9984, places=3)

    def test_flat_dict_without_res(self):
        text, conf = _parse_v3(
            [{"rec_texts": ["鄂D7B5Q2"], "rec_scores": [0.88]}]
        )
        self.assertEqual(text, "鄂D7B5Q2")
        self.assertAlmostEqual(conf, 0.88)

    def test_json_as_string(self):
        import json as _json

        payload = _json.dumps({"res": {"rec_texts": ["苏A·PL12A"], "rec_scores": [0.9]}})
        text, _ = _parse_v3([_FakeResult(payload)])
        self.assertEqual(text, "苏A·PL12A")

    def test_multiple_results(self):
        a = _FakeResult({"res": {"rec_texts": ["苏A"], "rec_scores": [0.9]}})
        b = _FakeResult({"res": {"rec_texts": ["B8Q62"], "rec_scores": [0.7]}})
        text, conf = _parse_v3([a, b])
        self.assertEqual(text, "苏AB8Q62")
        self.assertAlmostEqual(conf, 0.8)

    def test_rec_texts_attribute_fallback(self):
        res = types_Simple(rec_texts=["粤B12345"], rec_scores=[0.95])
        text, conf = _parse_v3([res])
        self.assertEqual(text, "粤B12345")
        self.assertAlmostEqual(conf, 0.95)

    def test_empty(self):
        for empty in (None, [], ()):
            self.assertEqual(_parse_v3(empty), ("", 0.0))

    def test_unknown_shape_does_not_raise(self):
        # 认不出来时返回空, 不能抛异常打断整批
        self.assertIsInstance(_parse_v3([object()]), tuple)


def types_Simple(**kw):
    from types import SimpleNamespace

    return SimpleNamespace(**kw)


class TestDetectApi(unittest.TestCase):
    def test_version_2(self):
        class P:
            __version__ = "2.7.0.0"

        self.assertEqual(_detect_paddle_api(P), "2.x")

    def test_version_3(self):
        class P:
            __version__ = "3.7.0"

        self.assertEqual(_detect_paddle_api(P), "3.x")

    def test_version_4_hypothetical(self):
        class P:
            __version__ = "4.0.0"

        self.assertEqual(_detect_paddle_api(P), "3.x")

    def test_no_version_falls_back_to_predicate(self):
        class WithPredict:
            def predict(self, *a, **k):
                pass

        class WithoutPredict:
            pass

        self.assertEqual(_detect_paddle_api(WithPredict), "3.x")
        self.assertEqual(_detect_paddle_api(WithoutPredict), "2.x")


class TestStubEngine(unittest.TestCase):
    def test_runs_without_paddle(self):
        import numpy as np

        from tl_vision.ocr import PlateOcr

        ocr = PlateOcr(engine="stub")
        self.assertEqual(ocr.api_version, "stub")
        p = ocr.read(np.zeros((40, 120, 3), dtype=np.uint8))
        self.assertFalse(p.is_valid)
        self.assertEqual(p.issue, "空结果")

    def test_unknown_engine_rejected(self):
        from tl_vision.ocr import OcrError, PlateOcr

        with self.assertRaises(OcrError):
            PlateOcr(engine="nonexistent")


class TestOcrConfig(unittest.TestCase):
    """ocr 小节的解析与校验 (config.py 本身不含 ROS, 可直接测)。"""

    def _load(self, overrides):
        import tempfile
        from pathlib import Path

        from tl_vision.config import load_config

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.yaml"
            path.write_text("vision_node:\n  ros__parameters:\n", encoding="utf-8")
            return load_config(path, overrides)

    def test_defaults(self):
        cfg = self._load(None)
        self.assertEqual(cfg.ocr.engine, "paddleocr")
        self.assertEqual(cfg.ocr.topics.plate_sub, "/tl_vision/plate_crop")
        self.assertEqual(cfg.ocr.topics.plate_text_pub, "/tl_vision/plate_text")
        self.assertEqual(cfg.ocr.queue_size, 1)
        self.assertFalse(cfg.ocr.strict)
        self.assertFalse(cfg.ocr.enable_mkldnn)

    def test_shipped_yaml_parses(self):
        """仓库里那份 yaml 必须能加载 —— 它是队友拿到包后第一个会碰的文件。"""
        from tl_vision.config import DEFAULT_CONFIG_PATH, load_config

        cfg = load_config(DEFAULT_CONFIG_PATH)
        self.assertEqual(cfg.ocr.engine, "paddleocr")
        self.assertEqual(cfg.ocr.topics.plate_sub, cfg.topics.plate_crop_pub)
        # 检测端和 OCR 端必须约定在同一个话题上, 否则永远收不到
        self.assertEqual(cfg.topics.plate_crop_pub, "/tl_vision/plate_crop")

    def test_overrides_applied(self):
        cfg = self._load({"ocr": {"engine": "stub", "strict": True, "min_conf": 0.8}})
        self.assertEqual(cfg.ocr.engine, "stub")
        self.assertTrue(cfg.ocr.strict)
        self.assertAlmostEqual(cfg.ocr.min_conf, 0.8)

    def test_topic_override_keeps_siblings(self):
        """只改 plate_text_pub 时, plate_sub / annotated_pub 不能被丢掉。"""
        cfg = self._load({"ocr": {"topics": {"plate_text_pub": "/my/text"}}})
        self.assertEqual(cfg.ocr.topics.plate_text_pub, "/my/text")
        self.assertEqual(cfg.ocr.topics.plate_sub, "/tl_vision/plate_crop")
        self.assertEqual(
            cfg.ocr.topics.annotated_pub, "/tl_vision/plate_text_image"
        )

    def test_rejects_self_subscription(self):
        """自订阅会形成无限回环, 必须在配置阶段就报错。"""
        with self.assertRaises(ValueError) as ctx:
            self._load({"ocr": {"topics": {"plate_sub": "/x", "plate_text_pub": "/x"}}})
        self.assertIn("回环", str(ctx.exception))

    def test_rejects_bad_engine(self):
        with self.assertRaises(ValueError):
            self._load({"ocr": {"engine": "tesseract"}})

    def test_rejects_out_of_range_min_conf(self):
        with self.assertRaises(ValueError):
            self._load({"ocr": {"min_conf": 1.5}})
        with self.assertRaises(ValueError):
            self._load({"ocr": {"min_conf": -0.1}})

    def test_rejects_bad_queue_size(self):
        with self.assertRaises(ValueError):
            self._load({"ocr": {"queue_size": 0}})

    def test_plate_crop_image_defaults_off(self):
        cfg = self._load(None)
        self.assertFalse(cfg.publishing.plate_crop_image)
        self.assertTrue(cfg.publishing.plate_crop)


if __name__ == "__main__":
    unittest.main()
