"""车牌 OCR —— 支持 PaddleOCR 2.x / 3.x, 并预留后端切换。

为什么要自己包一层:
    PaddleOCR 2.x 和 3.x 的构造参数与调用方法都变了 (use_gpu -> device,
    ocr(img, cls=True) -> predict(img)), 而队友机器上装的是哪个版本不可控。
    这里做运行时探测, 两套都能跑, 免得为了升级依赖手工改代码。

后端可插拔, 便于在没装 Paddle 的机器上跑单测:
    >>> PlateOcr(engine="stub").read_file("x.jpg")   # 不碰任何外部库
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import List, Optional, Union

import numpy as np

from .plate_text import PlateText
from .plate_text import build as build_plate_text

LOG = logging.getLogger("tl_vision.ocr")

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


class OcrError(RuntimeError):
    """OCR 引擎不可用或初始化失败。"""


@dataclass
class OcrStats:
    total: int = 0
    succeeded: int = 0
    failed: int = 0
    valid: int = 0
    invalid: int = 0
    total_ms: float = 0.0

    @property
    def avg_ms(self) -> float:
        return self.total_ms / self.succeeded if self.succeeded else 0.0

    def as_dict(self) -> dict:
        return {
            "total": self.total,
            "succeeded": self.succeeded,
            "failed": self.failed,
            "valid": self.valid,
            "invalid": self.invalid,
            "avg_ms": round(self.avg_ms, 2),
        }


class PlateOcr:
    """车牌识别。``engine`` 支持 ``paddleocr`` (默认) / ``stub``。"""

    def __init__(
        self,
        engine: str = "paddleocr",
        lang: str = "ch",
        use_gpu: bool = False,
        use_angle_cls: bool = True,
        strict: bool = False,
        min_conf: float = 0.0,
        enable_mkldnn: bool = False,
        recognition_only: bool = False,
        model_name: Optional[str] = None,
        model_dir: Optional[str] = None,
        cpu_threads: int = 2,
    ) -> None:
        self.engine = engine
        self.lang = lang
        self.use_gpu = use_gpu
        self.use_angle_cls = use_angle_cls
        self.strict = strict
        self.min_conf = float(min_conf)
        self.enable_mkldnn = enable_mkldnn
        self.recognition_only = recognition_only
        self.model_name, self.model_dir = model_name, model_dir
        self.cpu_threads = int(cpu_threads)
        self._api_version: Optional[str] = None
        self._ocr = self._create_engine()

    @property
    def api_version(self) -> str:
        """实际探测到的 PaddleOCR API 代次 ("2.x" / "3.x" / "stub")。"""
        return self._api_version or "unknown"

    # ------------------------------------------------------------ 引擎构建
    def _create_engine(self):
        if self.engine == "stub":
            self._api_version = "stub"
            return _StubEngine()
        if self.engine != "paddleocr":
            raise OcrError(f"未知 OCR 引擎 {self.engine!r}, 可选: paddleocr, stub")

        try:
            if self.recognition_only:
                from paddleocr import TextRecognition
                self._api_version = 'recognition3.x'
                return TextRecognition(
                    model_name=self.model_name or 'PP-OCRv5_mobile_rec',
                    model_dir=self.model_dir, device='gpu' if self.use_gpu else 'cpu',
                    enable_mkldnn=self.enable_mkldnn, cpu_threads=self.cpu_threads)
            from paddleocr import PaddleOCR
        except ImportError as exc:
            raise OcrError(
                "未安装 paddleocr。安装方式:\n"
                "  pip install paddlepaddle paddleocr\n"
                "国内源更快: pip install -i https://paddlepaddle.org.cn/packages/stable/ "
                "paddlepaddle\n"
                "如果只是要跑检测、不需要识别车牌, 可以不装 —— 检测部分不依赖它。"
            ) from exc

        api = _detect_paddle_api(PaddleOCR)
        self._api_version = api
        try:
            if api == "3.x":
                # PaddleOCR 3.x: device 取代 use_gpu, use_textline_orientation 取代
                # use_angle_cls
                kwargs = {
                    "lang": self.lang,
                    "device": "gpu" if self.use_gpu else "cpu",
                }
                if self.use_angle_cls:
                    kwargs["use_textline_orientation"] = True
                # paddlepaddle 3.3.x + oneDNN 在 PIR 新执行器下会抛
                # "(Unimplemented) ConvertPirAttribute2RuntimeAttribute not support",
                # 识别直接全挂。3.x 上默认关掉 oneDNN 换取可用性。
                # 2.x 不受影响 (那个组合是好的), 保持原样不动。
                if not self.enable_mkldnn:
                    kwargs["enable_mkldnn"] = False
            else:
                # PaddleOCR 2.x —— 原脚本的组合, 已验证可用, 不加额外参数
                kwargs = {
                    "lang": self.lang,
                    "use_angle_cls": self.use_angle_cls,
                    "use_gpu": self.use_gpu,
                }
            return PaddleOCR(**kwargs)
        except TypeError as exc:
            raise OcrError(
                f"构造 PaddleOCR 失败 (API {api}): {exc}\n"
                f"当前 paddleocr 版本可能与预期不符, 请反馈具体版本号。"
            ) from exc
        except Exception as exc:
            raise OcrError(f"构造 PaddleOCR 失败 (API {api}): {exc}") from exc

    # --------------------------------------------------------------- 识别
    def read(self, image_bgr: np.ndarray) -> PlateText:
        """对已加载的 BGR 裁剪图做识别。"""
        t0 = time.perf_counter()
        raw, conf = self._run(image_bgr)
        elapsed = (time.perf_counter() - t0) * 1000.0
        plate = build_plate_text(raw, confidence=conf, strict=self.strict)
        if plate.is_valid and plate.confidence < self.min_conf:
            plate = replace(plate, issue='LOW_OCR_CONFIDENCE',
                            text='' if self.strict else plate.text,
                            display='' if self.strict else plate.display)
        LOG.debug("OCR %.1f ms -> %r", elapsed, plate.text)
        return plate

    def read_file(self, path: Union[str, Path]) -> PlateText:
        """从文件读图再识别。"""
        import cv2

        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise OcrError(f"图片读取失败: {path}")
        return self.read(image)

    def read_dir(
        self, directory: Union[str, Path], pattern: str = "*"
    ) -> List[tuple]:
        """批处理目录下所有图片, 返回 ``[(Path, PlateText, 耗时ms), ...]``。"""
        d = Path(directory)
        if not d.is_dir():
            raise OcrError(f"目录不存在: {d}")
        files = [
            p
            for p in sorted(d.glob(pattern))
            if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
        ]
        if not files:
            raise OcrError(f"目录下没有支持的图片: {d}")

        results = []
        for path in files:
            t0 = time.perf_counter()
            try:
                plate = self.read_file(path)
                err = None
            except Exception as exc:  # 单张失败不应中断整批
                plate = PlateText(raw="", issue=f"识别失败: {exc}")
                err = exc
            elapsed = (time.perf_counter() - t0) * 1000.0
            if err is None:
                LOG.info("%-52s %-14s %s", path.name, plate.display or "-", plate.issue or "")
            else:
                LOG.error("%-52s 失败: %s", path.name, err)
            results.append((path, plate, elapsed))
        return results

    # --------------------------------------------------------- 引擎调用
    def _run(self, image_bgr: np.ndarray):
        """把两代 API 的差异收敛到这里。返回 (文本, 置信度)。"""
        if isinstance(self._ocr, _StubEngine):
            return self._ocr.ocr(image_bgr)

        if self.recognition_only:
            return _parse_v3(self._ocr.predict(input=image_bgr, batch_size=1))

        if self._api_version == "3.x":
            # 3.x: predict() 返回 Result 对象列表, 需取 .json / dict 形式
            out = self._ocr.predict(image_bgr)
            return _parse_v3(out)

        # 2.x: ocr(img, cls=True) -> [[ [box, (text, score)], ... ]]
        out = self._ocr.ocr(image_bgr, cls=True)
        return _parse_v2(out)


def _detect_paddle_api(PaddleOCR) -> str:
    """判断装的是 2.x 还是 3.x。"""
    major = str(getattr(PaddleOCR, "__version__", "")).split(".")[0]
    if major.isdigit():
        return "3.x" if int(major) >= 3 else "2.x"
    # 没暴露版本号时按特征判断
    return "3.x" if hasattr(PaddleOCR, "predict") else "2.x"


def _parse_v2(out) -> tuple:
    """PaddleOCR 2.x 返回值 -> (文本, 平均置信度)。"""
    if not out:
        return "", 0.0
    texts: List[str] = []
    scores: List[float] = []
    for line in out:
        if not line:
            continue
        for item in line:
            # 单条结果形如 [box, (text, score)]; 结构不规整就跳过, 绝不能因为
            # 一条脏数据把整批 OCR 打断
            if not item or len(item) < 2:
                continue
            pair = item[1]
            if not isinstance(pair, (list, tuple)) or len(pair) < 2:
                continue
            word, score = pair[0], pair[1]
            if word:
                texts.append(str(word))
                try:
                    scores.append(float(score))
                except (TypeError, ValueError):
                    pass
    conf = sum(scores) / len(scores) if scores else 0.0
    return "".join(texts), conf


def _parse_v3(out) -> tuple:
    """PaddleOCR 3.x 返回值 -> (文本, 平均置信度)。

    3.x 的 Result 对象结构随小版本有差异, 这里按几种常见形态依次尝试,
    实在认不出来就退回"把所有字符串拼起来"的最保守策略。
    """
    if out is None:
        return "", 0.0

    results = out if isinstance(out, (list, tuple)) else [out]
    texts: List[str] = []
    scores: List[float] = []

    for res in results:
        data = None
        # 优先用官方提供的结构化输出
        if hasattr(res, "json"):
            try:
                data = res.json  # 属性
            except Exception:
                data = None
            if callable(data):
                try:
                    data = data()
                except Exception:
                    data = None
        elif isinstance(res, dict):
            data = res

        if isinstance(data, str):  # 有些版本 json 是字符串
            import json as _json

            try:
                data = _json.loads(data)
            except ValueError:
                data = None

        if isinstance(data, dict):
            texts.extend(_harvest_v3_dict(data, scores))
        elif data is not None:
            texts.extend(_harvest_strings(data, scores))

    if not texts and results:
        # 兜底: 直接读对象属性 (某些版本 rec_texts 只作为属性暴露)
        for res in results:
            texts.extend(_harvest_strings(getattr(res, "rec_texts", None) or [], scores))
            if not texts:
                continue
            for s in getattr(res, "rec_scores", None) or []:
                try:
                    scores.append(float(s))
                except (TypeError, ValueError):
                    pass
            break

    conf = sum(scores) / len(scores) if scores else 0.0
    return "".join(texts), conf


def _harvest_v3_dict(data: dict, scores: List[float]) -> List[str]:
    """从 3.x 的 dict 结构里取出 rec_texts / rec_scores。"""
    # 结构大致是 {"res": {...}} 或直接 {...}
    for node in (data.get("res", data), data):
        if not isinstance(node, dict):
            continue
        if 'rec_text' in node:
            try:
                scores.append(float(node.get('rec_score',0.0)))
            except (ValueError,TypeError):
                pass
            return [str(node['rec_text'])] if node['rec_text'] else []
        rec = node.get("rec_texts")
        if rec is None:
            continue
        texts = [str(t) for t in rec if t]
        sco = node.get("rec_scores") or []
        for s in sco:
            try:
                scores.append(float(s))
            except (TypeError, ValueError):
                pass
        return texts
    return []


def _harvest_strings(obj, scores: List[float]) -> List[str]:
    if obj is None:
        return []
    if isinstance(obj, str):
        return [obj]
    out: List[str] = []
    try:
        for item in obj:
            if isinstance(item, (list, tuple)) and item:
                if isinstance(item[0], (int, float)):
                    scores.append(float(item[0]))
                if len(item) > 1:
                    out.append(str(item[1]))
            elif item:
                out.extend(_harvest_strings(item, scores))
    except TypeError:
        return [str(obj)]
    return out


class _StubEngine:
    """不依赖任何外部库的假引擎, 供单测使用。"""

    def __init__(self) -> None:
        self.calls = 0

    def ocr(self, image_bgr: np.ndarray):
        self.calls += 1
        return "", 0.0
