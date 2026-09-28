"""YOLO 推理核心 —— 不含任何 ROS 依赖。

设计原则:
    实时节点 (node.py)、离线批处理 (offline.py)、单元测试 (test/) 全部复用
    本模块。因此这里禁止 import rospy / cv_bridge, 也不允许出现硬编码路径。

类别名统一使用下划线小写 (red_on), 与 config/data.yaml 的 names 严格一致。
注意: 早期版本的 test_vision.py 用的是 "red on"(带空格), 两者不兼容,
本包统一为下划线风格。
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

LOG = logging.getLogger(__name__)

#: 类别 id -> 名称。顺序即 class_id 的含义, 与 config/data.yaml 保持一致。
DEFAULT_CLASS_NAMES: Tuple[str, ...] = (
    "resident",
    "stranger",
    "red_on",
    "red_off",
    "yellow_on",
    "yellow_off",
    "green_on",
    "green_off",
    "license_plate",
)

PLATE_CLASS = "license_plate"
PERSON_CLASSES = frozenset({"resident", "stranger"})
TRAFFIC_LIGHT_CLASSES = frozenset(
    {"red_on", "red_off", "yellow_on", "yellow_off", "green_on", "green_off"}
)

#: 灯色分组, 便于下游只关心"当前是什么颜色"
LIGHT_STATE_GROUPS: Mapping[str, str] = {
    "red_on": "red",
    "red_off": "red",
    "yellow_on": "yellow",
    "yellow_off": "yellow",
    "green_on": "green",
    "green_off": "green",
}


class DetectorError(RuntimeError):
    """权重缺失 / 加载失败等启动期错误。"""


@dataclass(frozen=True)
class Detection:
    """一个检测框。坐标为像素, 左上角为原点。"""

    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def xyxy(self) -> Tuple[float, float, float, float]:
        return (self.x1, self.y1, self.x2, self.y2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return max(0.0, self.width) * max(0.0, self.height)

    @property
    def is_traffic_light(self) -> bool:
        return self.class_name in TRAFFIC_LIGHT_CLASSES

    @property
    def is_person(self) -> bool:
        return self.class_name in PERSON_CLASSES

    @property
    def is_plate(self) -> bool:
        return self.class_name == PLATE_CLASS

    @property
    def light_state(self) -> Optional[str]:
        """红绿灯专用: 返回 "red"/"yellow"/"green"; 非灯色返回 None。"""
        return LIGHT_STATE_GROUPS.get(self.class_name)

    def to_dict(self) -> dict:
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "xyxy": [round(float(v), 1) for v in self.xyxy],
        }


@dataclass
class InferenceResult:
    """一帧的推理输出。"""

    detections: List[Detection] = field(default_factory=list)
    inference_ms: float = 0.0
    frame_width: int = 0
    frame_height: int = 0
    #: 本帧是否走了 force_full_image 调试桩, 而非真实模型输出
    forced: bool = False

    def __len__(self) -> int:
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)

    def by_class(self, *class_names: str) -> List[Detection]:
        wanted = set(class_names)
        return [d for d in self.detections if d.class_name in wanted]

    @property
    def plates(self) -> List[Detection]:
        return self.by_class(PLATE_CLASS)

    @property
    def traffic_lights(self) -> List[Detection]:
        return [d for d in self.detections if d.is_traffic_light]

    @property
    def people(self) -> List[Detection]:
        return [d for d in self.detections if d.is_person]

    def dominant_light(self) -> Optional[Detection]:
        """最显著的灯色 —— 置信度最高的红绿灯框。无人车过闸常用。"""
        lights = self.traffic_lights
        return max(lights, key=lambda d: d.confidence) if lights else None

    def summary(self) -> str:
        if not self.detections:
            return "无目标"
        return ", ".join(
            f"{d.class_name}@{d.confidence:.2f}" for d in self.detections
        )


@dataclass
class ForceRule:
    """force_full_image 调试规则的解析结果。"""

    match: str
    class_name: str


def parse_force_rules(raw: Optional[Iterable[Mapping]]) -> List[ForceRule]:
    """把 yaml 里的规则列表转成 ForceRule, 顺带做一次名称校验。"""
    rules: List[ForceRule] = []
    for item in raw or []:
        match = str(item.get("match", "")).strip()
        class_name = str(item.get("class_name", "")).strip()
        if not match or not class_name:
            continue
        if class_name not in DEFAULT_CLASS_NAMES:
            LOG.warning(
                "force_full_image 规则 %r 的 class_name=%r 不在类别表中, 已忽略",
                match,
                class_name,
            )
            continue
        rules.append(ForceRule(match=match, class_name=class_name))
    return rules


class YoloDetector:
    """YOLOv8 推理封装。

    线程安全性: 单次 infer() 内部无共享可变状态, 但 ultralytics 的 predictor
    内部有缓存, **不要** 从多线程并发调用同一个实例。ROS 回调请用
    ``threading.Lock`` 串行化 (node.py 已处理)。
    """

    def __init__(
        self,
        weights: str | Path,
        class_names: Sequence[str] = DEFAULT_CLASS_NAMES,
        device: Optional[str] = None,
        imgsz: int = 640,
        conf: float = 0.5,
        iou: float = 0.45,
        max_det: int = 100,
        class_filter: Optional[Iterable[int]] = None,
        force_rules: Optional[Sequence[ForceRule]] = None,
        warmup: bool = True,
    ) -> None:
        self.weights = Path(weights)
        self.class_names = tuple(class_names)
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.max_det = int(max_det)
        self.class_filter = (
            {int(c) for c in class_filter} if class_filter else None
        )
        self.force_rules = tuple(force_rules or ())
        self._forced_warned = False

        if len(self.class_names) != len(set(self.class_names)):
            raise ValueError("class_names 存在重复项")
        self._model = self._load_model(device, half=None)
        if warmup:
            self.warmup()

    # ------------------------------------------------------------------ 加载
    def _load_model(self, device: Optional[str], half: Optional[bool]):
        if not self.weights.is_file():
            raise DetectorError(
                f"找不到权重文件: {self.weights}\n"
                f"请把 best.pt 放到 config 相对路径下, 或设置环境变量 "
                f"TL_VISION_WEIGHTS 指向它。"
            )
        try:
            from ultralytics import YOLO  # 延迟导入: 允许 --help 无 ROS 环境跑
        except ImportError as exc:  # pragma: no cover
            raise DetectorError(
                "未安装 ultralytics, 请执行: pip install -r requirements.txt"
            ) from exc

        LOG.info("加载权重 %s (device=%s)", self.weights, device or "auto")
        kwargs = {"verbose": False}
        if device:
            kwargs["device"] = device
        try:
            return YOLO(str(self.weights), **kwargs)
        except Exception as exc:  # pragma: no cover
            raise DetectorError(f"加载权重失败: {exc}") from exc

    def warmup(self, rounds: int = 1) -> None:
        """空跑几张假图, 把 CUDA kernel / 显存分配的开销挪到启动阶段。

        ROS 节点启动后第一帧延迟通常有 1~2 秒, 预热后可以压到几十毫秒。
        """
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        for _ in range(max(1, rounds)):
            self._model.predict(
                dummy,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                max_det=self.max_det,
                classes=sorted(self.class_filter) if self.class_filter else None,
                verbose=False,
            )
        LOG.info("预热完成 (%d 轮, imgsz=%d)", rounds, self.imgsz)

    # ------------------------------------------------------------------ 推理
    def infer(self, image_bgr: np.ndarray, source_name: str = "") -> InferenceResult:
        """对一张 BGR 图做推理。

        Args:
            image_bgr: HxWx3 uint8, OpenCV BGR 顺序。
            source_name: 离线模式下用于匹配 force 规则的文件名。
        """
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("输入图像为空")
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise TypeError(
                f"期望 HxWx3 BGR 图像, 实际 shape={image_bgr.shape}"
            )

        h, w = image_bgr.shape[:2]

        forced = self._match_force_rule(source_name, w, h)
        if forced is not None:
            return forced

        t0 = time.perf_counter()
        raw = self._model.predict(
            image_bgr,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            max_det=self.max_det,
            classes=sorted(self.class_filter) if self.class_filter else None,
            verbose=False,
        )[0]
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        return InferenceResult(
            detections=self._to_detections(raw),
            inference_ms=elapsed_ms,
            frame_width=int(w),
            frame_height=int(h),
        )

    def _to_detections(self, raw) -> List[Detection]:
        out: List[Detection] = []
        boxes = getattr(raw, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return out
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i].tolist())
            name = (
                self.class_names[cls_id]
                if 0 <= cls_id < len(self.class_names)
                else f"class_{cls_id}"
            )
            out.append(
                Detection(
                    class_id=cls_id,
                    class_name=name,
                    confidence=conf,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        return _sorted_by_group(out)

    # ------------------------------------------------------- force 调试桩
    def _match_force_rule(
        self, source_name: str, width: int, height: int
    ) -> Optional[InferenceResult]:
        """命中 force 规则时构造「整图 = 指定类别」的假结果, 未命中返回 None。"""
        if not self.force_rules or not source_name:
            return None
        for rule in self.force_rules:
            if rule.match not in source_name:
                continue
            if not self._forced_warned:
                LOG.warning(
                    "force_full_image_boxes 已启用, 命中 %r 的图片将跳过模型 "
                    "直接返回整图框 —— 这不是模型预测结果, 不可用于评估!",
                    rule.match,
                )
                self._forced_warned = True
            margin = 10
            return InferenceResult(
                detections=[
                    Detection(
                        class_id=self.class_names.index(rule.class_name),
                        class_name=rule.class_name,
                        confidence=0.9,
                        x1=float(margin),
                        y1=float(margin),
                        x2=float(max(margin + 1, width - margin)),
                        y2=float(max(margin + 1, height - margin)),
                    )
                ],
                inference_ms=0.0,
                frame_width=int(width),
                frame_height=int(height),
                forced=True,
            )
        return None


def _sorted_by_group(dets: List[Detection]) -> List[Detection]:
    """按 行人 -> 红绿灯 -> 车牌 分组, 组内置信度降序。

    下游 (播报/闸机) 习惯取 detections[0] 为最高优先级目标, 固定顺序后
    该约定才成立。
    """

    def rank(d: Detection) -> int:
        if d.is_person:
            return 0
        if d.is_traffic_light:
            return 1
        if d.is_plate:
            return 2
        return 3

    return sorted(dets, key=lambda d: (rank(d), -d.confidence))
