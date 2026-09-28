"""配置解析 —— 不含 ROS 依赖。

作用: 把 config/vision_config.yaml 变成一个带默认值的 dataclass, 让
node.py / offline.py / 单测 都能在没有 ROS 的情况下读同一份配置。

优先级约定:
    yaml 文件  <  环境变量 TL_VISION_WEIGHTS 覆盖权重
ROS 节点里 yaml 会被 rosparam 再覆盖一层 (见 node.py)。
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import yaml

from .detector import DEFAULT_CLASS_NAMES, parse_force_rules

LOG = logging.getLogger(__name__)

WEIGHTS_ENV_VAR = "TL_VISION_WEIGHTS"
PKG_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_CONFIG_PATH = PKG_ROOT / "config" / "vision_config.yaml"


@dataclass
class ModelCfg:
    weights: str = ""
    device: Optional[str] = None
    imgsz: int = 640
    half: bool = False


@dataclass
class InferenceCfg:
    conf_threshold: float = 0.5
    iou_threshold: float = 0.45
    max_detections: int = 100
    class_filter: List[int] = field(default_factory=list)


@dataclass
class TopicsCfg:
    image_sub: str = "/camera/image_raw"
    detections_pub: str = "/tl_vision/detections"
    annotated_image_pub: str = "/tl_vision/image_annotated"
    #: 结构化的车牌裁剪 (PlateCrop: 位置 + 图像), OCR 节点订阅这个
    plate_crop_pub: str = "/tl_vision/plate_crop"
    #: 裸裁剪图 (sensor_msgs/Image), 只为 rqt_image_view 看图方便
    plate_crop_image_pub: str = "/tl_vision/plate_crop_image"


@dataclass
class PublishingCfg:
    annotated_image: bool = True
    plate_crop: bool = True
    #: 是否额外发一份裸 Image 裁剪图。PlateCrop 里已经带了图像, 这份纯粹
    #: 是给 rqt_image_view / image_view 之类的通用工具看的, 默认关闭省带宽。
    plate_crop_image: bool = False
    plate_crop_margin: int = 6
    stamp_from_input: bool = True


@dataclass
class OcrTopicsCfg:
    plate_sub: str = "/tl_vision/plate_crop"
    plate_text_pub: str = "/tl_vision/plate_text"
    #: 把识别结果画回裁剪图后发布 (绿框=合规, 红框=不合规)
    annotated_pub: str = "/tl_vision/plate_text_image"


@dataclass
class OcrCfg:
    """车牌 OCR 节点 (``tl_vision_ocr_node``) 的配置。

    单独一小节而不是塞进 inference: 检测要 torch/ultralytics, OCR 要
    paddleocr/paddlepaddle, 这两套依赖的 numpy、opencv、Python 版本互相冲突,
    装不到同一个环境里。拆成两个节点后各跑各的 conda 环境, 只通过话题通信。
    """

    engine: str = "paddleocr"  # paddleocr / stub
    lang: str = "ch"
    use_gpu: bool = False
    use_angle_cls: bool = True
    #: 校验不通过时把 text 清空。下游若是"非空即信任", 就该开。
    strict: bool = False
    #: 低于该 OCR 置信度的结果标为不合规; 0 = 不额外过滤
    min_conf: float = 0.0
    #: paddlepaddle 3.3.x + oneDNN 在 PIR 新执行器下会抛 Unimplemented,
    #: 识别直接全挂。3.x 上默认关掉; 2.x 不受影响 (那个组合是好的)。
    enable_mkldnn: bool = False
    #: 只发布通过校验的结果 (不合规的只在日志里报)
    drop_invalid: bool = False
    #: OCR 很慢, 队列保持 1 —— 处理不过来时丢旧帧, 不要让延迟越堆越高
    queue_size: int = 1
    log_every_n: int = 10
    topics: OcrTopicsCfg = field(default_factory=OcrTopicsCfg)


@dataclass
class DebugCfg:
    force_full_image_boxes: bool = False
    force_full_image_rules: List[Dict[str, str]] = field(default_factory=list)


@dataclass
class VisionConfig:
    model: ModelCfg = field(default_factory=ModelCfg)
    inference: InferenceCfg = field(default_factory=InferenceCfg)
    topics: TopicsCfg = field(default_factory=TopicsCfg)
    publishing: PublishingCfg = field(default_factory=PublishingCfg)
    ocr: OcrCfg = field(default_factory=OcrCfg)
    debug: DebugCfg = field(default_factory=DebugCfg)
    frame_id: str = ""
    queue_size: int = 1
    log_every_n: int = 30
    #: yaml 文件所在目录, 用于解析相对路径
    base_dir: Path = field(default_factory=lambda: PKG_ROOT)

    # ------------------------------------------------------------ 权重解析
    def resolve_weights(self) -> Path:
        """按 yaml -> 环境变量 的顺序确定权重绝对路径。

        相对路径相对于 base_dir 解析, 这样整包换机器不用改配置。
        """
        raw = (self.model.weights or "").strip()
        source = "config"
        if not raw:
            raw = os.environ.get(WEIGHTS_ENV_VAR, "").strip()
            source = "env"
        if not raw:
            raise FileNotFoundError(
                f"未指定模型权重。请在 {self.base_dir}/config/vision_config.yaml "
                f"的 model.weights 填路径, 或设置环境变量 {WEIGHTS_ENV_VAR}=/path/to/best.pt"
            )

        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = (self.base_dir / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(
                f"权重文件不存在: {path}  (来源: {source})\n"
                f"模型权重不入 git, 请自行拷贝或从训练机获取。"
            )
        LOG.debug("权重解析自 %s: %s", source, path)
        return path

    def resolve_device(self) -> Optional[str]:
        dev = (self.model.device or "").strip()
        if dev.lower() in ("", "auto", "none", "null"):
            return None
        return dev


def _as_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _merge(base: Mapping[str, Any], override: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    """递归合并 —— 覆盖嵌套小节时必须保留同级未提及的键。

    例如只覆盖 ``debug.force_full_image_boxes`` 时, 不能把 yaml 里的
    ``debug.force_full_image_rules`` 整块丢掉。
    """
    out: Dict[str, Any] = dict(base)
    for key, val in (override or {}).items():
        if isinstance(val, Mapping) and isinstance(out.get(key), Mapping):
            out[key] = _merge(out[key], val)
        else:
            out[key] = val
    return out


def load_config(
    path: Optional[str | Path] = None,
    overrides: Optional[Mapping[str, Any]] = None,
) -> VisionConfig:
    """读取并合并配置。

    Args:
        path: yaml 路径, 省略则用 config/vision_config.yaml
        overrides: 顶层小节的覆盖字典 (ROS 参数注入用), 形如
                   ``{"inference": {"conf_threshold": 0.3}}``
    """
    cfg_path = Path(path).expanduser() if path else DEFAULT_CONFIG_PATH
    if not cfg_path.is_file():
        raise FileNotFoundError(f"配置文件不存在: {cfg_path}")

    # utf-8-sig: 容忍 Windows 编辑器 (记事本/VSCode 部分设置) 写出的 BOM。
    # 用纯 utf-8 读会残留 U+FEFF, 导致 yaml 解析直接抛 ParserError。
    with cfg_path.open("r", encoding="utf-8-sig") as fh:
        raw = yaml.safe_load(fh) or {}

    # 三种布局都要支持:
    #   a) vision_node: {ros__parameters: {...}}   <- ROS 标准布局
    #   b) {ros__parameters: {...}}                <- 去掉节点名一层
    #   c) {...}                                   <- 小节直接写在顶层
    for candidate in (
        raw.get("vision_node"),
        raw,
    ):
        if isinstance(candidate, Mapping) and "ros__parameters" in candidate:
            raw = candidate["ros__parameters"] or {}
            break
    if not isinstance(raw, Mapping):
        raise TypeError(f"配置文件格式无法识别: {cfg_path}")
    raw = _merge(raw, overrides)

    model = _as_dict(raw.get("model"))
    inference = _as_dict(raw.get("inference"))
    topics = _as_dict(raw.get("topics"))
    publishing = _as_dict(raw.get("publishing"))
    ocr = _as_dict(raw.get("ocr"))
    ocr_topics = _as_dict(ocr.get("topics"))
    debug = _as_dict(raw.get("debug"))

    cfg = VisionConfig(
        model=ModelCfg(
            weights=str(model.get("weights", "") or ""),
            device=model.get("device") or None,
            imgsz=int(model.get("imgsz", 640)),
            half=bool(model.get("half", False)),
        ),
        inference=InferenceCfg(
            conf_threshold=float(inference.get("conf_threshold", 0.5)),
            iou_threshold=float(inference.get("iou_threshold", 0.45)),
            max_detections=int(inference.get("max_detections", 100)),
            class_filter=list(inference.get("class_filter") or []),
        ),
        topics=TopicsCfg(
            image_sub=str(topics.get("image_sub", TopicsCfg.image_sub)),
            detections_pub=str(topics.get("detections_pub", TopicsCfg.detections_pub)),
            annotated_image_pub=str(
                topics.get("annotated_image_pub", TopicsCfg.annotated_image_pub)
            ),
            plate_crop_pub=str(topics.get("plate_crop_pub", TopicsCfg.plate_crop_pub)),
            plate_crop_image_pub=str(
                topics.get("plate_crop_image_pub", TopicsCfg.plate_crop_image_pub)
            ),
        ),
        publishing=PublishingCfg(
            annotated_image=bool(publishing.get("annotated_image", True)),
            plate_crop=bool(publishing.get("plate_crop", True)),
            plate_crop_image=bool(publishing.get("plate_crop_image", False)),
            plate_crop_margin=int(publishing.get("plate_crop_margin", 6)),
            stamp_from_input=bool(publishing.get("stamp_from_input", True)),
        ),
        ocr=OcrCfg(
            engine=str(ocr.get("engine", "paddleocr")),
            lang=str(ocr.get("lang", "ch")),
            use_gpu=bool(ocr.get("use_gpu", False)),
            use_angle_cls=bool(ocr.get("use_angle_cls", True)),
            strict=bool(ocr.get("strict", False)),
            min_conf=float(ocr.get("min_conf", 0.0)),
            enable_mkldnn=bool(ocr.get("enable_mkldnn", False)),
            drop_invalid=bool(ocr.get("drop_invalid", False)),
            queue_size=int(ocr.get("queue_size", 1)),
            log_every_n=int(ocr.get("log_every_n", 10)),
            topics=OcrTopicsCfg(
                plate_sub=str(ocr_topics.get("plate_sub", OcrTopicsCfg.plate_sub)),
                plate_text_pub=str(
                    ocr_topics.get("plate_text_pub", OcrTopicsCfg.plate_text_pub)
                ),
                annotated_pub=str(
                    ocr_topics.get("annotated_pub", OcrTopicsCfg.annotated_pub)
                ),
            ),
        ),
        debug=DebugCfg(
            force_full_image_boxes=bool(debug.get("force_full_image_boxes", False)),
            force_full_image_rules=list(debug.get("force_full_image_rules") or []),
        ),
        frame_id=str(raw.get("frame_id", "") or ""),
        queue_size=int(raw.get("queue_size", 1)),
        log_every_n=int(raw.get("log_every_n", 30)),
        base_dir=cfg_path.resolve().parent.parent,
    )

    _validate(cfg)
    return cfg


def _validate(cfg: VisionConfig) -> None:
    if not 0.0 <= cfg.inference.conf_threshold <= 1.0:
        raise ValueError(
            f"inference.conf_threshold 必须在 [0,1], 当前 {cfg.inference.conf_threshold}"
        )
    if not 0.0 <= cfg.inference.iou_threshold <= 1.0:
        raise ValueError(
            f"inference.iou_threshold 必须在 [0,1], 当前 {cfg.inference.iou_threshold}"
        )
    if cfg.model.imgsz <= 0 or cfg.model.imgsz % 32 != 0:
        raise ValueError(
            f"model.imgsz 必须是 32 的正整数倍 (YOLO 下采样要求), 当前 {cfg.model.imgsz}"
        )
    if cfg.inference.max_detections <= 0:
        raise ValueError(
            f"inference.max_detections 必须为正, 当前 {cfg.inference.max_detections}"
        )
    for cid in cfg.inference.class_filter:
        if not 0 <= cid < len(DEFAULT_CLASS_NAMES):
            raise ValueError(
                f"inference.class_filter 含越界类别 id {cid} "
                f"(合法范围 0-{len(DEFAULT_CLASS_NAMES) - 1})"
            )
    if cfg.ocr.engine not in ("paddleocr", "stub"):
        raise ValueError(
            f"ocr.engine 只能是 paddleocr 或 stub, 当前 {cfg.ocr.engine!r}"
        )
    if not 0.0 <= cfg.ocr.min_conf <= 1.0:
        raise ValueError(
            f"ocr.min_conf 必须在 [0,1], 当前 {cfg.ocr.min_conf}"
        )
    if cfg.ocr.queue_size <= 0:
        raise ValueError(f"ocr.queue_size 必须为正, 当前 {cfg.ocr.queue_size}")
    if cfg.ocr.topics.plate_sub == cfg.ocr.topics.plate_text_pub:
        # 自己订阅自己 = 无限回环, 会把节点打满 CPU
        raise ValueError(
            "ocr.topics.plate_sub 与 plate_text_pub 不能是同一个话题 "
            "(会造成自订阅回环)"
        )
    if cfg.publishing.plate_crop_margin < 0:
        raise ValueError(
            f"publishing.plate_crop_margin 不能为负, 当前 {cfg.publishing.plate_crop_margin}"
        )
    if cfg.debug.force_full_image_boxes and not cfg.debug.force_full_image_rules:
        LOG.warning(
            "debug.force_full_image_boxes=true 但未配置任何规则, 该开关不会生效"
        )
    # 提前解析一次, 让配置错误在启动时暴露而不是第一帧
    parse_force_rules(cfg.debug.force_full_image_rules)
