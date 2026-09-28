"""tl_vision ROS1 实时检测节点。

职责: 订阅 sensor_msgs/Image -> YOLO 推理 -> 发布 DetectionArray,
可选发布画框图与车牌裁剪图 (PlateCrop)。

    rosrun tl_vision tl_vision_node
    rosrun tl_vision tl_vision_node _conf:=0.35
    rosrun tl_vision tl_vision_node _config:=/path/to/my.yaml

车牌识别不在本节点内做 —— 见 ocr_node.py 的说明。检测端只负责把车牌裁出来
发到 /tl_vision/plate_crop, 至于谁来读是 OCR 节点的事。

线程模型: rospy 回调跑在各自线程, 但 ultralytics predictor 内部有共享缓存,
因此用一把锁把推理串行化。图像队列深度默认 1 (只要最新帧), 避免小机器人
上因处理不过来而堆积延迟。
"""
from __future__ import annotations

import logging
import sys
import threading
from typing import Any, Dict

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tl_vision.config import load_config
from tl_vision.detector import (
    DEFAULT_CLASS_NAMES,
    Detection,
    DetectorError,
    InferenceResult,
    YoloDetector,
    parse_force_rules,
)

# 生成的 ROS 消息 (由 catkin 的 message_generation 产出)
from tl_vision.msg import Detection as DetectionMsg
from tl_vision.msg import DetectionArray, PlateCrop
from tl_vision.visualize import annotate, crop_plate

LOG = logging.getLogger("tl_vision.node")


class VisionNode:
    def __init__(self) -> None:
        cfg, overrides = self._load_params()
        self.cfg = cfg
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame_count = 0
        self._force_warned = False

        self._setup_logging(overrides)

        LOG.info("正在加载模型 ... (首次启动可能需要十几秒)")
        try:
            weights = cfg.resolve_weights()
            self.detector = YoloDetector(
                weights=weights,
                device=cfg.resolve_device(),
                imgsz=cfg.model.imgsz,
                conf=cfg.inference.conf_threshold,
                iou=cfg.inference.iou_threshold,
                max_det=cfg.inference.max_detections,
                class_filter=cfg.inference.class_filter,
                force_rules=(
                    parse_force_rules(cfg.debug.force_full_image_rules)
                    if cfg.debug.force_full_image_boxes
                    else ()
                ),
            )
        except (DetectorError, FileNotFoundError, ValueError) as exc:
            LOG.fatal("模型初始化失败: %s", exc)
            raise SystemExit(1) from exc
        except Exception as exc:  # 依赖缺失 / CUDA 不可用等
            LOG.fatal("加载模型时发生未预期错误: %s", exc)
            raise SystemExit(1) from exc
        LOG.info("模型就绪, %d 类检测已加载", len(DEFAULT_CLASS_NAMES))

        self.detections_pub = rospy.Publisher(
            cfg.topics.detections_pub, DetectionArray, queue_size=1
        )
        self.annotated_pub = (
            rospy.Publisher(cfg.topics.annotated_image_pub, Image, queue_size=1)
            if cfg.publishing.annotated_image
            else None
        )
        # 结构化裁剪: 自带位置, OCR 节点订阅这个。不带位置的话下游只能靠
        # "同帧内第 N 张图 = 第 N 个检测框" 去猜, 队列一丢消息就错位了。
        self.plate_pub = (
            rospy.Publisher(cfg.topics.plate_crop_pub, PlateCrop, queue_size=1)
            if cfg.publishing.plate_crop
            else None
        )
        # 裸 Image 裁剪, 只给 rqt_image_view 之类通用工具看图用 (默认关)
        self.plate_image_pub = (
            rospy.Publisher(cfg.topics.plate_crop_image_pub, Image, queue_size=1)
            if cfg.publishing.plate_crop and cfg.publishing.plate_crop_image
            else None
        )

        self.sub = rospy.Subscriber(
            cfg.topics.image_sub,
            Image,
            self.on_image,
            queue_size=cfg.queue_size,
            buff_size=2 ** 24,  # 16MB, 兜住 1080p 单帧
        )
        LOG.info(
            "订阅 %s -> 发布 %s (annotated=%s, plate=%s, plate_image=%s)",
            cfg.topics.image_sub,
            cfg.topics.detections_pub,
            cfg.publishing.annotated_image,
            cfg.publishing.plate_crop,
            cfg.publishing.plate_crop_image,
        )

    # ------------------------------------------------------------- 参数
    @staticmethod
    def _load_params():
        """合并 yaml 配置与 ROS 私有参数 (~conf 之类)。"""
        path = rospy.get_param("~config", "")
        overrides: Dict[str, Any] = {}

        conf = rospy.get_param("~conf", None)
        if conf is not None:
            overrides.setdefault("inference", {})["conf_threshold"] = float(conf)

        iou = rospy.get_param("~iou", None)
        if iou is not None:
            overrides.setdefault("inference", {})["iou_threshold"] = float(iou)

        device = rospy.get_param("~device", "")
        if device:
            overrides.setdefault("model", {})["device"] = device

        weights = rospy.get_param("~weights", "")
        if weights:
            overrides.setdefault("model", {})["weights"] = weights

        imgsz = rospy.get_param("~imgsz", None)
        if imgsz is not None:
            overrides.setdefault("model", {})["imgsz"] = int(imgsz)

        for key, target in (
            ("publish_annotated", ("publishing", "annotated_image")),
            ("publish_plates", ("publishing", "plate_crop")),
        ):
            val = rospy.get_param("~" + key, None)
            if val is not None:
                section, field = target
                overrides.setdefault(section, {})[field] = bool(val)

        cfg = load_config(path or None, overrides or None)
        return cfg, overrides

    def _setup_logging(self, overrides: Dict[str, Any]) -> None:
        level_name = str(rospy.get_param("~log_level", "INFO")).upper()
        level = getattr(logging, level_name, logging.INFO)
        logging.basicConfig(
            level=level,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        )
        logging.getLogger().setLevel(level)

    # ------------------------------------------------------------- 回调
    def on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            # 丢帧即可, 不该让一帧坏图终止节点
            LOG.exception("图像转换失败 (encoding=%s)", msg.encoding)
            return

        with self._lock:
            try:
                result = self.detector.infer(bgr, source_name="")
            except Exception:  # 推理异常不能拖垮整个节点
                LOG.exception("推理异常, 本帧丢弃")
                return

            self._publish(msg, bgr, result)

        self._frame_count += 1
        if self.cfg.log_every_n and self._frame_count % self.cfg.log_every_n == 0:
            LOG.info(
                "已处理 %d 帧 | 本帧 %d 目标 | %.1f ms | %s",
                self._frame_count,
                len(result),
                result.inference_ms,
                result.summary(),
            )

    # ------------------------------------------------------------- 发布
    def _publish(self, msg: Image, bgr, result: InferenceResult) -> None:
        stamp = msg.header.stamp if self.cfg.publishing.stamp_from_input else rospy.Time.now()
        frame_id = self.cfg.frame_id or msg.header.frame_id

        out = DetectionArray()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.inference_ms = result.inference_ms
        out.frame_width = result.frame_width
        out.frame_height = result.frame_height
        out.detections = [_to_msg(d) for d in result.detections]
        self.detections_pub.publish(out)

        if self.annotated_pub is not None:
            canvas = annotate(bgr, result)
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            )

        if self.plate_pub is not None:
            margin = self.cfg.publishing.plate_crop_margin
            for index, det in enumerate(result.plates):
                patch = crop_plate(bgr, det, margin=margin)
                if patch.size == 0:
                    continue

                crop_msg = PlateCrop()
                crop_msg.header.stamp = stamp
                crop_msg.header.frame_id = frame_id
                crop_msg.index = index
                crop_msg.x1 = det.x1
                crop_msg.y1 = det.y1
                crop_msg.x2 = det.x2
                crop_msg.y2 = det.y2
                crop_msg.confidence = det.confidence
                crop_msg.image = self.bridge.cv2_to_imgmsg(patch, encoding="bgr8")
                self.plate_pub.publish(crop_msg)

                if self.plate_image_pub is not None:
                    raw = self.bridge.cv2_to_imgmsg(patch, encoding="bgr8")
                    raw.header.stamp = stamp
                    raw.header.frame_id = frame_id
                    self.plate_image_pub.publish(raw)

    def spin(self) -> None:
        rospy.spin()
        LOG.info("节点退出, 累计处理 %d 帧", self._frame_count)


def _to_msg(det: Detection) -> DetectionMsg:
    """detector.Detection dataclass -> ROS msg。"""
    return DetectionMsg(
        class_id=det.class_id,
        class_name=det.class_name,
        confidence=det.confidence,
        x1=det.x1,
        y1=det.y1,
        x2=det.x2,
        y2=det.y2,
    )


def main() -> int:
    try:
        VisionNode().spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
