"""tl_vision 车牌 OCR ROS1 节点。

职责: 订阅 tl_vision/PlateCrop -> PaddleOCR 识别 -> 发布 tl_vision/PlateText。

    rosrun tl_vision tl_vision_ocr_node
    rosrun tl_vision tl_vision_ocr_node _engine:=stub      # 不装 paddleocr 也能联调
    rosrun tl_vision tl_vision_ocr_node _strict:=true      # 非法结果不给出 text

为什么要独立成一个节点, 而不是并进检测节点:
    1. 依赖装不到一起。检测要 torch/ultralytics, OCR 要 paddleocr/paddlepaddle,
       这两套的 numpy(1.24 vs 2.2)、opencv(4.6 vs 5.0)、Python(3.8 vs 3.10)
       互相冲突, 装不进同一个 conda 环境。拆成两个节点后各跑各的环境。
    2. 速度差两个数量级。检测 13~25 ms/帧, OCR 单张 370 ms(2.x CPU) ~ 4 s(3.x)。
       绑一起相机会掉到 0.25~2.7 FPS。
    3. OCR 是可选的。车上没装 paddleocr 时, 检测节点照跑不受影响。

因此本文件**只 import 不依赖 torch 的模块** —— 导入 tl_vision.config 是安全的
(它内部的 ultralytics 是延迟导入), 但绝不能碰会拉起 torch 的路径。

背压: OCR 远慢于相机帧率, 队列深度固定取 ocr.queue_size (默认 1)。
处理不过来时 rospy 直接丢旧帧, 宁可丢也不能让延迟无界增长 —— 车牌文本
几百毫秒后就已经过期了。
"""
from __future__ import annotations

import dataclasses
import logging
import sys
import time
from typing import Optional

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tl_vision.config import OcrCfg, load_config

# 生成的 ROS 消息 (由 catkin 的 message_generation 产出)
from tl_vision.msg import PlateCrop
from tl_vision.msg import PlateText as PlateTextMsg
from tl_vision.ocr import OcrError, PlateOcr
from tl_vision.plate_text import PlateText
from tl_vision.visualize import annotate_plate_text

LOG = logging.getLogger("tl_vision.ocr_node")


class OcrNode:
    def __init__(self) -> None:
        cfg, overrides = self._load_params()
        self.cfg: OcrCfg = cfg.ocr
        self.bridge = CvBridge()
        self._setup_logging(overrides)

        self._count = 0
        self._ok = 0
        self._invalid = 0
        self._failed = 0
        self._dropped = 0
        self._total_ms = 0.0

        LOG.info("正在加载 OCR 引擎 (engine=%s) ...", self.cfg.engine)
        try:
            self.ocr = PlateOcr(
                engine=self.cfg.engine,
                lang=self.cfg.lang,
                use_gpu=self.cfg.use_gpu,
                use_angle_cls=self.cfg.use_angle_cls,
                strict=self.cfg.strict,
                min_conf=self.cfg.min_conf,
                enable_mkldnn=self.cfg.enable_mkldnn,
            )
        except OcrError as exc:
            LOG.fatal("OCR 引擎初始化失败: %s", exc)
            raise SystemExit(1) from exc
        except Exception as exc:  # 依赖缺失 / 模型下载失败等
            LOG.fatal("加载 OCR 引擎时发生未预期错误: %s", exc)
            raise SystemExit(1) from exc

        self.text_pub = rospy.Publisher(
            self.cfg.topics.plate_text_pub, PlateTextMsg, queue_size=1
        )
        self.annotated_pub = rospy.Publisher(
            self.cfg.topics.annotated_pub, Image, queue_size=1
        )
        self.sub = rospy.Subscriber(
            self.cfg.topics.plate_sub,
            PlateCrop,
            self.on_plate,
            queue_size=self.cfg.queue_size,
            buff_size=2 ** 20,  # 1MB, 车牌裁剪图很小
        )
        LOG.info(
            "订阅 %s -> 发布 %s (引擎=%s API=%s strict=%s drop_invalid=%s)",
            self.cfg.topics.plate_sub,
            self.cfg.topics.plate_text_pub,
            self.cfg.engine,
            self.ocr.api_version,
            self.cfg.strict,
            self.cfg.drop_invalid,
        )
        if self.ocr.api_version == "stub":
            LOG.warning("当前是 stub 引擎, 只会返回空结果, 不会真正识别 —— 仅供联调")

    # ------------------------------------------------------------- 参数
    @staticmethod
    def _load_params():
        """合并 yaml 与 ROS 私有参数。"""
        path = rospy.get_param("~config", "")
        overrides = {}

        def put(section: str, field: str, key: str, cast=None) -> None:
            val = rospy.get_param("~" + key, None)
            if val is None:
                return
            if cast is not None:
                val = cast(val)
            overrides.setdefault(section, {})[field] = val

        put("ocr", "engine", "engine", str)
        put("ocr", "lang", "lang", str)
        put("ocr", "use_gpu", "gpu", bool)
        put("ocr", "use_angle_cls", "angle_cls", bool)
        put("ocr", "strict", "strict", bool)
        put("ocr", "min_conf", "min_conf", float)
        put("ocr", "enable_mkldnn", "mkldnn", bool)
        put("ocr", "drop_invalid", "drop_invalid", bool)
        put("ocr", "log_every_n", "log_every_n", int)

        topics = overrides.setdefault("ocr", {}).setdefault("topics", {})
        for field, key in (
            ("plate_sub", "plate_topic"),
            ("plate_text_pub", "text_topic"),
            ("annotated_pub", "annotated_topic"),
        ):
            val = rospy.get_param("~" + key, "")
            if val:
                topics[field] = val

        cfg = load_config(path or None, overrides or None)
        return cfg, overrides

    @staticmethod
    def _setup_logging(overrides) -> None:
        level_name = str(rospy.get_param("~log_level", "INFO")).upper()
        level = getattr(logging, level_name, logging.INFO)
        logging.basicConfig(
            level=level,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        )
        logging.getLogger().setLevel(level)

    # ------------------------------------------------------------- 回调
    def on_plate(self, msg: PlateCrop) -> None:
        self._count += 1

        image = self._to_bgr(msg)
        if image is None:
            self._failed += 1
            self._maybe_log()
            return

        t0 = time.perf_counter()
        try:
            plate = self.ocr.read(image)
        except Exception as exc:  # 单块车牌失败不能拖垮整个节点
            self._failed += 1
            LOG.error(  # noqa: TRY400
                "识别失败 (stamp=%s index=%d): %s", msg.header.stamp, msg.index, exc
            )
            self._maybe_log()
            return
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        self._total_ms += elapsed_ms

        plate = self._apply_min_conf(plate)
        if plate.is_valid:
            self._ok += 1
        else:
            self._invalid += 1

        self._publish(msg, plate, image, elapsed_ms)
        self._maybe_log(plate=plate, elapsed_ms=elapsed_ms)

    def _to_bgr(self, msg: PlateCrop):
        if not msg.image.data:
            LOG.warning("收到空的裁剪图 (stamp=%s index=%d)", msg.header.stamp, msg.index)
            return None
        try:
            return self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
        except CvBridgeError:
            LOG.exception("裁剪图转换失败 (encoding=%s)", msg.image.encoding)
            return None

    def _apply_min_conf(self, plate: PlateText) -> PlateText:
        """置信度低于阈值的也标为不合规, 但保留已识别出的文本。"""
        if self.cfg.min_conf <= 0.0 or not plate.is_valid:
            return plate
        if plate.confidence >= self.cfg.min_conf:
            return plate
        return dataclasses.replace(
            plate,
            issue=(
                f"置信度 {plate.confidence:.3f} "
                f"低于阈值 {self.cfg.min_conf:.3f}"
            ),
        )

    # ------------------------------------------------------------- 发布
    def _publish(
        self, crop: PlateCrop, plate: PlateText, image, elapsed_ms: float
    ) -> None:
        if plate.is_valid:
            LOG.info(
                "识别 %s (置信度 %.2f, %.0f ms)",
                plate.display or "-",
                plate.confidence,
                elapsed_ms,
            )
        else:
            LOG.warning(
                "不合规 raw=%r -> %s",
                plate.raw,
                plate.issue or "未知原因",
            )

        if self.cfg.drop_invalid and not plate.is_valid:
            self._dropped += 1
            return

        out = PlateTextMsg()
        out.header = crop.header  # 原样回传, (stamp, index) 即可定位来源
        out.index = crop.index
        out.x1 = crop.x1
        out.y1 = crop.y1
        out.x2 = crop.x2
        out.y2 = crop.y2
        out.raw = plate.raw
        out.text = plate.text
        out.display = plate.display
        out.confidence = plate.confidence
        out.elapsed_ms = elapsed_ms
        out.valid = plate.is_valid
        out.issue = plate.issue or ""
        self.text_pub.publish(out)

        # 图上只画边框色和置信度 —— cv2 的 Hershey 字体没有中文字形,
        # 车牌号画上去会变成一串 "???"。准确文本看消息或日志。
        canvas = annotate_plate_text(
            image, plate.is_valid, plate.confidence, index=crop.index
        )
        img_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        img_msg.header = crop.header
        self.annotated_pub.publish(img_msg)

    # ------------------------------------------------------------- 日志
    def _maybe_log(self, plate: Optional[PlateText] = None, elapsed_ms: float = 0.0):
        every = self.cfg.log_every_n
        if not every or self._count % every != 0:
            return
        avg = self._total_ms / self._ok if self._ok else 0.0
        LOG.info(
            "累计 %d 块 | 合规 %d | 不合规 %d | 失败 %d | 丢弃 %d | 识别均耗时 %.0f ms",
            self._count, self._ok, self._invalid, self._failed, self._dropped, avg,
        )
        if plate is not None and elapsed_ms > 2000.0:
            LOG.warning("单块耗时 %.0f ms, 明显偏慢, 建议换 GPU 或检查是否被 drop_invalid 之外的逻辑拖累", elapsed_ms)

    def spin(self) -> None:
        rospy.spin()
        LOG.info(
            "节点退出, 累计 %d 块 (合规 %d / 不合规 %d / 失败 %d / 丢弃 %d)",
            self._count, self._ok, self._invalid, self._failed, self._dropped,
        )


def main() -> int:
    try:
        OcrNode().spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
