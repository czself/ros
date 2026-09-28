"""检测结果绘制 —— 不含 ROS 依赖。

cv2.putText 的 Hershey 字体不支持中文, 因此图上标注一律用 ASCII 类别名
(red_on / license_plate ...)。中文说明请看终端日志或自定义 msg 里的
class_name 语义。
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from .detector import Detection, InferenceResult

BGR = Tuple[int, int, int]

#: 按语义分色, 便于人眼快速区分 (BGR)
DEFAULT_COLORS: Dict[str, BGR] = {
    # 行人 —— 蓝
    "resident": (255, 128, 0),
    "stranger": (0, 128, 255),
    # 红绿灯 —— 各自灯色
    "red_on": (0, 0, 255),
    "red_off": (128, 128, 128),
    "yellow_on": (0, 220, 255),
    "yellow_off": (160, 160, 160),
    "green_on": (0, 220, 0),
    "green_off": (128, 128, 128),
    # 车牌 —— 绿
    "license_plate": (0, 255, 128),
}
_FALLBACK_COLOR: BGR = (255, 255, 255)


def color_for(class_name: str, colors: Optional[Dict[str, BGR]] = None) -> BGR:
    table = colors or DEFAULT_COLORS
    return table.get(class_name, _FALLBACK_COLOR)


def draw_detections(
    image_bgr: np.ndarray,
    detections: Iterable[Detection],
    colors: Optional[Dict[str, BGR]] = None,
    thickness: int = 2,
    font_scale: float = 0.5,
    show_conf: bool = True,
    draw_labels: bool = True,
) -> np.ndarray:
    """在图上画出检测框。**就地修改并返回** image_bgr (不复制)。

    画布太小时自动收窄线宽和字号, 避免标签糊成一团。
    """
    import cv2  # 局部导入: 让 --help / 单测不必装 opencv

    h, w = image_bgr.shape[:2]
    if min(h, w) < 480:
        thickness = max(1, thickness - 1)
        font_scale = max(0.35, font_scale - 0.1)

    for det in detections:
        color = color_for(det.class_name, colors)
        p1 = (int(det.x1), int(det.y1))
        p2 = (int(det.x2), int(det.y2))
        cv2.rectangle(image_bgr, p1, p2, color, thickness)

        if not draw_labels:
            continue
        label = det.class_name
        if show_conf:
            label = f"{label} {det.confidence:.2f}"

        (tw, th), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
        )
        # 贴边时把标签翻到框内侧, 否则会被裁掉
        ty = p1[1] - 4 if p1[1] - th - baseline - 4 >= 0 else p1[1] + th + 6
        tx = p1[0]
        if tx + tw + 4 > w:
            tx = max(0, w - tw - 6)

        cv2.rectangle(
            image_bgr,
            (tx, ty - th - baseline - 2),
            (tx + tw + 4, ty + 2),
            color,
            -1,  # 实心
        )
        # 实心底色上画黑字保证可读
        cv2.putText(
            image_bgr,
            label,
            (tx + 2, ty - baseline - 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            max(1, thickness - 1),
            cv2.LINE_AA,
        )

    return image_bgr


def crop_plate(
    image_bgr: np.ndarray,
    det: Detection,
    margin: int = 6,
) -> np.ndarray:
    """按检测框裁出车牌, 向外扩 margin 像素并做边界裁剪。

    扩边是为了给 OCR 留出字符边缘 —— 收得太紧会切掉车牌首尾字符。
    """
    h, w = image_bgr.shape[:2]
    x1 = max(0, int(det.x1) - margin)
    y1 = max(0, int(det.y1) - margin)
    x2 = min(w, int(det.x2) + margin)
    y2 = min(h, int(det.y2) + margin)
    if x2 <= x1 or y2 <= y1:
        return np.empty((0, 0, 3), dtype=np.uint8)
    return image_bgr[y1:y2, x1:x2].copy()


def annotate(
    image_bgr: np.ndarray,
    result: InferenceResult,
    colors: Optional[Dict[str, BGR]] = None,
    copy: bool = True,
    show_conf: bool = True,
) -> np.ndarray:
    """画框 + 左上角写一行汇总统计。"""
    import cv2

    canvas = image_bgr.copy() if copy else image_bgr
    draw_detections(canvas, result.detections, colors=colors, show_conf=show_conf)

    _h, w = canvas.shape[:2]
    summary = f"{len(result)} obj | {result.inference_ms:.0f} ms"
    if result.forced:
        summary += " | FORCED"
    (tw, th), baseline = cv2.getTextSize(
        summary, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
    )
    cv2.rectangle(canvas, (0, 0), (min(w, tw + 10), th + baseline + 6), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        summary,
        (5, th + 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


#: 识别结果标注的边框色 (BGR) —— 绿=合规, 红=不合规
PLATE_OK_COLOR: BGR = (0, 220, 0)
PLATE_BAD_COLOR: BGR = (0, 0, 255)


def annotate_plate_text(
    image_bgr: np.ndarray,
    valid: bool,
    confidence: float = 0.0,
    index: Optional[int] = None,
    copy: bool = True,
) -> np.ndarray:
    """把 OCR 结果画回车牌裁剪图: 边框色表示是否合规 + ASCII 置信度。

    **故意不把车牌号画到图上** —— cv2.putText 的 Hershey 字体没有中文字形,
    "苏A·B8Q62" 会画成一串 "???"。想确认识别成什么, 看 PlateText 消息或
    终端日志, 那里的文本是准确的。

    Args:
        valid:  是否通过号牌规则校验, 决定边框颜色
        confidence: OCR 置信度, 图上只画这个 ASCII 数字
        index:  该帧内第几个车牌, 会一起画上去
    """
    import cv2

    if image_bgr is None or image_bgr.size == 0:
        return image_bgr

    canvas = image_bgr.copy() if copy else image_bgr
    color = PLATE_OK_COLOR if valid else PLATE_BAD_COLOR
    h, w = canvas.shape[:2]

    # 裁剪图可能很扁 (车牌比例约 2:1), 线宽给 2 就够粗了
    thickness = 3 if min(h, w) >= 40 else 2
    cv2.rectangle(canvas, (0, 0), (w - 1, h - 1), color, thickness)

    label = f"{confidence:.2f}"
    if index is not None:
        label = f"#{index} {label}"

    (tw, th), baseline = cv2.getTextSize(
        label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1
    )
    # 标签贴在框内侧 —— 往上贴会被自己的边框盖住, 往下贴可能超出图外
    ty = th + baseline + 2
    if ty >= h:
        # 图太矮, 标签压在图上
        ty = h - 2
    cv2.rectangle(canvas, (1, 1), (tw + 6, ty + 2), color, -1)
    cv2.putText(
        canvas,
        label,
        (4, ty - 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return canvas
