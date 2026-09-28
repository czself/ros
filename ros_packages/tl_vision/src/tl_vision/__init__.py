"""tl_vision —— 红绿灯 / 车牌 / 行人一体化视觉检测 (YOLOv8 + ROS1)。

快速上手:
    实时:  rosrun tl_vision tl_vision_node
    离线:  python3 -m tl_vision.offline -i images/val -o out/
    体检:  python3 -m tl_vision.dataset --root /data/tl check
"""
from .detector import (
    DEFAULT_CLASS_NAMES,
    Detection,
    DetectorError,
    InferenceResult,
    YoloDetector,
)

__version__ = "1.0.0"

__all__ = [
    "DEFAULT_CLASS_NAMES",
    "Detection",
    "DetectorError",
    "InferenceResult",
    "YoloDetector",
    "__version__",
]
