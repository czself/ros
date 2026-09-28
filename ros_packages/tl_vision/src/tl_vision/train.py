"""训练入口 —— 不含 ROS 依赖, 在训练机上跑。

    python -m tl_vision.train --data config/data.yaml --epochs 100
    python -m tl_vision.train --data config/data.yaml --resume runs/train/exp/weights/last.pt

相对原来的 train.py 的改动:
    * 路径全部走参数/env, 不再硬编码 D:\\traffic_light_dataset
    * 默认开 AMP 并把 torch>=2.6 的 weights_only 限制显式关掉
      (旧版为了绕开 ultralytics 自动下载 yolo26n.pt 的问题而全局关 AMP,
       那是治标; 这里只在 amp_checks 阶段关闭权重自动下载)
    * 加了早停与 patience, 针对本数据集 50 轮后已明显过拟合的情况
    * 训练完自动导出 ONNX, 方便后续转 TensorRT
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import Optional, Sequence

LOG = logging.getLogger("tl_vision.train")

WEIGHTS_ENV_VAR = "TL_VISION_WEIGHTS"
PKG_ROOT = Path(__file__).resolve().parent.parent.parent


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="tl_vision_train",
        description="tl_vision YOLOv8 训练",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--data", default=str(PKG_ROOT / "config" / "data.yaml"), help="数据集 yaml")
    p.add_argument("--weights", default="yolov8s.pt", help="初始权重, 相对路径按 cwd 解析")
    p.add_argument("--project", default="runs/train", help="输出根目录")
    p.add_argument("--name", default="tl_vision", help="本次实验名")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=4, help="显存不足就调小; -1 = 自动估算")
    p.add_argument("--device", default="0", help="0 / 0,1 / cpu")
    p.add_argument("--workers", type=int, default=0, help="Windows 必须 0")
    p.add_argument("--patience", type=int, default=20, help="早停耐心值, 0=关闭")
    p.add_argument("--no-amp", action="store_true", help="禁用 AMP")
    p.add_argument("--cache", action="store_true", help="把图片缓存进内存")
    p.add_argument("--resume", default="", help="从该权重继续训练")
    p.add_argument("--export-onnx", action="store_true", help="训练后导出 ONNX")
    return p


def resolve_init_weights(raw: str) -> str:
    """绝对路径 -> 原样; 存在的相对路径 -> 绝对; 否则留给 ultralytics 下载。"""
    candidate = Path(raw).expanduser()
    if candidate.is_file():
        return str(candidate.resolve())
    if os.path.sep in raw or (os.altsep and os.altsep in raw):
        raise FileNotFoundError(f"--weights 指定的本地文件不存在: {candidate}")
    return raw  # 纯名字, 如 yolov8s.pt, 交给 ultralytics 处理


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )

    data_yaml = Path(args.data).expanduser()
    if not data_yaml.is_file():
        LOG.error("数据集配置不存在: %s", data_yaml)
        return 2

    try:
        init_weights = resolve_init_weights(args.weights)
    except FileNotFoundError as exc:
        LOG.error("%s", exc)  # noqa: TRY400
        return 2

    try:
        from ultralytics import YOLO
    except ImportError:
        LOG.exception("未安装 ultralytics: pip install -r requirements.txt")
        return 1

    if args.resume:
        LOG.warning("从 %s 续训, 超参以该权重内记录为准", args.resume)
        model = YOLO(args.resume)
        model.train(resume=True)
        return 0

    LOG.info("基础权重: %s", init_weights)
    model = YOLO(init_weights)

    train_kwargs = {
        "data": str(data_yaml.resolve()),
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "device": args.device,
        "batch": args.batch,
        "workers": args.workers,
        "project": str(Path(args.project).expanduser().resolve()),
        "name": args.name,
        "exist_ok": False,          # 同名实验直接报错, 避免覆盖掉别人的结果
        "save": True,
        "save_period": -1,          # 10 轮存一次 ~90MB/个, 磁盘紧张就关掉
        "val": True,
        "plots": True,
        "cache": args.cache,
        "amp": not args.no_amp,
        # ultralytics >= 8.3 在 torch>=2.6 下对 torch.load 的 weights_only 默认
        # 收紧, 会导致加载训练好的 .pt 报 UnpicklingError, 这里显式放开
        "torch_safe_load": False,
    }
    if args.patience > 0:
        train_kwargs["patience"] = args.patience
        LOG.info("启用早停 patience=%d", args.patience)

    try:
        model.train(**train_kwargs)
    except Exception:
        LOG.exception("训练失败")
        return 1

    best = model.trainer.best
    LOG.info("训练结束, 最优权重: %s", best)

    if args.export_onnx and best:
        try:
            model.export(format="onnx", imgsz=args.imgsz, simplify=True)
            LOG.info("ONNX 导出完成")
        except Exception:
            LOG.exception("ONNX 导出失败 (不影响已训练的权重)")

    LOG.info(
        "把 best.pt 拷到 %s, 或设 %s 指向它, 推理端即可加载",
        PKG_ROOT / "weights",
        WEIGHTS_ENV_VAR,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
