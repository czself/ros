"""离线批量推理 —— 不需要 ROS。

替代原来的 test_vision.py, 修掉了两个问题:
    1. 不再按文件名硬编码假框, 改为默认关闭的 debug.force_full_image_boxes;
    2. 不再对中文文件名做无编码声明的读写 (原脚本在部分平台会写出乱码文件名)。

    python -m tl_vision.offline --input <图片或目录> --weights best.pt
    python -m tl_vision.offline -i images/val -o out/ --conf 0.35 --save-crops
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from tl_vision.config import load_config
from tl_vision.detector import (
    DEFAULT_CLASS_NAMES,
    DetectorError,
    YoloDetector,
    parse_force_rules,
)
from tl_vision.plate_text import format_for_file
from tl_vision.visualize import annotate, crop_plate

LOG = logging.getLogger("tl_vision.offline")

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="tl_vision_offline",
        description="tl_vision 离线批量推理 (无需 ROS 环境)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("-i", "--input", required=True, help="图片文件或目录")
    p.add_argument("-o", "--output", default="outputs", help="输出目录")
    p.add_argument("-w", "--weights", default="", help="权重路径 (覆盖配置文件)")
    p.add_argument("-c", "--config", default="", help="vision_config.yaml 路径")
    p.add_argument("--conf", type=float, default=None, help="置信度阈值")
    p.add_argument("--iou", type=float, default=None, help="NMS IoU 阈值")
    p.add_argument("--imgsz", type=int, default=None, help="推理分辨率")
    p.add_argument("--device", default=None, help="cuda:0 / cpu, 留空自动")
    p.add_argument("--classes", default="", help="只保留这些类别 id, 逗号分隔, 如 2,3,8")
    p.add_argument("--no-annotate", action="store_true", help="不保存画框图")
    p.add_argument("--save-crops", action="store_true", help="额外保存车牌裁剪图")
    p.add_argument("--plate-margin", type=int, default=None, help="车牌裁剪外扩像素")
    p.add_argument("--json", dest="json_out", default="", help="把结果汇总写入该 json")
    p.add_argument("--fail-on-empty", action="store_true", help="无目标时返回非零退出码")

    ocr_grp = p.add_argument_group("车牌 OCR (需要 pip install paddleocr)")
    ocr_grp.add_argument("--ocr", action="store_true",
                         help="对裁剪出的车牌图直接做 OCR, 结果写进 json")
    ocr_grp.add_argument("--ocr-output", default="",
                         help="OCR 结果文本文件, 与旧版 ocr_result.txt 同格式")
    ocr_grp.add_argument("--ocr-gpu", action="store_true", help="OCR 用 GPU")
    ocr_grp.add_argument("--ocr-engine", default="paddleocr",
                         choices=["paddleocr", "stub"],
                         help="stub = 不调 Paddle, 用于验证流程")
    ocr_grp.add_argument("--ocr-strict", action="store_true",
                         help="校验不合法的车牌, text 置空")
    p.add_argument(
        "--allow-force",
        action="store_true",
        help="允许启用 debug.force_full_image_boxes 调试桩 (结果不可信)",
    )
    return p


def collect_images(path: Path) -> List[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(f"输入路径不存在: {path}")
    imgs = [
        p
        for p in sorted(path.iterdir())
        if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
    ]
    if not imgs:
        raise FileNotFoundError(f"目录下没有支持的图片: {path}")
    return imgs


def build_config_from_args(args) -> object:
    """把命令行参数折进 yaml 覆盖层, 保证只有一份配置语义。"""
    overrides: Dict[str, Dict] = {}
    if args.weights:
        overrides.setdefault("model", {})["weights"] = args.weights
    if args.device is not None:
        overrides.setdefault("model", {})["device"] = args.device
    if args.imgsz is not None:
        overrides.setdefault("model", {})["imgsz"] = args.imgsz
    if args.conf is not None:
        overrides.setdefault("inference", {})["conf_threshold"] = args.conf
    if args.iou is not None:
        overrides.setdefault("inference", {})["iou_threshold"] = args.iou
    if args.classes:
        try:
            ids = [int(x) for x in args.classes.replace(" ", "").split(",") if x]
        except ValueError as exc:
            raise SystemExit(
                f"--classes 必须是逗号分隔的整数, 收到 {args.classes!r}"
            ) from exc
        overrides.setdefault("inference", {})["class_filter"] = ids
    if args.plate_margin is not None:
        overrides.setdefault("publishing", {})["plate_crop_margin"] = args.plate_margin
    if args.allow_force:
        overrides.setdefault("debug", {})["force_full_image_boxes"] = True
    return load_config(args.config or None, overrides or None)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )

    try:
        cfg = build_config_from_args(args)
        images = collect_images(Path(args.input).expanduser())
    except (FileNotFoundError, ValueError) as exc:
        # 参数/路径问题, 打 traceback 只会淹没真正有用的那一行提示
        LOG.error("%s", exc)  # noqa: TRY400
        return 2

    if cfg.debug.force_full_image_boxes and not args.allow_force:
        LOG.error(
            "配置启用了 debug.force_full_image_boxes, 但命令行没给 --allow-force。\n"
            "该开关会让命中规则的图片跳过模型直接返回整图框, 结果不可用于评估。\n"
            "确实要跑请追加 --allow-force。"
        )
        return 2

    try:
        detector = YoloDetector(
            weights=cfg.resolve_weights(),
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
    except (DetectorError, FileNotFoundError) as exc:
        LOG.error("%s", exc)  # noqa: TRY400
        return 1
    except Exception:  # 依赖缺失 / CUDA 不可用等
        LOG.exception("初始化检测器失败")
        return 1

    import cv2

    out_dir = Path(args.output).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    crop_dir = out_dir / "plate_crops"
    if args.save_crops:
        crop_dir.mkdir(parents=True, exist_ok=True)

    LOG.info(
        "共 %d 张图片, conf=%.2f, 输出 -> %s", len(images), cfg.inference.conf_threshold, out_dir
    )

    per_class: Dict[str, int] = {}
    records: List[Dict] = []
    total_ms = 0.0
    empty_count = 0
    forced_count = 0
    plate_ocr = None
    ocr_lines: List[str] = []

    if args.ocr:
        try:
            from .ocr import PlateOcr

            plate_ocr = PlateOcr(
                engine=args.ocr_engine,
                use_gpu=args.ocr_gpu,
                strict=args.ocr_strict,
            )
            LOG.info("车牌 OCR 已启用 (引擎=%s, API=%s)", args.ocr_engine, plate_ocr.api_version)
        except Exception as exc:
            LOG.error("初始化 OCR 失败: %s", exc)  # noqa: TRY400
            return 1

    for idx, img_path in enumerate(images, 1):
        image = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if image is None:
            LOG.warning("读取失败, 跳过: %s", img_path.name)
            continue

        try:
            result = detector.infer(image, source_name=img_path.name)
        except Exception:
            LOG.exception("推理失败 %s", img_path.name)
            continue

        total_ms += result.inference_ms
        if result.forced:
            forced_count += 1
        if not result.detections:
            empty_count += 1

        for det in result.detections:
            per_class[det.class_name] = per_class.get(det.class_name, 0) + 1

        if not args.no_annotate:
            canvas = annotate(image, result)
            cv2.imwrite(str(out_dir / img_path.name), canvas)

        plate_texts: List[Dict] = []
        if args.save_crops or plate_ocr is not None:
            for det in result.plates:
                patch = crop_plate(image, det, cfg.publishing.plate_crop_margin)
                if not patch.size:
                    continue

                stem = f"{img_path.stem}_{int(det.x1)}_{int(det.y1)}_{int(det.x2)}_{int(det.y2)}"
                # imwrite 只认 ASCII 文件名, 中文名先转安全名避免写出乱码
                safe = stem.encode("ascii", "backslashreplace").decode("ascii")
                crop_name = f"{safe}.jpg"

                if args.save_crops:
                    cv2.imwrite(str(crop_dir / crop_name), patch)

                if plate_ocr is not None:
                    try:
                        plate = plate_ocr.read(patch)
                        plate_texts.append(
                            {
                                "crop": crop_name,
                                "text": plate.text,
                                "display": plate.display,
                                "valid": plate.is_valid,
                                "issue": plate.issue,
                                "confidence": round(plate.confidence, 4),
                            }
                        )
                        ocr_lines.append(
                            f"{crop_name}\t{format_for_file(plate.display or plate.text or plate.raw)}"
                        )
                    except Exception as exc:  # 单个车牌识别失败不影响整批
                        LOG.error("OCR 失败 %s: %s", crop_name, exc)  # noqa: TRY400
                        plate_texts.append(
                            {"crop": crop_name, "text": "", "display": "",
                             "valid": False, "issue": f"识别失败: {exc}", "confidence": 0.0}
                        )
                        ocr_lines.append(f"{crop_name}\t")

        record: Dict = {
            "image": img_path.name,
            "inference_ms": round(result.inference_ms, 2),
            "forced": result.forced,
            "detections": [d.to_dict() for d in result.detections],
        }
        if plate_ocr is not None:
            record["plates"] = plate_texts
        records.append(record)

        plate_note = ""
        if plate_ocr is not None and plate_texts:
            shown = ", ".join(
                p["display"] or "-" for p in plate_texts
            )
            bad = [p for p in plate_texts if not p["valid"]]
            plate_note = f" | 车牌: {shown}"
            if bad:
                plate_note += f" ({len(bad)} 个不合规)"
        LOG.info(
            "[%d/%d] %s -> %d 目标 (%.1f ms) %s%s",
            idx,
            len(images),
            img_path.name,
            len(result),
            result.inference_ms,
            result.summary(),
            plate_note,
        )

    done = len(records)
    failed = len(images) - done
    LOG.info("=" * 60)
    LOG.info("处理 %d 张, 平均 %.1f ms/张", done, total_ms / done if done else 0.0)
    LOG.info("无目标图片: %d 张", empty_count)
    if forced_count:
        LOG.warning("有 %d 张图片走了 force 调试桩, 其结果不是模型输出", forced_count)
    if per_class:
        LOG.info("各类别检出数: %s", per_class)
    else:
        LOG.info("全程没有检出任何目标 —— 试试调低 --conf")

    if plate_ocr is not None:
        all_plates = [p for r in records for p in r.get("plates", [])]
        bad = [p for p in all_plates if not p["valid"]]
        LOG.info(
            "车牌 OCR: %d 个 (合规 %d / 不合规 %d)",
            len(all_plates), len(all_plates) - len(bad), len(bad),
        )
        for p in bad:
            LOG.warning("  不合规车牌 %s -> %s", p["crop"], p["issue"])

    if args.ocr_output:
        ocr_path = Path(args.ocr_output).expanduser()
        ocr_path.parent.mkdir(parents=True, exist_ok=True)
        ocr_path.write_text("\n".join(ocr_lines) + "\n", encoding="utf-8")
        LOG.info("OCR 文本结果 -> %s", ocr_path)

    if args.json_out:
        json_path = Path(args.json_out).expanduser()
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open("w", encoding="utf-8") as fh:
            json.dump(
                {
                    "config": {
                        "conf": cfg.inference.conf_threshold,
                        "iou": cfg.inference.iou_threshold,
                        "imgsz": cfg.model.imgsz,
                        "class_filter": cfg.inference.class_filter,
                        "force_full_image": cfg.debug.force_full_image_boxes,
                        "ocr": bool(plate_ocr),
                    },
                    "class_names": list(DEFAULT_CLASS_NAMES),
                    "summary": {
                        "images": done,
                        "empty": empty_count,
                        "forced": forced_count,
                        "avg_ms": round(total_ms / done, 2) if done else 0.0,
                        "per_class": per_class,
                        "plates_ocr": sum(
                            len(r.get("plates", [])) for r in records
                        ),
                    },
                    "results": records,
                },
                fh,
                ensure_ascii=False,
                indent=2,
            )
        LOG.info("json 汇总 -> %s", json_path)

    if done == 0:
        LOG.error(
            "%d 张图片全部处理失败, 无任何有效结果 —— 退出码 1", len(images)
        )
        return 1
    if failed:
        LOG.warning("有 %d 张图片读取或推理失败, 已跳过", failed)

    if args.fail_on_empty and empty_count:
        LOG.error("有 %d 张图片无目标, 按 --fail-on-empty 返回非零", empty_count)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
