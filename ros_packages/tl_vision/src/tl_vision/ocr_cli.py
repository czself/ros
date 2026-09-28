"""车牌 OCR 命令行工具 —— 不需要 ROS。

替代原来的 ocr_batch.py, 保留了它的核心行为 (批量识别裁剪图 -> 结果文件),
同时修掉了几个问题:

    1. 不再硬编码 D:\\traffic_light_dataset 与固定结果文件名
    2. 输出显式指定 encoding, 避免在非 UTF-8 平台写出乱码
    3. 识别结果会做规范化与 GA 36 校验, 非法结果单独标出来而不是混在一起
    4. 单张失败不会中断整批 (原版会直接崩掉)

    # 识别裁剪图目录 (接在离线推理 --save-crops 后面用)
    python3 -m tl_vision.ocr -i outputs/val_run1/plate_crops -o outputs/ocr.txt

    # 识别单张图
    python3 -m tl_vision.ocr -i plate.jpg

    # 只看结果不上报告, 用于 CI
    python3 -m tl_vision.ocr -i crops/ --strict --fail-on-invalid
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

from .ocr import IMAGE_SUFFIXES, OcrError, OcrStats, PlateOcr
from .plate_text import format_for_file

LOG = logging.getLogger("tl_vision.ocr.cli")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="tl_vision_ocr",
        description="tl_vision 车牌 OCR 批量识别 (无需 ROS)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("-i", "--input", required=True, help="图片文件或目录")
    p.add_argument("-o", "--output", default="", help="结果文本文件, 省略则只打印")
    p.add_argument("--json", dest="json_out", default="", help="结构化结果 json")
    p.add_argument("--engine", default="paddleocr", choices=["paddleocr", "stub"],
                   help="stub = 不调 Paddle, 恒返回空 (用于验证流程)")
    p.add_argument("--lang", default="ch", help="PaddleOCR 语言, 如 ch / en")
    p.add_argument("--gpu", action="store_true", help="用 GPU 跑 OCR (默认 CPU)")
    p.add_argument("--no-angle-cls", action="store_true", help="关闭方向分类")
    p.add_argument("--mkldnn", action="store_true",
                   help="启用 oneDNN 加速 (3.x 默认关闭: paddlepaddle 3.3.x 上会崩)")
    p.add_argument("--strict", action="store_true",
                   help="校验不通过时把 text 清空 (只保留合规结果)")
    p.add_argument("--tsv", action="store_true",
                   help="结果文件用制表符分隔 (与旧版 ocr_result.txt 同格式)")
    p.add_argument("--fail-on-invalid", action="store_true",
                   help="存在非法车牌时返回非零退出码")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )

    src = Path(args.input).expanduser()
    if not src.exists():
        LOG.error("输入路径不存在: %s", src)
        return 2

    try:
        ocr = PlateOcr(
            engine=args.engine,
            lang=args.lang,
            use_gpu=args.gpu,
            use_angle_cls=not args.no_angle_cls,
            strict=args.strict,
            enable_mkldnn=args.mkldnn,
        )
    except OcrError as exc:
        LOG.error("%s", exc)  # noqa: TRY400
        return 1

    # ------------------------------------------------------------- 收集待识别文件
    if src.is_file():
        targets = [src]
    else:
        targets = [
            p
            for p in sorted(src.iterdir())
            if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
        ]
        if not targets:
            LOG.error("目录下没有支持的图片: %s", src)
            return 2

    LOG.info("待识别 %d 张, 引擎=%s%s", len(targets), args.engine,
             " (GPU)" if args.gpu else " (CPU)")

    # ------------------------------------------------------------------ 跑
    stats = OcrStats()
    lines = []
    records = []

    for path in targets:
        stats.total += 1
        t0 = time.perf_counter()
        try:
            plate = ocr.read_file(path)
            stats.succeeded += 1
        except Exception as exc:
            LOG.error("识别失败 %s: %s", path.name, exc)  # noqa: TRY400
            plate = None
            stats.failed += 1
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if plate is not None:
            stats.total_ms += elapsed_ms

        if plate is None:
            lines.append(f"{path.name}\t")
            records.append({
                "image": path.name, "text": "", "display": "",
                "valid": False, "issue": "识别失败", "raw": "",
                "confidence": 0.0, "ms": round(elapsed_ms, 2),
            })
            continue

        if plate.is_valid:
            stats.valid += 1
        else:
            stats.invalid += 1

        # 与旧版一致: "文件名\t识别结果"; 保留原始输出便于排查
        result_txt = plate.display or plate.text or plate.raw
        lines.append(f"{path.name}\t{format_for_file(result_txt)}")

        records.append({
            "image": path.name,
            "text": plate.text,
            "display": plate.display,
            "valid": plate.is_valid,
            "issue": plate.issue,
            "raw": plate.raw,
            "confidence": round(plate.confidence, 4),
            "ms": round(elapsed_ms, 2),
        })
        flag = "" if plate.is_valid else f"  <- {plate.issue}"
        LOG.info("%-52s %-14s%s", path.name, result_txt or "-", flag)

    # ------------------------------------------------------------------ 输出
    LOG.info("=" * 66)
    LOG.info("统计: %s", stats.as_dict())

    if args.output:
        out = Path(args.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        LOG.info("文本结果 -> %s", out)

    if args.json_out:
        jp = Path(args.json_out).expanduser()
        jp.parent.mkdir(parents=True, exist_ok=True)
        jp.write_text(
            json.dumps(
                {
                    "config": {
                        "engine": args.engine, "lang": args.lang,
                        "gpu": args.gpu, "strict": args.strict,
                    },
                    "summary": stats.as_dict(),
                    "results": records,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        LOG.info("json 结果 -> %s", jp)

    if stats.succeeded == 0:
        LOG.error("全部识别失败, 退出码 1")
        return 1
    if args.fail_on_invalid and stats.invalid:
        LOG.error("有 %d 个非法车牌, 按 --fail-on-invalid 返回非零", stats.invalid)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
