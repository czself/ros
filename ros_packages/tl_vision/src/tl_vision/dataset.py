"""数据集体检工具 —— 不含 ROS 依赖。

替代原来的 check_match.py, 补上了它没查的几项:

    python -m tl_vision.dataset check   --root D:/traffic_light_dataset
    python -m tl_vision.dataset stats   --root D:/traffic_light_dataset
    python -m tl_vision.dataset rebalance --root D:/traffic_light_dataset

check   图片/标签一一对应、标签格式、越界坐标、空标签
stats   每类数量 + 长尾占比
rebalance  打印长尾类别与建议采样倍数 (只报告, 不改数据)
"""
from __future__ import annotations

import argparse
import logging
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

LOG = logging.getLogger("tl_vision.dataset")

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
DEFAULT_NAMES: Tuple[str, ...] = (
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


@dataclass
class SplitReport:
    split: str
    image_dir: Path
    label_dir: Path
    n_images: int = 0
    n_labels: int = 0
    images_without_label: List[str] = field(default_factory=list)
    labels_without_image: List[str] = field(default_factory=list)
    empty_labels: List[str] = field(default_factory=list)
    bad_lines: List[str] = field(default_factory=list)
    per_class: Counter = field(default_factory=Counter)

    @property
    def ok(self) -> bool:
        return not (
            self.images_without_label
            or self.labels_without_image
            or self.empty_labels
            or self.bad_lines
        )


def _read_labels(path: Path, report: SplitReport) -> List[Tuple[int, float, float, float, float]]:
    """解析单个标签文件, 返回 (cls, cx, cy, w, h), 问题记进 report。"""
    rows: List[Tuple[int, float, float, float, float]] = []
    try:
        # labels 全是 ASCII, 但仍显式指定 encoding 以免在非 UTF-8 平台炸掉
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        report.bad_lines.append(f"{path.name}: 读取失败 {exc}")
        return rows

    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 5:
            report.bad_lines.append(f"{path.name}:{lineno}: 字段数应为 5, 实际 {len(parts)}")
            continue
        try:
            cls = int(parts[0])
            cx, cy, bw, bh = (float(v) for v in parts[1:])
        except ValueError:
            report.bad_lines.append(f"{path.name}:{lineno}: 含非数字内容 {line!r}")
            continue
        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
            report.bad_lines.append(
                f"{path.name}:{lineno}: 归一化坐标越界 cx={cx} cy={cy} w={bw} h={bh}"
            )
            continue
        rows.append((cls, cx, cy, bw, bh))
        report.per_class[cls] += 1
    if not rows:
        report.empty_labels.append(path.name)
    return rows


def check_split(split: str, images_dir: Path, labels_dir: Path) -> SplitReport:
    rep = SplitReport(split, images_dir, labels_dir)
    if not images_dir.is_dir():
        rep.bad_lines.append(f"图片目录不存在: {images_dir}")
        return rep
    if not labels_dir.is_dir():
        rep.bad_lines.append(f"标签目录不存在: {labels_dir}")
        return rep

    img_stems = {
        p.stem: p
        for p in images_dir.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
    }
    lbl_stems = {p.stem: p for p in labels_dir.iterdir() if p.is_file() and p.suffix == ".txt"}

    rep.n_images = len(img_stems)
    rep.n_labels = len(lbl_stems)
    rep.images_without_label = sorted(set(img_stems) - set(lbl_stems))
    rep.labels_without_image = sorted(set(lbl_stems) - set(img_stems))

    for stem in sorted(set(img_stems) & set(lbl_stems)):
        _read_labels(lbl_stems[stem], rep)
    return rep


def load_names(data_yaml: Optional[Path]) -> Tuple[str, ...]:
    if not data_yaml or not data_yaml.is_file():
        return DEFAULT_NAMES
    raw = yaml.safe_load(data_yaml.read_text(encoding="utf-8")) or {}
    names = raw.get("names")
    if isinstance(names, dict):
        return tuple(str(names[k]) for k in sorted(names))
    if isinstance(names, (list, tuple)) and names:
        return tuple(str(n) for n in names)
    return DEFAULT_NAMES


def cmd_check(args) -> int:
    reports = [
        check_split("train", args.root / "images" / "train", args.root / "labels" / "train"),
        check_split("val", args.root / "images" / "val", args.root / "labels" / "val"),
    ]
    print("=" * 66)
    for rep in reports:
        status = "OK" if rep.ok else "有问题"
        print(f"[{rep.split}] {rep.n_images} 图 / {rep.n_labels} 标签  -> {status}")
        if rep.images_without_label:
            print(f"  图缺标签 ({len(rep.images_without_label)}): {rep.images_without_label[:10]}")
        if rep.labels_without_image:
            print(f"  孤立标签 ({len(rep.labels_without_image)}): {rep.labels_without_image[:10]}")
        if rep.empty_labels:
            print(f"  空标签 ({len(rep.empty_labels)}): {rep.empty_labels[:10]}")
        for line in rep.bad_lines[:20]:
            print(f"  格式错误: {line}")
        if len(rep.bad_lines) > 20:
            print(f"  ... 另有 {len(rep.bad_lines) - 20} 条格式错误")
    print("=" * 66)

    bad = [r for r in reports if not r.ok]
    if bad:
        print("结论: 存在问题, 建议先修干净再训练")
        return 1
    print("结论: 数据集格式检查通过")
    return 0


def cmd_stats(args) -> int:
    names = load_names(args.data)
    total: Counter = Counter()
    per_split: Dict[str, Counter] = {}
    image_counts: Dict[str, int] = {}

    for split in ("train", "val"):
        rep = check_split(split, args.root / "images" / split, args.root / "labels" / split)
        per_split[split] = rep.per_class
        image_counts[split] = rep.n_images
        total.update(rep.per_class)

    def name_of(cid: int) -> str:
        return names[cid] if 0 <= cid < len(names) else f"class_{cid}"

    print(f"{'id':>3}  {'类别':<16} {'train':>7} {'val':>6} {'合计':>7}  {'占比':>7}")
    print("-" * 66)
    for cid in range(len(names)):
        tr = per_split["train"].get(cid, 0)
        va = per_split["val"].get(cid, 0)
        tt = tr + va
        share = tt / sum(total.values()) if total else 0.0
        print(f"{cid:>3}  {name_of(cid):<16} {tr:>7} {va:>6} {tt:>7}  {share:>6.1%}")
    print("-" * 66)
    print(
        f"合计 {sum(total.values())} 个标注实例 | "
        f"{image_counts['train']} 张训练图 / {image_counts['val']} 张验证图"
    )

    # 长尾提示
    if total:
        top = max(total.values())
        tail = [(cid, c) for cid, c in total.items() if c < top * 0.25]
        if tail:
            print("\n长尾类别 (< 最热类别的 25%):")
            for cid, cnt in sorted(tail, key=lambda x: x[1]):
                print(f"  {name_of(cid):<16} {cnt:>5} 个")
    return 0


def cmd_rebalance(args) -> int:
    """只做诊断建议, 不改动任何数据。"""
    from .detector import DEFAULT_CLASS_NAMES as CANON

    names = load_names(args.data)
    if list(names) != list(CANON):
        LOG.warning(
            "data.yaml 的类别顺序与 detector.DEFAULT_CLASS_NAMES 不一致!\n"
            "  data.yaml : %s\n  detector  : %s\n"
            "类别 id 必须逐位对应, 否则推理会张冠李戴。",
            list(names),
            list(CANON),
        )

    rep = check_split("train", args.root / "images" / "train", args.root / "labels" / "train")
    if not rep.per_class:
        print("训练集没有任何标注, 先跑 check 看看格式")
        return 1

    counts = [rep.per_class.get(cid, 0) for cid in range(len(names))]
    top = max(counts)
    print(f"训练集最热类别 {top} 个实例\n")
    print(f"{'类别':<16} {'实例':>6} {'相对热度':>9}  建议")
    print("-" * 58)
    for cid, cnt in enumerate(counts):
        ratio = cnt / top if top else 0.0
        if cnt == 0:
            advice = "完全缺失, 必须补数据"
        elif ratio < 0.25:
            advice = f"过采样 x{min(10, round(top / max(cnt, 1)))} 或补图"
        elif ratio < 0.5:
            advice = "轻度过采样或补图"
        else:
            advice = "尚可"
        label = names[cid] if cid < len(names) else f"class_{cid}"
        print(f"{label:<16} {cnt:>6} {ratio:>8.1%}  {advice}")

    print(
        "\n建议做法 (Ultralytics 侧):\n"
        "  1) 先补实例数最少的那几类 —— 技巧救不了根本没见过样本的类别\n"
        "  2) 长尾类别在 data.yaml 同级目录放一份 <类别名>.txt, 内容是重复的图片\n"
        "     路径, 交给 ultralytics 的 oversampling 机制\n"
        "  3) 扩大 val 集, 当前每类只有 1~11 个验证框, 指标噪声大于信号\n"
        "  4) 采样倍数别超过 10, 再高会过拟合那几张重复图"
    )
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog="tl_vision.dataset", description="tl_vision 数据集体检"
    )
    p.add_argument("--data", default="", help="data.yaml 路径 (可选)")
    p.add_argument("--root", default=".", help="数据集根目录, 含 images/ 与 labels/")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="格式与配对检查")
    sub.add_parser("stats", help="类别分布统计")
    sub.add_parser("rebalance", help="长尾诊断建议")

    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args.root = Path(args.root).expanduser().resolve()
    args.data = Path(args.data).expanduser() if args.data else None

    if args.cmd == "check":
        return cmd_check(args)
    if args.cmd == "stats":
        return cmd_stats(args)
    return cmd_rebalance(args)


if __name__ == "__main__":
    sys.exit(main())
