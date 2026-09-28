# 数据集体检报告

> 数据来源：`D:\traffic_light_dataset`　生成时间：2026-09-27
> 本文档记录体检结论，供后续补数据 / 复训时参考。

## 1. 规模

| 项 | 数量 |
|---|---|
| 训练图 | 285 张（全部 480×640） |
| 验证图 | 18 张 |
| 标注实例 | 878 个 |
| 图片/标签配对 | 285/285、18/18，**完全一致，无缺失无孤立** |
| 标签格式 | 全部合法，字段数与归一化坐标均无越界 |

## 2. 类别分布

| id | 类别 | train | val | 合计 | 占比 |
|---:|---|---:|---:|---:|---:|
| 0 | `resident` | 380 | 1 | 381 | 43.4% |
| 1 | `stranger` | 34 | 1 | 35 | 4.0% |
| 2 | `red_on` | 43 | 5 | 48 | 5.5% |
| 3 | `red_off` | 86 | 6 | 92 | 10.5% |
| 4 | `yellow_on` | 21 | 1 | 22 | 2.5% |
| 5 | `yellow_off` | 109 | 11 | 120 | 13.7% |
| 6 | `green_on` | 65 | 5 | 70 | 8.0% |
| 7 | `green_off` | 66 | 5 | 71 | 8.1% |
| 8 | `license_plate` | 35 | 4 | 39 | 4.4% |

最热类 `resident` 是最冷类 `yellow_on` 的 **18 倍**。

### 验证集的问题

18 张图 / 每类 1~11 个框。`resident` 和 `stranger` 在验证集里**各只有 1 个框**，
这两个类的验证指标没有任何统计意义。当前报告的 mAP50 基本由红绿灯类支撑。

## 3. 过拟合

`best.pt`（epoch 53）与末轮（epoch 100）对比：

| | best (ep53) | last (ep100) |
|---|---:|---:|
| mAP50 | 0.834 | 0.823 |
| mAP50-95 | 0.629 | 0.639 |
| train/box_loss | 0.742 | 0.661 |
| val/box_loss | 2.355 | 2.152 |
| train/cls_loss | 0.357 | 0.277 |
| val/cls_loss | 2.386 | 2.661 |

- 训练/验证 loss 比：box **3.3×**，cls **9.6×**，明显过拟合
- mAP50 在第 53 轮见顶后横盘 47 轮，后 47 轮纯属浪费算力
- `val/cls_loss` 反而从 2.386 涨到 2.661，进一步印证

训练脚本已默认开启 `patience=20` 早停。

## 4. 分布外样本（最需要处理的问题）

验证集有 4 张图模型**完全检不出**：

| 图片 | 尺寸 | 真值 | conf 0.5 | conf 0.05 |
|---|---|---|---|---|
| `red.on.png` | 1024×1024 | `red_on` | 无 | `license_plate@0.06` |
| `red_off.png` | 1024×1024 | `red_off` | 无 | 无 |
| `yellow_off.png` | 1024×1024 | `yellow_off` | 无 | `resident@0.41` |
| `yellow_on.png` | 1024×1024 | `yellow_on` | 无 | 无 |

**根因**：这 4 张是 1024×1024 的纯色灯色特写图，而 285 张训练图**全部**是
480×640 真实场景图，且训练集中不含任何同类图。训练分布里完全没有这类样本，
属于典型的 OOD，调阈值无法解决。

### 关于旧版的文件名特判

旧 `test_vision.py` 里有一段逻辑：文件名含 `红灯亮`/`红灯暗` 时，**跳过模型**，
直接返回整张图 + 指定类别、`conf` 写死 0.9。这正是为了盖住上面这 4 张图。

两点问题：

1. **匹配不上。** 验证集里这 4 张实际叫 `red.on.png` / `red_off.png` 等英文名，
   而特判规则写的是中文 `红灯亮`/`红灯暗`。所以那段代码对它们**根本没生效**，
   旧 `val_result` 里那 2 个乱码中文名文件才是真正被特判命中的。
2. **不是预测结果。** 输出里那些 `conf=0.9` 的整图框是写死的常量。

本包的处理：同样能力保留为 `debug.force_full_image_boxes`，但
**默认关闭**、需 `--allow-force` 显式开启、结果标 `forced=true`、
json 汇总单独计数、批量跑时打 WARNING。

### 建议

1. 把这类整幅灯色特写图补进训练集；或对训练图做尺度/裁剪增广提升尺度鲁棒性。
2. 若小车实际不会遇到这种画面，从验证集剔除——否则指标长期被这 4 张拖住，
   且对模型改进毫无指导意义。
3. **不要**再引入文件名特判。

## 5. 未使用的数据

- `extra_unlabeled_imgs/` — 73 个文件（`robot-vision` 无标注图 + 若干灯色/车牌特写），
  不在 `data.yaml` 中，当前完全未被使用。可作为伪标注 / 补数据的素材来源。
- `robot-vision.ndjson`、`robot-vision (1).ndjson` — 标注源文件，整理数据时需要保留。

## 6. 磁盘

原目录内有大量可清理的重复权重：

- `weights.zip` — **849 MB**
- `epoch0.pt` … `epoch90.pt` — 10 个 × ~89 MB ≈ **900 MB**

`best.pt` / `last.pt` 各 22MB。日常只需要 `best.pt`。
新仓库的 `.gitignore` 已排除所有 `*.pt` / `*.zip`，不会误传。

## 7. 复现体检

```bash
python3 -m tl_vision.dataset --root /path/to/traffic_light_dataset check
python3 -m tl_vision.dataset --root /path/to/traffic_light_dataset stats
python3 -m tl_vision.dataset --root /path/to/traffic_light_dataset rebalance
```
