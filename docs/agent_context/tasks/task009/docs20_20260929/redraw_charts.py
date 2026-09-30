#!/usr/bin/env python3
"""Redraw the two report figures from the frozen 20-run CSV."""

import csv
from pathlib import Path
from statistics import mean, median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


ROOT = Path(__file__).resolve().parent
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fontManager.addfont(FONT_PATH)
FONT = FontProperties(fname=FONT_PATH)
plt.rcParams.update({
    "font.family": FONT.get_name(),
    "axes.unicode_minus": False,
    "font.size": 11,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
})

with (ROOT / "all_twenty_runs.csv").open(newline="", encoding="utf-8") as handle:
    rows = sorted(csv.DictReader(handle), key=lambda row: int(row["ordinal"]))

assert len(rows) == 20
assert [int(row["ordinal"]) for row in rows] == list(range(1, 21))

x = [int(row["ordinal"]) for row in rows]
accepted = [row["independent_acceptance"] == "True" for row in rows]
duration = [float(row["mission_sim_s"]) for row in rows]
amcl_position = [float(row["home_amcl_position_cm"]) for row in rows]
physical_position = [float(row["home_physical_chassis_return_cm"]) for row in rows]
amcl_heading = [float(row["home_amcl_heading_rad"]) for row in rows]
physical_heading = [float(row["home_physical_yaw_return_rad"]) for row in rows]
physical_pass = [row["home_physical_3cm_004rad_pass"] == "True" for row in rows]

assert sum(accepted) == 16
assert sum(physical_pass) == 0

GREEN = "#168477"
RED = "#C24D4D"
BLUE = "#3077B5"
ORANGE = "#E68A28"
INK = "#233142"
GRID = "#E5EAF0"


def finish(fig, name):
    for suffix in ("png", "svg"):
        fig.savefig(ROOT / f"{name}.{suffix}", dpi=180, bbox_inches="tight")
    plt.close(fig)


fig, ax = plt.subplots(figsize=(13.2, 5.5), layout="constrained")
fig.suptitle("20 次完整任务耗时", fontsize=19, color=INK, fontweight="bold")
ax.bar(x, duration, color=[GREEN if ok else RED for ok in accepted], width=0.72)
ax.axhline(median(duration), color=INK, linestyle="--", linewidth=1.5, zorder=3)
ax.annotate(f"中位数 {median(duration):.3f} 秒", (20.4, median(duration) + 0.7),
            ha="right", va="bottom", fontsize=10, color=INK)
ax.set(xlim=(0.3, 20.7), ylim=(0, 166), xticks=x, xlabel="任务序号", ylabel="仿真时间（秒）")
ax.grid(axis="y", color=GRID, linewidth=0.8)
ax.set_axisbelow(True)
ax.legend(handles=[Patch(facecolor=GREEN, label="独立审计通过：16/20"),
                   Patch(facecolor=RED, label="独立审计失败：第 1、5、15、16 轮")],
          loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.02))
ax.text(0, -0.18,
        f"全体均值 {mean(duration):.3f} 秒  ·  范围 {min(duration):.3f}–{max(duration):.3f} 秒"
        "  ·  来源：all_twenty_runs.csv / 原独立审计",
        transform=ax.transAxes, fontsize=10, color="#526273")
finish(fig, "mission_times")


fig, axes = plt.subplots(1, 2, figsize=(14, 5.4), sharex=True, layout="constrained")
fig.suptitle("HOME 回位：定位估计与实际车身", fontsize=19, color=INK, fontweight="bold")
for ax in axes:
    ax.set(xlim=(0.6, 20.4), xticks=[1, 5, 10, 15, 20], xlabel="任务序号")
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)

axes[0].plot(x, amcl_position, "o-", color=BLUE, linewidth=1.8, markersize=4.5)
axes[0].plot(x, physical_position, "s-", color=ORANGE, linewidth=1.8, markersize=4.5)
axes[0].axhline(3, color=RED, linestyle="--", linewidth=1.4)
axes[0].set(ylabel="位置差（厘米）", ylim=(0, 8.1), title="位置差")
axes[0].annotate("3 厘米门槛", (19.7, 3.07), ha="right", color=RED, fontsize=9)

axes[1].plot(x, amcl_heading, "o-", color=BLUE, linewidth=1.8, markersize=4.5)
axes[1].plot(x, physical_heading, "s-", color=ORANGE, linewidth=1.8, markersize=4.5)
axes[1].axhline(0.04, color=RED, linestyle="--", linewidth=1.4)
axes[1].set(ylabel="方向差（弧度）", ylim=(0, 0.255), title="方向差")
axes[1].annotate("0.04 弧度门槛", (19.7, 0.045), ha="right", color=RED, fontsize=9)

axes[0].legend(handles=[Line2D([], [], color=BLUE, marker="o", label="AMCL/map 估计"),
                        Line2D([], [], color=ORANGE, marker="s", label="Gazebo 车身首末帧差"),
                        Line2D([], [], color=RED, linestyle="--", label="物理回位门槛")],
               loc="upper right", frameon=True, facecolor="white", edgecolor="none", fontsize=9)
fig.text(0.5, -0.035,
         "实际车身同时满足位置 ≤3 厘米、方向 ≤0.04 弧度：0/20  ·  来源：all_twenty_runs.csv",
         ha="center", fontsize=10, color="#526273")
finish(fig, "home_errors")
