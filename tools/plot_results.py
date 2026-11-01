#!/usr/bin/env python3
"""Draw the before/after figures used in the README and interviews.

Reads the recorded runs under data/ and writes PNGs to doc/figures/:
  speed_step.png    0.1 m/s ground step, NVS gains (ki=0.5) vs tuned (kp=1.2, ki=4)
  odom_gaps.png     /odom inter-arrival gaps before and after the micro-ROS spin fix
  imu_heading.png   IMU vs odometry heading while still and driving straight,
                    before and after the gyro bias tracking

Usage:
  python3 tools/plot_results.py [--out doc/figures]
"""

import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# Reference palette (dataviz skill): categorical slots 1-2 and light-mode text.
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
AFTER = "#2a78d6"   # slot 1 blue: the fixed / tuned / reference series
BEFORE = "#eb6834"  # slot 2 orange: the original behaviour

CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def setup_style():
    if os.path.exists(CJK_FONT):
        font_manager.fontManager.addfont(CJK_FONT)
        family = font_manager.FontProperties(fname=CJK_FONT).get_name()
        plt.rcParams["font.family"] = [family, "DejaVu Sans"]
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.linewidth": 1.0,
        "axes.labelcolor": TEXT_2,
        "axes.titlecolor": TEXT,
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 1.0,
        "grid.linestyle": "-",
        "xtick.color": TEXT_2,
        "ytick.color": TEXT_2,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.unicode_minus": False,
        "lines.linewidth": 2.0,
        "lines.solid_capstyle": "round",
        "lines.solid_joinstyle": "round",
        "legend.frameon": False,
        "legend.fontsize": 9,
        "savefig.dpi": 160,
        "savefig.facecolor": SURFACE,
    })


def despine(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.set_axisbelow(True)


def read_csv(rel):
    with open(os.path.join(ROOT, rel), newline="") as f:
        return list(csv.DictReader(f))


def first_cross(samples, level):
    return next((t for t, v in samples if v >= level), None)


def speed_step(out):
    def series(rel, target="0.1"):
        rows = read_csv(rel)
        return [(float(r["t_since_step_s"]), float(r["vx_mps"])) for r in rows
                if abs(float(r["target_mps"]) - float(target)) < 1e-9]

    before = series("data/baseline_20261023_ground/speed_steps.csv")
    after = series("data/pid_grid_20261023_ground/kp1.2_ki4.0_r1.csv")

    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    despine(ax)
    ax.axhline(0.1, color=TEXT_2, linewidth=1.0)
    ax.text(3.0, 0.1015, "目标 0.1 m/s", color=TEXT_2, fontsize=9, ha="right", va="bottom")
    for samples, color, label in ((before, BEFORE, "调参前（NVS 中 kp=1.2, ki=0.5）"),
                                  (after, AFTER, "调参后（kp=1.2, ki=4）")):
        ax.plot([t for t, _ in samples], [v for _, v in samples], color=color, label=label)
        t90 = first_cross(samples, 0.09)
        if t90 is not None:
            ax.plot([t90], [0.09], "o", markersize=8, color=color, markeredgecolor=SURFACE,
                    markeredgewidth=2, zorder=5)
            ax.annotate(f"到达 90%：{t90:.2f} s", (t90, 0.09), xytext=(8, -16),
                        textcoords="offset points", color=TEXT, fontsize=9)
    ax.set_xlim(0, 3.0)
    ax.set_ylim(0, 0.125)
    ax.set_xlabel("阶跃后时间（s）")
    ax.set_ylabel("车体速度 /odom（m/s）")
    ax.set_title("速度阶跃 0.1 m/s（地面）：稳态误差 -10.8% → 1% 以内")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "speed_step.png"))
    plt.close(fig)


def odom_gaps(out):
    def gaps(rel):
        times = [float(r["t_s"]) for r in read_csv(rel) if r["topic"] == "odom"]
        return [(b - a) * 1000.0 for a, b in zip(times, times[1:])]

    panels = (("修复前：34 Hz，常有整周期漏发", gaps("data/rt_analysis_20261025/rates_idle_before_fix.csv"), BEFORE),
              ("修复后：50 Hz", gaps("data/rt_analysis_20261025/rates_idle_after_fix.csv"), AFTER))
    bins = list(range(0, 85, 5))
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.4), sharey=True)
    for ax, (title, values, color) in zip(axes, panels):
        despine(ax)
        weights = [100.0 / len(values)] * len(values)
        ax.hist([min(v, 79.9) for v in values], bins=bins, weights=weights, color=color,
                rwidth=0.82)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("相邻两条 /odom 的间隔（ms）")
        ax.set_xticks([0, 20, 40, 60, 80])
        ax.text(0.98, 0.95, f"{len(values) + 1} 条，60 s 静止", transform=ax.transAxes,
                ha="right", va="top", color=TEXT_2, fontsize=9)
    axes[0].set_ylabel("占比（%）")
    axes[0].annotate("40 ms：漏掉一整个 20 ms 周期", (42.5, 20.1), xytext=(46, 27),
                     color=TEXT, fontsize=9, arrowprops=dict(arrowstyle="-", color=TEXT_2, lw=1))
    fig.suptitle("micro-ROS 发布间隔：每轮等待 20 ms 改为 5 ms", x=0.01, ha="left",
                 fontsize=12, fontweight="bold", color=TEXT)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "odom_gaps.png"))
    plt.close(fig)


def imu_heading(out):
    def headings(rel):
        """Heading change from the start of the still phase, per source, and the
        time the straight phase starts, both on the same time base."""
        rows = [r for r in read_csv(rel) if r["phase"] in ("still", "straight")]
        t0 = min(float(r["t_s"]) for r in rows)
        result = {}
        for source in ("odom", "imu"):
            samples = [(float(r["t_s"]), float(r["yaw_rad"])) for r in rows if r["source"] == source]
            total, last, unwrapped = 0.0, samples[0][1], []
            for t, yaw in samples:
                total += math.atan2(math.sin(yaw - last), math.cos(yaw - last))
                last = yaw
                unwrapped.append((t - t0, math.degrees(total)))
            result[source] = unwrapped
        straight_start = min(float(r["t_s"]) for r in rows if r["phase"] == "straight")
        return result, straight_start - t0

    panels = (("修复前（开机约 1 小时）", "data/imu_heading_20261024/run1.csv"),
              ("修复后（开机 29 分钟）", "data/imu_heading_20261024/fixed_run3_warm.csv"))
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.6), sharey=True)
    for ax, (title, rel) in zip(axes, panels):
        despine(ax)
        data, straight_t = headings(rel)
        ax.axvline(straight_t, color=TEXT_2, linewidth=1.0)
        ax.text(straight_t / 2, 0.98, "静止 20 s", transform=ax.get_xaxis_transform(),
                ha="center", va="top", color=TEXT_2, fontsize=9)
        ax.text(straight_t + 0.5, 0.98, "直行 8 s", transform=ax.get_xaxis_transform(),
                ha="left", va="top", color=TEXT_2, fontsize=9)
        for source, color, label in (("odom", AFTER, "里程计"), ("imu", BEFORE, "IMU")):
            pts = data[source]
            ax.plot([t for t, _ in pts], [y for _, y in pts], color=color, label=label)
            t_end, y_end = pts[-1]
            ax.plot([t_end], [y_end], "o", markersize=8, color=color, markeredgecolor=SURFACE,
                    markeredgewidth=2, zorder=5)
            ax.annotate(f"{y_end:+.1f}°", (t_end, y_end), xytext=(6, 0), textcoords="offset points",
                        va="center", color=TEXT, fontsize=9)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("时间（s）")
        ax.set_xlim(0, None)
    axes[0].set_ylabel("航向变化（°）")
    axes[0].legend(loc="center left")
    fig.suptitle("IMU 航向：静止时在线跟踪陀螺仪零偏", x=0.01, ha="left",
                 fontsize=12, fontweight="bold", color=TEXT)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "imu_heading.png"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=os.path.join(ROOT, "doc", "figures"))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    setup_style()
    speed_step(args.out)
    odom_gaps(args.out)
    imu_heading(args.out)
    print(f"figures written to {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
