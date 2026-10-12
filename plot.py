#!/usr/bin/env python3
"""results/<seq>.csv -> results/<seq>_sweep.png (error vs level) and results/<seq>_clean_traj.png."""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from bench import gt_velo  # noqa: E402

OUT = Path(__file__).parent / "results"
INK, MUTED, GRID = "#0b0b0b", "#898781", "#e1e0d9"
COLOR = {"kiss": "#2a78d6", "gicp": "#eb6834", "icp": "#1baf7a", "vgicp": "#eda100"}  # fixed categorical order
NAME = {"kiss": "KISS-ICP (default)", "gicp": "small_gicp GICP, scan-to-scan", "icp": "small_gicp point-to-point ICP, scan-to-scan",
        "vgicp": "small_gicp VGICP, scan-to-model"}
PANELS = {  # degradation -> (title, clean tick, level -> tick)
    "beams": ("Beam decimation (rings)", "64", lambda v: f"{int(v)}"),
    "dropout": ("Random point dropout", "0%", lambda v: f"{v:.0%}"),
    "noise": ("Range noise σ along ray", "0", lambda v: f"{v * 100:g} cm"),
    "maxrange": ("Max-range truncation", "none", lambda v: f"{v:g} m"),
    "rate": ("Scan rate (frame skip)", "10 Hz", lambda v: f"{10 / v:g} Hz"),
    "rate_gtseed": ("Scan rate, GT-seeded first step", "10 Hz", lambda v: f"{10 / v:g} Hz"),
}
ROWS = [("ate_rmse_m", "ATE RMSE [m]\n(SE(3)-aligned)"), ("rpe10_m", "RPE over 10 m [m]\n(translation RMSE)")]


def main(seq):
    rows = list(csv.DictReader(open(OUT / f"{seq}.csv")))
    methods = [m for m in COLOR if any(r["method"] == m for r in rows)]
    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": GRID,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlesize": 10})
    fig, axes = plt.subplots(len(ROWS), len(PANELS), figsize=(19, 6.4), sharey="row", facecolor="#fcfcfb")
    for (metric, ylabel), axrow in zip(ROWS, axes):
        for (kind, (title, clean_tick, fmt)), ax in zip(PANELS.items(), axrow):
            ax.set_facecolor("#fcfcfb")
            ax.grid(axis="y", color=GRID, lw=0.6)
            ax.set_yscale("log")
            levels = sorted({float(r["level"]) for r in rows if r["degradation"] == kind},
                            reverse=kind in ("beams", "maxrange"))  # worse to the right in every panel
            for m in methods:
                by = defaultdict(list)  # x index -> [(value, diverged)] over seeds
                for r in rows:
                    if r["method"] != m or not r[metric]:
                        continue
                    if r["degradation"] == "clean":
                        x = 0
                    elif r["degradation"] == kind:
                        x = 1 + levels.index(float(r["level"]))
                    else:
                        continue
                    by[x].append((float(r[metric]), r["diverged"] == "True"))
                xs = sorted(by)
                mean = [np.mean([v for v, _ in by[x]]) for x in xs]
                lo = [mean[i] - min(v for v, _ in by[x]) for i, x in enumerate(xs)]
                hi = [max(v for v, _ in by[x]) - mean[i] for i, x in enumerate(xs)]
                ax.errorbar(xs, mean, yerr=[lo, hi], color=COLOR[m], lw=2, marker="o", ms=5, capsize=2,
                            label=NAME[m])
                bad = [(x, v) for x in xs for v, d in by[x] if d]
                if bad:
                    ax.scatter(*zip(*bad), marker="x", s=60, color=INK, zorder=5, lw=1.5)
            ax.set_xticks(range(len(levels) + 1), [clean_tick] + [fmt(v) for v in levels])
            if metric == ROWS[0][0]:
                ax.set_title(title)
        axrow[0].set_ylabel(ylabel)
    handles, labels = axes[0][0].get_legend_handles_labels()
    handles.append(plt.Line2D([], [], color=INK, marker="x", ls="", ms=7))
    labels.append("diverged (KITTI drift > 10 %)")
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.955), ncol=len(labels), frameon=False)
    n = max(int(r["n_scans"]) for r in rows)
    fig.suptitle(f"KITTI odometry seq {seq} ({n} scans): error vs degradation level. Left-most tick = clean input; "
                 "bars = min/max over seeds", y=0.995, fontsize=10, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(OUT / f"{seq}_sweep.png", dpi=130)

    gt = gt_velo(seq)[:, :2, 3]
    fig, ax = plt.subplots(figsize=(7, 4), facecolor="#fcfcfb")
    ax.plot(gt[:, 0], gt[:, 1], color=INK, lw=2, ls="--", label="ground truth (velodyne frame)")
    for m in methods:
        est = np.loadtxt(OUT / "traj" / f"{seq}_{m}_clean_0_0.txt")
        ax.plot(est[:, 3], est[:, 7], color=COLOR[m], lw=2, label=NAME[m])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(color=GRID, lw=0.6)
    ax.legend(frameon=False)
    ax.set_title(f"Seq {seq}, clean input (unaligned, all start at identity)")
    fig.tight_layout()
    fig.savefig(OUT / f"{seq}_clean_traj.png", dpi=130)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "04")
