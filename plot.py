#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def as_float(value: str) -> float:
    if value == "":
        return float("nan")
    return float(value)


def yaw_error_deg(pred: float, truth: float) -> float:
    if not math.isfinite(pred) or not math.isfinite(truth):
        return float("nan")
    return (pred - truth + 180.0) % 360.0 - 180.0


def load_rows(path: Path):
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def pitch_error_deg(pred: float, truth: float) -> float:
    if not math.isfinite(pred) or not math.isfinite(truth):
        return float("nan")
    return pred - truth


def plot_csv(path: Path):
    rows = load_rows(path)
    if not rows:
        return

    t = [as_float(row["t"]) for row in rows]
    dt_ms = [as_float(row["dt_ms"]) for row in rows]
    raw_yaw = [as_float(row["raw_target_yaw_deg"]) for row in rows]
    pred_yaw = [as_float(row["target_yaw_deg"]) for row in rows]
    truth_yaw = [as_float(row["truth_target_yaw_deg"]) for row in rows]
    pred_pitch = [as_float(row["target_pitch_deg"]) for row in rows]
    truth_pitch = [as_float(row["truth_target_pitch_deg"]) for row in rows]
    pitch_err = [pitch_error_deg(pred, truth) for pred, truth in zip(pred_pitch, truth_pitch)]
    yaw_err = [yaw_error_deg(pred, truth) for pred, truth in zip(pred_yaw, truth_yaw)]
    stutter = [int(row["stutter"]) for row in rows]
    occluded = [int(row["occluded"]) for row in rows]

    fig, axes = plt.subplots(5, 1, figsize=(12, 13), sharex=True)

    axes[0].plot(t, pred_yaw, label="target_yaw", linewidth=1.5)
    axes[0].plot(t, truth_yaw, label="truth_yaw", linewidth=1.2, alpha=0.8)
    axes[0].scatter(t, raw_yaw, label="raw_target_yaw", s=10, color="tab:gray", alpha=0.45)
    occ_x = [x for x, flag in zip(t, occluded) if flag]
    stutter_x = [x for x, flag in zip(t, stutter) if flag]
    if occ_x:
        axes[0].scatter(occ_x, [0.0] * len(occ_x), s=9, c="tab:red", label="occluded", alpha=0.6)
    if stutter_x:
        axes[0].scatter(stutter_x, [0.0] * len(stutter_x), s=12, c="tab:orange", marker="x", label="stutter", alpha=0.8)
    axes[0].set_ylabel("yaw (deg)")
    axes[0].set_title(path.stem)
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(t, yaw_err, label="yaw_err", color="tab:red")
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("err (deg)")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best")

    axes[2].plot(t, dt_ms, label="dt_ms", color="tab:green")
    axes[2].set_ylabel("dt (ms)")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(loc="best")

    axes[3].plot(t, pred_pitch, label="target_pitch", color="tab:blue", linewidth=1.5)
    axes[3].plot(t, truth_pitch, label="truth_pitch", color="tab:orange", linewidth=1.2, alpha=0.8)
    axes[3].set_ylabel("pitch (deg)")
    axes[3].grid(True, alpha=0.25)
    axes[3].legend(loc="best")

    axes[4].plot(t, pitch_err, label="pitch_err", color="tab:red")
    axes[4].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[4].set_ylabel("err (deg)")
    axes[4].set_xlabel("time (s)")
    axes[4].grid(True, alpha=0.25)
    axes[4].legend(loc="best")

    fig.tight_layout()
    out_path = path.with_name(path.stem + "_target_yaw.png")
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    print(out_path)


def main():
    if len(sys.argv) < 2:
        print("usage: plot.py <csv-or-dir>", file=sys.stderr)
        return 1

    src = Path(sys.argv[1])
    if src.is_dir():
        csv_paths = sorted(src.rglob("*.csv"))
    else:
        csv_paths = [src]

    if not csv_paths:
        print("no csv found", file=sys.stderr)
        return 1

    for path in csv_paths:
        plot_csv(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
