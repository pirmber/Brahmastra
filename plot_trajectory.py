#!/usr/bin/env python3
"""
plot_trajectory.py — Missile Guidance Simulation Visualizer
============================================================
Reads output/trajectory.csv and produces:
  1. An animated real-time trajectory plot
  2. A static summary figure saved as output/trajectory_summary.png

Usage:
    python3 scripts/plot_trajectory.py [--static] [--csv path/to/file.csv]

Options:
    --static   Skip animation, only produce static summary image
    --csv      Override CSV path (default: output/trajectory.csv)
"""

import sys
import os
import argparse
import math
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

# ── Aesthetics ────────────────────────────────────────────────────────────────
BG_COLOR      = "#0d1117"
GRID_COLOR    = "#1e2a38"
MISSILE_COLOR = "#00e5ff"
TARGET_COLOR  = "#ff4757"
TRAIL_COLOR   = "#00bcd4"
TGT_TRL_COLOR = "#ff6b81"
TEXT_COLOR    = "#c9d1d9"
ACCENT_COLOR  = "#ffd700"
HIT_COLOR     = "#69ff47"

matplotlib.rcParams.update({
    "figure.facecolor": BG_COLOR,
    "axes.facecolor":   BG_COLOR,
    "axes.edgecolor":   GRID_COLOR,
    "axes.labelcolor":  TEXT_COLOR,
    "xtick.color":      TEXT_COLOR,
    "ytick.color":      TEXT_COLOR,
    "text.color":       TEXT_COLOR,
    "grid.color":       GRID_COLOR,
    "grid.linewidth":   0.5,
    "font.family":      "monospace",
})


def load_data(csv_path: str) -> pd.DataFrame:
    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV file not found: {csv_path}")
        sys.exit(1)
    df = pd.read_csv(csv_path)
    required = ["time_s", "missile_x", "missile_y",
                "target_x", "target_y", "distance_m", "missile_speed"]
    for col in required:
        if col not in df.columns:
            print(f"[ERROR] Missing column '{col}' in CSV")
            sys.exit(1)
    return df


def compute_stats(df: pd.DataFrame) -> dict:
    intercept_row = df[df["distance_m"] <= 5.0]
    intercept_time = intercept_row["time_s"].values[0] if len(intercept_row) > 0 else None
    min_dist_idx  = df["distance_m"].idxmin()
    return {
        "intercept":      intercept_time is not None,
        "intercept_time": intercept_time,
        "total_time":     df["time_s"].iloc[-1],
        "min_dist":       df["distance_m"].min(),
        "min_dist_time":  df["time_s"].iloc[min_dist_idx],
        "max_speed":      df["missile_speed"].max(),
        "avg_speed":      df["missile_speed"].mean(),
        "n_points":       len(df),
    }


def make_static_figure(df: pd.DataFrame, stats: dict, out_path: str):
    fig = plt.figure(figsize=(16, 9), dpi=140)
    fig.suptitle("Missile Guidance Simulation — Trajectory Analysis",
                 fontsize=14, color=TEXT_COLOR, y=0.98)

    gs = GridSpec(2, 3, figure=fig, hspace=0.38, wspace=0.35,
                  left=0.06, right=0.97, top=0.93, bottom=0.07)

    # ── Trajectory plot ──────────────────────────────────────────────────────
    ax_traj = fig.add_subplot(gs[:, :2])
    ax_traj.set_title("2D Trajectory", color=TEXT_COLOR, pad=8)
    ax_traj.set_xlabel("X (m)")
    ax_traj.set_ylabel("Y (m)")
    ax_traj.set_aspect("equal")
    ax_traj.grid(True, alpha=0.4)

    n = len(df)
    alpha_arr = np.linspace(0.1, 1.0, n)

    # Missile trail with gradient alpha
    for i in range(1, n):
        ax_traj.plot([df["missile_x"].iloc[i-1], df["missile_x"].iloc[i]],
                     [df["missile_y"].iloc[i-1], df["missile_y"].iloc[i]],
                     color=TRAIL_COLOR, alpha=float(alpha_arr[i]), linewidth=1.2)

    # Target trail
    for i in range(1, n):
        ax_traj.plot([df["target_x"].iloc[i-1], df["target_x"].iloc[i]],
                     [df["target_y"].iloc[i-1], df["target_y"].iloc[i]],
                     color=TGT_TRL_COLOR, alpha=float(alpha_arr[i]) * 0.6, linewidth=1.0)

    # Markers
    ax_traj.scatter(df["missile_x"].iloc[0], df["missile_y"].iloc[0],
                    s=80, color=MISSILE_COLOR, zorder=5, label="Missile start")
    ax_traj.scatter(df["target_x"].iloc[0], df["target_y"].iloc[0],
                    s=80, color=TARGET_COLOR, marker="^", zorder=5, label="Target start")

    if stats["intercept"]:
        ix = df.iloc[-1]["missile_x"]
        iy = df.iloc[-1]["missile_y"]
        circle = plt.Circle((ix, iy), 5.0, color=HIT_COLOR, fill=False,
                              linewidth=2.0, linestyle="--", zorder=6)
        ax_traj.add_patch(circle)
        ax_traj.scatter(ix, iy, s=120, color=HIT_COLOR, marker="*",
                        zorder=7, label=f"Intercept @ t={stats['intercept_time']:.2f}s")

    ax_traj.legend(loc="upper right", fontsize=8,
                   facecolor="#1e2a38", edgecolor=GRID_COLOR)

    # ── Distance over time ───────────────────────────────────────────────────
    ax_dist = fig.add_subplot(gs[0, 2])
    ax_dist.set_title("Miss Distance vs Time", color=TEXT_COLOR, pad=6)
    ax_dist.set_xlabel("Time (s)")
    ax_dist.set_ylabel("Distance (m)")
    ax_dist.grid(True, alpha=0.4)
    ax_dist.plot(df["time_s"], df["distance_m"],
                 color=ACCENT_COLOR, linewidth=1.4, label="Distance")
    ax_dist.axhline(5.0, color=HIT_COLOR, linewidth=0.8,
                    linestyle="--", label="Kill radius (5m)")
    ax_dist.legend(fontsize=7, facecolor="#1e2a38", edgecolor=GRID_COLOR)

    # ── Speed over time ──────────────────────────────────────────────────────
    ax_speed = fig.add_subplot(gs[1, 2])
    ax_speed.set_title("Missile Speed vs Time", color=TEXT_COLOR, pad=6)
    ax_speed.set_xlabel("Time (s)")
    ax_speed.set_ylabel("Speed (m/s)")
    ax_speed.grid(True, alpha=0.4)
    ax_speed.plot(df["time_s"], df["missile_speed"],
                  color=MISSILE_COLOR, linewidth=1.4)
    ax_speed.fill_between(df["time_s"], df["missile_speed"],
                          alpha=0.15, color=MISSILE_COLOR)

    # ── Stats text box ───────────────────────────────────────────────────────
    status  = f"{'✓ INTERCEPT' if stats['intercept'] else '✗ MISS'}"
    s_color = HIT_COLOR if stats["intercept"] else TARGET_COLOR
    stats_text = (
        f"{'Status':<18}: {status}\n"
        f"{'Total time':<18}: {stats['total_time']:.3f} s\n"
        f"{'Min distance':<18}: {stats['min_dist']:.2f} m\n"
        f"{'Min dist time':<18}: {stats['min_dist_time']:.3f} s\n"
        f"{'Max speed':<18}: {stats['max_speed']:.1f} m/s\n"
        f"{'Avg speed':<18}: {stats['avg_speed']:.1f} m/s\n"
        f"{'Data points':<18}: {stats['n_points']}"
    )
    fig.text(0.06, 0.01, stats_text, fontsize=8, color=TEXT_COLOR,
             va="bottom", family="monospace",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#1e2a38",
                       edgecolor=GRID_COLOR, alpha=0.9))

    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[OK] Static figure saved: {out_path}")
    return fig


def animate(df: pd.DataFrame, stats: dict):
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle("Missile Guidance Simulation — Live Trajectory",
                 color=TEXT_COLOR, fontsize=13)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.4)

    # Padding
    xmin = min(df["missile_x"].min(), df["target_x"].min())
    xmax = max(df["missile_x"].max(), df["target_x"].max())
    ymin = min(df["missile_y"].min(), df["target_y"].min())
    ymax = max(df["missile_y"].max(), df["target_y"].max())
    pad  = max((xmax - xmin), (ymax - ymin)) * 0.1 + 50
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    msl_trail, = ax.plot([], [], color=TRAIL_COLOR, lw=1.5, alpha=0.8, label="Missile")
    tgt_trail, = ax.plot([], [], color=TGT_TRL_COLOR, lw=1.0, alpha=0.5, label="Target")
    msl_dot,   = ax.plot([], [], "o", color=MISSILE_COLOR, ms=8, zorder=5)
    tgt_dot,   = ax.plot([], [], "^", color=TARGET_COLOR, ms=8, zorder=5)

    info_text = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                        fontsize=8, va="top", color=TEXT_COLOR, family="monospace",
                        bbox=dict(boxstyle="round", facecolor="#1e2a38",
                                  edgecolor=GRID_COLOR, alpha=0.85))

    ax.legend(loc="upper right", fontsize=9, facecolor="#1e2a38",
              edgecolor=GRID_COLOR)

    step = max(1, len(df) // 600)   # target ~600 animation frames

    def update(frame):
        idx = frame * step
        if idx >= len(df): idx = len(df) - 1
        row = df.iloc[idx]

        msl_trail.set_data(df["missile_x"][:idx+1], df["missile_y"][:idx+1])
        tgt_trail.set_data(df["target_x"][:idx+1],  df["target_y"][:idx+1])
        msl_dot.set_data([row["missile_x"]], [row["missile_y"]])
        tgt_dot.set_data([row["target_x"]],  [row["target_y"]])

        info_text.set_text(
            f"t      = {row['time_s']:.2f} s\n"
            f"dist   = {row['distance_m']:.1f} m\n"
            f"speed  = {row['missile_speed']:.1f} m/s"
        )
        return msl_trail, tgt_trail, msl_dot, tgt_dot, info_text

    n_frames = math.ceil(len(df) / step)
    ani = FuncAnimation(fig, update, frames=n_frames,
                        interval=16, blit=True, repeat=False)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Missile sim trajectory plotter")
    parser.add_argument("--static", action="store_true",
                        help="Only produce static PNG, skip animation")
    parser.add_argument("--csv", default="output/trajectory.csv",
                        help="Path to CSV file")
    args = parser.parse_args()

    df    = load_data(args.csv)
    stats = compute_stats(df)

    print(f"[INFO] Loaded {len(df)} data points from '{args.csv}'")
    print(f"[INFO] Duration: {stats['total_time']:.3f}s | "
          f"Min dist: {stats['min_dist']:.2f}m | "
          f"{'INTERCEPT' if stats['intercept'] else 'MISS'}")

    out_png = os.path.join(os.path.dirname(args.csv), "trajectory_summary.png")
    make_static_figure(df, stats, out_png)

    if not args.static:
        animate(df, stats)


if __name__ == "__main__":
    main()
