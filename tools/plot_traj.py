"""
plot_traj.py — Compare INS (car) vs virtual (vst) trajectories from serial log.

Usage:
    uv run plot_traj.py <log.txt>
    uv run plot_traj.py log1.txt log2.txt
"""

import sys
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_log(path: str) -> dict[str, np.ndarray]:
    """Parse CSV data rows from serial log."""
    rows = []
    header = ("t", "car_x", "car_y", "car_th", "vst_x", "vst_y", "vst_th")
    ncols = len(header)

    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # skip non-data lines (comments, headers, etc.)
            if line.startswith("#") or not re.match(r"^[\d.\-]", line):
                continue
            parts = line.split(",")
            if len(parts) < ncols:
                continue
            try:
                row = [float(p) for p in parts[:ncols]]
            except ValueError:
                continue
            rows.append(row)

    if not rows:
        raise SystemExit(f"未找到有效数据行: {path}")

    arr = np.array(rows)
    return {k: arr[:, i] for i, k in enumerate(header)}


# Local waypoints (MODE 2/4)
WAYPOINTS = np.array([
    [1.715,  0.815],
    [3.445,  1.43],
    [4.565,  0.095],
    [2.965, -0.075],
    [3.70,  -1.48],
    [1.91,  -1.09],
])

# Start position
START = np.array([0.0, 0.0])


def plot_trajectories(files: list[str]):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Trajectory: INS (car) vs Virtual (vst)", fontsize=13)

    ax_xy = axes[0]
    ax_xy.set_title("XY Plane")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3)

    ax_t = axes[1]
    ax_t.set_title("Heading over Time")
    ax_t.set_xlabel("t [s]")
    ax_t.set_ylabel("theta [deg]")
    ax_t.grid(True, alpha=0.3)

    colors = plt.cm.tab10.colors

    for fi, path in enumerate(files):
        data = parse_log(path)
        label = Path(path).stem
        c = colors[fi % len(colors)]

        # XY trajectory
        ax_xy.plot(data["car_x"], data["car_y"], "-",  color=c, linewidth=1.2,
                   label=f"{label} car")
        ax_xy.plot(data["vst_x"], data["vst_y"], "--", color=c, linewidth=0.8,
                   label=f"{label} vst")
        # start markers
        ax_xy.scatter(data["car_x"][0], data["car_y"][0], color=c, marker="o", s=40, zorder=5)
        ax_xy.scatter(data["vst_x"][0], data["vst_y"][0], color=c, marker="s", s=30, zorder=5)

        # heading
        ax_t.plot(data["t"], data["car_th"], "-",  color=c, linewidth=1.2,
                  label=f"{label} car")
        ax_t.plot(data["t"], data["vst_th"], "--", color=c, linewidth=0.8,
                  label=f"{label} vst")

    # ── waypoints ──
    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax_xy.scatter(wx, wy, marker="x", color="red", s=60, zorder=6)
        ax_xy.annotate(str(i), (wx, wy), textcoords="offset points",
                       xytext=(6, 6), fontsize=8, color="red")
    ax_xy.scatter(*START, marker="*", color="gold", s=100, zorder=6, label="Start")

    ax_xy.legend(fontsize=7, loc="best")
    ax_t.legend(fontsize=7, loc="best")
    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("用法: uv run plot_traj.py <log.txt> [log2.txt ...]")
        sys.exit(1)
    plot_trajectories(sys.argv[1:])


if __name__ == "__main__":
    main()
