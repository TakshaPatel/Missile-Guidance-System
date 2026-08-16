#!/usr/bin/env python3
#               vis
import sys

import numpy as np
import matplotlib.pyplot as plt


def load(csv_path):
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = data["t"]
    m = np.column_stack([data["mx"], data["my"], data["mz"]])
    tg = np.column_stack([data["tx"], data["ty"], data["tz"]])
    return t, m, tg


def main(csv_path):
    t, m, tg = load(csv_path)

    fig = plt.figure(figsize=(14, 5.5))
    fig.suptitle("Missile Guidance: Proportional Navigation Intercept", fontsize=14)

    ax3d = fig.add_subplot(121, projection="3d")
    ax3d.plot(*m.T, color="red", lw=2, label="Missile")
    ax3d.plot(*tg.T, color="blue", lw=2, ls="--", label="Target")
    ax3d.scatter(*m[0], color="red", marker="o", s=60, label="Missile start")
    ax3d.scatter(*tg[0], color="blue", marker="o", s=60, label="Target start")
    ax3d.scatter(*m[-1], color="darkred", marker="x", s=80, label="Missile end")
    ax3d.scatter(*tg[-1], color="navy", marker="x", s=80, label="Target end")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.legend(loc="upper left", fontsize=8)
    ax3d.set_title(f"3D trajectories\nimpact t={t[-1]:.2f}s")

    ax2d = fig.add_subplot(122)
    miss = np.linalg.norm(m - tg, axis=1)
    ax2d.plot(t, miss, color="purple", lw=2)
    ax2d.axhline(5.0, color="gray", ls=":", label="impact dist (5 m)")
    ax2d.set_xlabel("time (s)")
    ax2d.set_ylabel("miss distance (m)")
    ax2d.set_title(f"Range vs time\nfinal miss = {miss[-1]:.2f} m")
    ax2d.legend(fontsize=8)
    ax2d.grid(alpha=0.3)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "trajectory.csv"
    main(path)
