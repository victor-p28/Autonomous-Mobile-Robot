"""Generate the README figures: docs/ekf_path.png and docs/ekf_run.gif.

Run from anywhere:  python sim/make_figures.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from diff_drive_sim import EKF, Controller, Robot, Simulations

DOCS = Path(__file__).resolve().parent.parent / "docs"
X_TARGET, Y_TARGET, DT, NOISE = 8, 12, 0.1, 0.5

TRUE_C, MEAS_C, EST_C, TARGET_C = "#1f2937", "#9ca3af", "#2563eb", "#dc2626"


def run_ekf(seed=3):
    np.random.seed(seed)
    sim = Simulations(DT, X_TARGET, Y_TARGET,
                      Controller(0.8, 0.1, 0), Controller(0.8, 0.1, 0),
                      Robot(), noise=NOISE, EKF=EKF())
    sim.run()
    return sim


def style(ax):
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.spines[["top", "right"]].set_visible(False)


def path_plot(sim):
    fig, ax = plt.subplots(figsize=(6, 7))
    ax.scatter(sim.noisy_x, sim.noisy_y, s=10, color=MEAS_C, label="Noisy position measurements")
    ax.plot(sim.x, sim.y, color=TRUE_C, lw=2, label="True path")
    ax.plot(sim.estimated_x, sim.estimated_y, color=EST_C, lw=2, label="EKF estimate")
    ax.plot(X_TARGET, Y_TARGET, "*", color=TARGET_C, markersize=16, label="Target")
    ax.set_title("EKF tracking a noisy diff-drive robot")
    style(ax)
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(DOCS / "ekf_path.png", dpi=150)
    plt.close(fig)


def gif(sim, stride=2):
    fig, ax = plt.subplots(figsize=(5, 6))
    ax.set_xlim(-2, X_TARGET + 2)
    ax.set_ylim(-2, Y_TARGET + 2)
    style(ax)
    ax.plot(X_TARGET, Y_TARGET, "*", color=TARGET_C, markersize=16, label="Target")
    meas = ax.scatter([], [], s=10, color=MEAS_C, label="Measurements")
    true_line, = ax.plot([], [], color=TRUE_C, lw=2, label="True path")
    est_line, = ax.plot([], [], color=EST_C, lw=2, label="EKF estimate")
    robot, = ax.plot([], [], "o", color=TRUE_C, markersize=8)
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()

    frames = list(range(1, len(sim.x), stride)) + [len(sim.x) - 1] * 15

    def update(i):
        meas.set_offsets(np.column_stack([sim.noisy_x[:i], sim.noisy_y[:i]]))
        true_line.set_data(sim.x[:i], sim.y[:i])
        est_line.set_data(sim.estimated_x[:i], sim.estimated_y[:i])
        robot.set_data([sim.x[i]], [sim.y[i]])
        return meas, true_line, est_line, robot

    ani = animation.FuncAnimation(fig, update, frames=frames, interval=50)
    ani.save(DOCS / "ekf_run.gif", writer=animation.PillowWriter(fps=20), dpi=80)
    plt.close(fig)


if __name__ == "__main__":
    DOCS.mkdir(exist_ok=True)
    sim = run_ekf()
    path_plot(sim)
    gif(sim)
    print(f"Wrote figures to {DOCS}")
