from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd

from MISSILE_TRUTH import Missile, missile_initial_state


TABLE_PATH = Path(__file__).with_name("INITIAL_TURN_STATE_TABLE.csv")
GAMMA_CMD = 75.0
FIN = 0.0
DT = 0.001
MAX_TIME = 100.0
DISPLAY_FPS = 30.0
CHASE_WIDTH = 400.0
CHASE_HEIGHT = 400.0


def interpolate_t_burst(gamma_cmd, table_path=TABLE_PATH):
    table = pd.read_csv(table_path).dropna(subset=["gamma", "t_burst"])
    if table.empty:
        raise ValueError("initial-turn state table contains no valid gamma/t_burst rows")

    table = table.sort_values("gamma")
    gamma_values = table["gamma"].to_numpy(dtype=float)
    t_burst_values = table["t_burst"].to_numpy(dtype=float)
    t_burst = float(np.interp(gamma_cmd, gamma_values, t_burst_values))

    if gamma_cmd < gamma_values[0] or gamma_cmd > gamma_values[-1]:
        print(
            f"gamma_cmd={gamma_cmd:.1f} is outside the table range "
            f"[{gamma_values[0]:.1f}, {gamma_values[-1]:.1f}]; "
            "using the nearest table endpoint."
        )

    return t_burst


def simulate_missile(t_burst, fin=FIN, dt=DT, max_time=MAX_TIME):
    missile = Missile(dt=dt, t_burst=t_burst, fin=fin)
    trajectory = [missile_initial_state()]
    max_steps = int(max_time / dt)

    for _ in range(max_steps):
        state = missile.step()
        trajectory.append(state.copy())
        if state["z"] <= 0.0 and missile.t > dt:
            break

    return trajectory


def animate_missile(trajectory, t_burst, fin):
    x_values = np.array([state["x"] for state in trajectory])
    z_values = np.array([state["z"] for state in trajectory])
    frame_step = max(1, round(1.0 / (DISPLAY_FPS * DT)))
    frame_indices = np.arange(0, len(trajectory), frame_step)
    if frame_indices[-1] != len(trajectory) - 1:
        frame_indices = np.append(frame_indices, len(trajectory) - 1)
    padding = max(50.0, 0.05 * max(np.ptp(x_values), np.ptp(z_values), 1.0))

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    full_axis, chase_axis = axes
    full_axis.set_title("Full trajectory")
    chase_axis.set_title("Chase camera")
    for axis in axes:
        axis.set_xlabel("x (m)")
        axis.set_ylabel("z (m)")
        axis.axhline(0.0, color="black", linewidth=1.0)
        axis.grid(True)

    full_axis.set_xlim(x_values.min() - padding, x_values.max() + padding)
    full_axis.set_ylim(min(z_values.min() - padding, -padding), z_values.max() + padding)
    full_trail, = full_axis.plot([], [], "b-")
    full_marker, = full_axis.plot([], [], "bo")
    chase_trail, = chase_axis.plot([], [], "b-")
    chase_marker, = chase_axis.plot([], [], "bo")

    def update(frame):
        sample = frame_indices[frame]
        full_trail.set_data(x_values[:sample + 1], z_values[:sample + 1])
        full_marker.set_data([x_values[sample]], [z_values[sample]])

        chase_x = x_values[sample]
        chase_z = z_values[sample]
        chase_axis.set_xlim(
            chase_x - CHASE_WIDTH / 2.0,
            chase_x + CHASE_WIDTH / 2.0,
        )
        chase_axis.set_ylim(
            chase_z - CHASE_HEIGHT / 2.0,
            chase_z + CHASE_HEIGHT / 2.0,
        )
        chase_trail.set_data(x_values[:sample + 1], z_values[:sample + 1])
        chase_marker.set_data([chase_x], [chase_z])
        return full_trail, full_marker, chase_trail, chase_marker

    animation = FuncAnimation(
        figure,
        update,
        frames=len(frame_indices),
        interval=1000.0 / DISPLAY_FPS,
        blit=True,
        repeat=False,
    )
    return figure, animation


def plot_full_trajectory(trajectory, t_burst, fin):
    x_values = np.array([state["x"] for state in trajectory])
    z_values = np.array([state["z"] for state in trajectory])
    padding = max(50.0, 0.05 * max(np.ptp(x_values), np.ptp(z_values), 1.0))

    figure, axis = plt.subplots()
    axis.plot(x_values, z_values, "b-")
    axis.plot(x_values[0], z_values[0], "go", label="Launch")
    axis.plot(x_values[-1], z_values[-1], "rx", markersize=8, label="Ground impact")
    axis.axhline(0.0, color="black", linewidth=1.0)
    axis.set_title(f"Complete missile trajectory: t_burst={t_burst:.4f} s, fin={fin:.1f}")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("z (m)")
    axis.set_xlim(x_values.min() - padding, x_values.max() + padding)
    axis.set_ylim(min(z_values.min() - padding, -padding), z_values.max() + padding)
    axis.grid(True)
    axis.legend()
    return figure


def plot_state_history(trajectory, dt):
    time_values = np.arange(len(trajectory)) * dt
    velocity = np.array([state["V"] for state in trajectory])
    alpha = np.rad2deg([state["alpha"] for state in trajectory])
    theta = np.rad2deg([state["theta"] for state in trajectory])
    gamma = theta - alpha

    figure, axes = plt.subplots(2, 2, sharex=True, figsize=(11, 7))
    histories = [
        (axes[0, 0], velocity, "V (m/s)", "tab:blue"),
        (axes[0, 1], gamma, "gamma (deg)", "tab:orange"),
        (axes[1, 0], alpha, "alpha (deg)", "tab:green"),
        (axes[1, 1], theta, "theta (deg)", "tab:red"),
    ]

    for axis, values, ylabel, color in histories:
        axis.plot(time_values, values, color=color)
        axis.set_ylabel(ylabel)
        axis.grid(True)

    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_xlabel("time (s)")
    figure.suptitle("Missile state history")
    figure.tight_layout()
    return figure


def main():
    t_burst = interpolate_t_burst(GAMMA_CMD)
    trajectory = simulate_missile(t_burst=t_burst, fin=FIN)
    print(f"gamma_cmd: {GAMMA_CMD:.1f} deg")
    print(f"t_burst: {t_burst:.6f} s")
    print(f"fin: {FIN:.1f}")
    print(f"ground impact: {trajectory[-1]['x']:.2f} m, t={len(trajectory) * DT:.2f} s")

    trajectory_figure = plot_full_trajectory(trajectory, t_burst, FIN)
    state_figure = plot_state_history(trajectory, DT)
    animation_figure, animation = animate_missile(trajectory, t_burst, FIN)
    plt.show()


if __name__ == "__main__":
    main()