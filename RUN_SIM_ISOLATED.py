from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd

from FireControl import FireControl, propagate_missile, state_lookup
from PARAMATERS import get_covariance_matrices


TABLE_PATH = Path(__file__).with_name("INITIAL_TURN_STATE_TABLE.csv")
INTERCEPT_POINT = np.array([80000.0, -50000.0])
PROPAGATION_DT = 0.01
PROPAGATION_T_MAX = 100.0
INITIAL_GAMMA_SAMPLES = 101
REFINEMENT_GAMMA_SAMPLES = 51
REFINEMENT_HALF_WIDTH_DEG = 2.0


def load_state_table(table_path=TABLE_PATH):
    return pd.read_csv(table_path)


def trajectory_cost(trajectory, intercept_point):
    """Return squared miss distance at the trajectory's POCA sample."""
    poca = trajectory[-1, 4:6]
    return np.linalg.norm(poca - intercept_point) ** 2


def find_optimal_state(fire_control, state_table, intercept_point):
    """Find gamma and its table-derived t_burst using a coarse scan and refinement."""
    valid_gamma = state_table["gamma"].dropna().to_numpy(dtype=float)
    if valid_gamma.size == 0:
        raise ValueError("state table contains no valid gamma values")

    gamma_min = float(np.min(valid_gamma))
    gamma_max = float(np.max(valid_gamma))

    def evaluate(gamma):
        initial_turn_state = state_lookup(gamma, state_table)
        trajectory = propagate_missile(
            initial_turn_state,
            intercept_point,
            dt=PROPAGATION_DT,
            t_max=PROPAGATION_T_MAX,
        )
        return trajectory_cost(trajectory, intercept_point), initial_turn_state, trajectory

    coarse_gammas = np.linspace(gamma_min, gamma_max, INITIAL_GAMMA_SAMPLES)
    coarse_results = [evaluate(gamma) for gamma in coarse_gammas]
    best_index = int(np.argmin([result[0] for result in coarse_results]))
    best_gamma = float(coarse_gammas[best_index])

    refine_min = max(gamma_min, best_gamma - REFINEMENT_HALF_WIDTH_DEG)
    refine_max = min(gamma_max, best_gamma + REFINEMENT_HALF_WIDTH_DEG)
    refined_gammas = np.linspace(refine_min, refine_max, REFINEMENT_GAMMA_SAMPLES)
    refined_results = [evaluate(gamma) for gamma in refined_gammas]
    refined_index = int(np.argmin([result[0] for result in refined_results]))
    optimal_gamma = float(refined_gammas[refined_index])
    cost, initial_turn_state, trajectory = refined_results[refined_index]

    t_burst = float(initial_turn_state[0])
    return fire_control, optimal_gamma, t_burst, cost, trajectory


def animate_trajectory(trajectory, intercept_point):
    """Animate the missile trajectory and mark the intercept point with an x."""
    positions = trajectory[:, 4:6]
    all_x = np.append(positions[:, 0], intercept_point[0])
    all_z = np.append(positions[:, 1], intercept_point[1])
    padding = max(100.0, 0.05 * max(np.ptp(all_x), np.ptp(all_z), 1.0))

    figure, axis = plt.subplots()
    axis.set_title("Isolated Missile Model Trajectory")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("z (m)")
    axis.set_xlim(np.min(all_x) - padding, np.max(all_x) + padding)
    axis.set_ylim(np.min(all_z) - padding, np.max(all_z) + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True)

    axis.plot(intercept_point[0], intercept_point[1], "x", markersize=12,
              markeredgewidth=2, label="Intercept point")
    trail, = axis.plot([], [], "b-", label="Missile trajectory")
    missile, = axis.plot([], [], "bo")
    axis.legend()

    def update(frame):
        trail.set_data(positions[:frame + 1, 0], positions[:frame + 1, 1])
        missile.set_data([positions[frame, 0]], [positions[frame, 1]])
        return trail, missile

    animation = FuncAnimation(
        figure,
        update,
        frames=len(positions),
        interval=PROPAGATION_DT * 1000,
        blit=True,
        repeat=False,
    )
    return figure, animation


def main():
    state_table = load_state_table()
    covariance_matrices = get_covariance_matrices()
    fire_control = FireControl(
        P_initial=covariance_matrices["P"],
        Q=covariance_matrices["Q"],
        R=covariance_matrices["R"],
    )

    _, optimal_gamma, t_burst, cost, trajectory = find_optimal_state(
        fire_control,
        state_table,
        INTERCEPT_POINT,
    )
    print(f"optimal gamma: {optimal_gamma:.6f} deg")
    print(f"t_burst: {t_burst:.6f} s")
    print(f"POCA cost: {cost:.6f} m^2")

    animate_trajectory(trajectory, INTERCEPT_POINT)
    plt.show()


if __name__ == "__main__":
    main()
