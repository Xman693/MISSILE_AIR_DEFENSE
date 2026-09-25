import numpy as np

from FireControl import FireControl
from PARAMATERS import get_covariance_matrices, get_simulation_parameters
from PARAMATERS import get_target_true_parameters
from TARGET_TRUTH import Truth


def _true_target_measurement(target_state):
    position = target_state[:2]
    velocity = target_state[2:4]
    range_value = np.linalg.norm(position)
    radial_velocity = np.dot(position, velocity) / max(range_value, 1e-9)
    line_of_sight = -np.arctan2(position[1], position[0])
    return np.array([line_of_sight, range_value, radial_velocity])


def Run_Simulation(sim_params, true_target_state, radar_params=None):
    truth = Truth(
        dt_sim=sim_params["dt"],
        target_state=true_target_state,
        t_max=sim_params["t_max"],
    )
    target_trajectory = truth.get_target_trajectory()

    covariance_matrices = get_covariance_matrices()
    fire_control = FireControl(
        P_initial=covariance_matrices["P"],
        Q=covariance_matrices["Q"],
        R=covariance_matrices["R"],
    )

    estimated_target_trajectory = []
    for target_state in target_trajectory[:sim_params["iter"]]:
        measurement = _true_target_measurement(target_state)
        estimated_state, _ = fire_control.estimate_target_state(
            noisy_radar_measurement=measurement,
            nominal_radar_pitch_angle=0.0,
            dt=sim_params["dt"],
            beam_angle=0.0,
        )
        estimated_target_trajectory.append(estimated_state)

    return np.asarray(target_trajectory[:sim_params["iter"]]), np.asarray(
        estimated_target_trajectory
    )


if __name__ == "__main__":
    simulation_parameters = get_simulation_parameters()
    target_state = get_target_true_parameters()
    Run_Simulation(simulation_parameters, target_state)
