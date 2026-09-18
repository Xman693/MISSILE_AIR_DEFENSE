import numpy as np
import pandas as pd
import MISSILE_TRUTH


dt = 0.01 # time step for simulation
dt_burst = 0.01 # time difference for table 
t_max = 1.5 # time duration for intial turn to be completed
dt_burst_max = 1

t_burst_values = np.arange(0, dt_burst_max, dt_burst)
n_steps = int(round(t_max / dt))
state_table = np.zeros((len(t_burst_values), 8))

for row_counter, t_burst in enumerate(t_burst_values):

    missile = None
    blew_up = False
    for idx in range(n_steps):
        t = idx * dt
        if idx == 0:
            missile = MISSILE_TRUTH.missile_truth_propagation(True, None, t, dt, t_burst, 0)
        else:
            missile = MISSILE_TRUTH.missile_truth_propagation(False, missile, t, dt, t_burst, 0)

        if any(np.isnan(v) for v in missile.values()):
            blew_up = True
            break

    if blew_up:
        state_table[row_counter, :] = np.nan
        continue

    # calculate gamma,
    gamma = np.rad2deg(missile["theta"] - missile["alpha"])  # assuming gamma is the pitch angle in degrees

    state_table[row_counter, :] = [t_burst, gamma, np.rad2deg(missile["theta"]), np.rad2deg(missile["alpha"]), missile["q"], missile["V"], missile["x"], missile["z"]]

state_table = pd.DataFrame(state_table, columns=["t_burst", "gamma", "theta", "alpha", "q", "V", "x", "z"])
print(state_table)

state_table.to_csv("INITIAL_TURN_STATE_TABLE.csv", index=False)

    