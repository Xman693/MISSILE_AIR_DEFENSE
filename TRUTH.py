import numpy as np


class Truth:
    def __init__(self, dt_sim, true_target_state_prev):
        self.dt_sim = dt_sim
        self.true_target_state_prev = np.asarray(true_target_state_prev, dtype=float)

    def get_true_target(self):
        # state = [x, y, vx, vy, ax, ay]
        def dynamics(state):
            pos = state[:2]
            vel = state[2:4]
            acc = state[4:6]
            return np.concatenate([vel, acc, np.zeros(2)])

        def rk4_step(f, y, dt):
            k1 = f(y)
            k2 = f(y + 0.5 * dt * k1)
            k3 = f(y + 0.5 * dt * k2)
            k4 = f(y + dt * k3)
            return y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        self.true_target_state_prev = rk4_step(dynamics, self.true_target_state_prev, self.dt_sim)
        return self.true_target_state_prev