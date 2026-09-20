import numpy as np


class Truth:
    def __init__(self, dt_sim, target_state, t_max):
        self.dt_sim = dt_sim
        self.target_state= np.asarray(target_state, dtype=float)
        self.t_max = t_max
        

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

        self.target_state = rk4_step(dynamics, self.target_state, self.dt_sim)
        return self.target_state
    
    def get_target_trajectory(self):
        target_trajectory = []
        for i in range(0, int(self.t_max / self.dt_sim)):
            if i == 0:
                target_trajectory.append(self.target_state)
            else:
                self.get_true_target()
                target_trajectory.append(self.target_state)
        return target_trajectory
    
    def get_target_at_time(self, time, target_trajectory):
        target_at_time = target_trajectory[int(time / self.dt_sim)]
        return target_at_time