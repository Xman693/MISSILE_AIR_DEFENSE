import numpy as np
import MISSILE_MODEL


def state_lookup(gamma_guess, lookup_table):
    """Interpolate the initial-turn state for a requested flight-path angle."""
    required_columns = ["gamma", "t_burst", "V", "alpha", "q", "theta", "x", "z"]
    missing_columns = [column for column in required_columns if column not in lookup_table.columns]
    if missing_columns:
        raise ValueError(f"lookup table is missing columns: {missing_columns}")

    valid_table = lookup_table[required_columns].dropna().sort_values("gamma")
    if valid_table.empty:
        raise ValueError("lookup table contains no finite states")

    gamma_values = valid_table["gamma"].to_numpy(dtype=float)
    gamma_guess = float(gamma_guess)
    interpolated_state = tuple(
        np.interp(gamma_guess, gamma_values, valid_table[column].to_numpy(dtype=float))
        for column in required_columns[1:]
    )

    return interpolated_state


def propagate_missile(initial_turn_state, intercept_point, dt=0.01, t_max=100.0, fin=0.0):
    """Propagate the missile through POCA relative to an intercept point.

    ``initial_turn_state`` is ordered as returned by ``state_lookup`` and its
    alpha and theta values are degrees, matching the offline lookup table.
    The returned columns are V, q, alpha, gamma, x, and z.
    """
    if len(initial_turn_state) != 7:
        raise ValueError("initial_turn_state must contain t_burst, V, alpha, q, theta, x, and z")

    intercept_point = np.asarray(intercept_point, dtype=float)
    if intercept_point.shape != (2,):
        raise ValueError("intercept_point must contain x and z")
    if dt <= 0 or t_max < 0:
        raise ValueError("dt must be positive and t_max must not be negative")

    t_burst, velocity, alpha, pitch_rate, theta, x_position, z_position = initial_turn_state
    state = {
        "V": float(velocity),
        "alpha": np.deg2rad(float(alpha)),
        "q": float(pitch_rate),
        "theta": np.deg2rad(float(theta)),
        "x": float(x_position),
        "z": float(z_position),
    }
    missile = MISSILE_MODEL.Missile(dt, float(t_burst), fin)
    missile.state = state

    trajectory = []
    previous_distance_squared = None
    num_steps = int(np.floor(t_max / dt))

    for _ in range(num_steps + 1):
        gamma = missile.state["theta"] - missile.state["alpha"]
        trajectory.append([
            missile.state["V"],
            missile.state["q"],
            missile.state["alpha"],
            gamma,
            missile.state["x"],
            missile.state["z"],
        ])

        dx = missile.state["x"] - intercept_point[0]
        dz = missile.state["z"] - intercept_point[1]
        distance_squared = dx * dx + dz * dz
        if (previous_distance_squared is not None
                and distance_squared > previous_distance_squared):
            trajectory.pop()
            break
        previous_distance_squared = distance_squared
        missile.step()

    return np.asarray(trajectory, dtype=float)


def propogate_missile(initial_turn_state, intercept_point, dt=0.01, t_max=100.0, fin=0.0):
    return propagate_missile(initial_turn_state, intercept_point, dt, t_max, fin)


def compute_cost(trajectory):
    """Return the squared norm of the trajectory's POCA position."""
    trajectory = np.asarray(trajectory, dtype=float)
    if trajectory.ndim != 2 or trajectory.shape[0] == 0 or trajectory.shape[1] < 6:
        raise ValueError("trajectory must contain at least one row with V, q, alpha, gamma, x, and z")

    poca = trajectory[-1, 4:6]
    return np.linalg.norm(poca) ** 2


class FireControl:
    def __init__(self, P_initial, Q, R):
        self.P_initial = np.asarray(P_initial, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.P = self.P_initial.copy()
        self.state_estimate_NED = None
        self.first_iteration = True

    @staticmethod
    def _rotation(nominal_radar_pitch_angle, beam_angle):
        phi = nominal_radar_pitch_angle + beam_angle
        return np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])

    def _ned_to_radar(self, state_NED, nominal_radar_pitch_angle, beam_angle):
        rotation = self._rotation(nominal_radar_pitch_angle, beam_angle)
        return np.concatenate((rotation @ state_NED[:2], rotation @ state_NED[2:4]))

    def _radar_to_ned(self, state_radar, nominal_radar_pitch_angle, beam_angle):
        rotation = self._rotation(nominal_radar_pitch_angle, beam_angle)
        return np.concatenate((rotation.T @ state_radar[:2], rotation.T @ state_radar[2:4]))

    @staticmethod
    def _measurement_model(state_radar):
        px, pz, vx, vz = state_radar
        range_value = max(np.hypot(px, pz), 1e-9)
        radial_velocity = (px * vx + pz * vz) / range_value
        predicted_measurement = np.array([-np.arctan2(pz, px), range_value, radial_velocity])

        denominator = max(px**2 + pz**2, 1e-12)
        H = np.zeros((3, 4))
        H[0, 0] = pz / denominator
        H[0, 1] = -px / denominator
        H[1, 0] = px / range_value
        H[1, 1] = pz / range_value
        H[2, 0] = (vx * range_value - radial_velocity * px) / denominator
        H[2, 1] = (vz * range_value - radial_velocity * pz) / denominator
        H[2, 2] = px / range_value
        H[2, 3] = pz / range_value
        return predicted_measurement, H

    def estimate_target_state(self, noisy_radar_measurement, nominal_radar_pitch_angle,
                              dt, beam_angle):
        
        
        # check this logic later (aka when no measuremnt is aval does KF keep propogating??? It should)
        if noisy_radar_measurement is None:
            return self.state_estimate_NED, self.P.copy()

        measurement = np.asarray(noisy_radar_measurement, dtype=float)
        if measurement.shape != (3,):
            raise ValueError("noisy_radar_measurement must have shape (3,)")
        if dt <= 0:
            raise ValueError("dt must be positive")

        if self.first_iteration or self.state_estimate_NED is None:
            los, range_value, range_rate = measurement
            state_radar = np.array([
                range_value * np.cos(los),
                -range_value * np.sin(los),
                range_rate * np.cos(los),
                -range_rate * np.sin(los),
            ])
            self.P = self.P_initial.copy()
            self.first_iteration = False
        else:
            state_radar = self._ned_to_radar(
                self.state_estimate_NED, nominal_radar_pitch_angle, beam_angle)

        transition = np.array([[1.0, 0.0, dt, 0.0],
                               [0.0, 1.0, 0.0, dt],
                               [0.0, 0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]])
        state_prior = transition @ state_radar
        covariance_prior = transition @ self.P @ transition.T + self.Q

        predicted_measurement, H = self._measurement_model(state_prior)
        innovation = measurement - predicted_measurement
        innovation[0] = (innovation[0] + np.pi) % (2.0 * np.pi) - np.pi
        innovation_covariance = H @ covariance_prior @ H.T + self.R
        kalman_gain = covariance_prior @ H.T @ np.linalg.inv(innovation_covariance)
        state_post = state_prior + kalman_gain @ innovation

        identity = np.eye(4)
        self.P = (identity - kalman_gain @ H) @ covariance_prior
        self.P = 0.5 * (self.P + self.P.T)
        self.state_estimate_NED = self._radar_to_ned(
            state_post, nominal_radar_pitch_angle, beam_angle)
        return self.state_estimate_NED.copy(), self.P.copy()

def target_state_extrapolation(state_estimate_NED, dt, t_max, x_launcher):
    if state_estimate_NED is None:
        return None


    state = np.asarray(state_estimate_NED, dtype=float)
    if state.shape != (4,):
        raise ValueError("state_estimate_NED must have shape (4,) as [px, pz, vx, vz]")

    num_steps = int(np.floor(t_max / dt))
    trajectory = np.empty((num_steps + 1, 4), dtype=float)
    trajectory[0] = state

    if state[0] < x_launcher:
        return 0.0, trajectory[:1], 0

    def dynamics(current_state):
        px, pz, vx, vz = current_state
        return np.array([vx, vz, 0.0, 0.0])

    for step in range(num_steps):
        k1 = dynamics(state)
        k2 = dynamics(state + 0.5 * dt * k1)
        k3 = dynamics(state + 0.5 * dt * k2)
        k4 = dynamics(state + dt * k3)
        state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        trajectory[step + 1] = state

        t_extrap = (step + 1) * dt
        if state[0] < x_launcher:
            return t_extrap, trajectory[:step + 2], step + 1

    return t_max, trajectory, num_steps

def intercept_point(t_max, trajectory, step):
    
    t_intercept = t_max - 50
    
    n_at_50_sec = int(t_intercept/t_max)*step
    
    intercept_point = trajectory[n_at_50_sec, 0:2] # extract the target position at 50 seconds
    
    return intercept_point

def traj_optimizer(self, intercept_point, gamma_guess):
    tol = 1e-2
    delta_gamma = np.deg2rad(1)
    max_iter = 10

    iter_count = 0
    gamma_update = float("inf")
    while abs(gamma_update) > tol and iter_count < max_iter:
        state = self.state_lookup(gamma_guess)
        state_plus = self.state_lookup(gamma_guess + delta_gamma)
        state_minus = self.state_lookup(gamma_guess - delta_gamma)

        trajectory = propogate_missile(state, intercept_point)
        trajectory_plus = propogate_missile(state_plus, intercept_point)
        trajectory_minus = propogate_missile(state_minus, intercept_point)

        cost = compute_cost(trajectory)
        cost_plus = compute_cost(trajectory_plus)
        cost_minus = compute_cost(trajectory_minus)

        J_d1 = (cost_plus - cost_minus) / (2 * delta_gamma)
        J_d2 = (cost_plus - 2 * cost + cost_minus) / (delta_gamma ** 2)

        gamma_update = -J_d1 / J_d2 if J_d2 != 0 else 0.0
        gamma_guess += gamma_update
        iter_count += 1

    return gamma_guess


def get_target_at_launch(intercept_point, target_state, tof):
    
    target_at_launch = intercept_point - target_state[:2] * tof
    return target_at_launch

def launch_true(target_at_launch, target_state, P):
    
    Px = P[0, 0]
    Pz = P[1, 1]
    
   
    
    if np.linalg.norm(target_state[0:2] + np.array([Px, Pz]) - target_at_launch) < 5:
        return True
    else:
        return False
    
    
    
def pre_launch_guidance(intercept_point, target_state, P):
    
    
    #----- function calls (in order) -------
    
    # estimate state 
    # rotate state to NED
    # extrapolate state
    # pick intercept point to be 50 sec out from launcher 
    # find optimal initial waypoint
    # using tof from above, find target position at launch
    # using target at launch, current target state estimate and covar determine if launch cmd should be given
    
    
    # -- return -- 
    # pre launch guidance should return launch decision, initial turn angle, burst time, and time of flight, pip
    
    
    
    gamma_guess = np.deg2rad(45)
    gamma_initial_turn, t_burst, tof = traj_optimizer(
        intercept_point=intercept_point,
        gamma_guess=gamma_guess,
    )

    target_at_launch = get_target_at_launch(intercept_point, target_state, tof)
    launch_decision = launch_true(target_at_launch, target_state, P)

    return launch_decision, gamma_initial_turn, t_burst, tof

def midcourse_guidance():
    
# ---- function calls (in order) ---------

# estimate target state 
# rotate state to NED
# call traj optimizer
# return gamma_cmd, tof, pip

    pass
    


   


    



