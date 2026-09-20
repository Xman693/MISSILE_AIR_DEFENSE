import numpy as np
import PARAMATERS

# FOR NOW, MISSILE MODEL IS TRUTH (LATER ADD MC)

missile_parameters = PARAMATERS.get_missile_parameters()
missile_aero = PARAMATERS.get_missile_aero()
environment_parameters = PARAMATERS.get_environment_parameters()

class Missile:
    def __init__(self, dt, t_burst, fin):
        self.dt = dt
        self.t_burst = t_burst      # TVC burst duration, s
        self.fin = fin              # fin deflection command, rad
        self.t = 0.0
        self.state = missile_initial_state()
        self.missile_parameters = missile_parameters
        self.missile_aero = missile_aero

    def step(self):
        self.state = missile_truth_propagation(False, self.state, self.t, self.dt, self.t_burst, self.fin)
        self.t += self.dt
        return self.state


def missile_initial_state():
    # Define the initial state of the missile
    state = {
        "V": 50.0,        # initial velocity, m/s
        "alpha": 0.0,    # initial angle of attack, rad
        "q": 0.0,        # initial pitch rate, rad/s
        "theta": np.deg2rad(30.0),    # initial pitch angle, rad
        "x": 1000,        # initial x position, m # launcher position rel to RADAR NED
        "z": 0.0         # initial z position, m
    }
    return state


def air_density(z):
    """Exponential atmosphere approximation, kg/m^3"""
    rho0 = environment_parameters["rho0"]
    scale_height = environment_parameters["scale_height"]
    return rho0 * np.exp(-z / scale_height)


def thrust_profile(t):
    Tmax = missile_parameters["Tmax"]
    thrust_ramp = missile_parameters["thrust_ramp"]
    thrust_hold = missile_parameters["thrust_hold"]

    if t < thrust_ramp:
        return Tmax * (t / thrust_ramp)
    elif t < (thrust_ramp + thrust_hold):
        return Tmax
    else:
        t_end_hold = thrust_ramp + thrust_hold
        if t < (t_end_hold + thrust_ramp):
            return Tmax * (1 - (t - t_end_hold) / thrust_ramp)
        else:
            return 0.0


def tvc_profile(t, t_burst):
    """Fixed-magnitude TVC pulse right off the rail, then off."""
    tvc_force = missile_parameters["tvc_force"]
    return tvc_force if t < t_burst else 0.0


def missile_coefficent_calculator(state, fin):
    aero = missile_aero
    c = missile_parameters["c"]

    V, alpha, q = state["V"], state["alpha"], state["q"]
    delta = fin

    CL = aero["CL_max"] * np.tanh(aero["CLalpha"] * alpha / aero["CL_max"]) + aero["CLdelta"] * delta
    CD = aero["CD0"] + aero["CDalpha2"] * alpha**2 + aero["CDalpha4"] * alpha**4 + aero["CDdelta2"] * delta**2
    Cm = aero["Cm0"] + aero["Cmalpha"] * alpha + aero["Cmalpha3"] * alpha**3 \
        + aero["Cmq"] * (q * c / (2 * V)) + aero["Cmdelta"] * delta

    return CL, CD, Cm


def missile_true_model(state, t, t_burst, fin):
    g = environment_parameters["g"]
    m = missile_parameters["m"]
    S = missile_parameters["S"]
    c = missile_parameters["c"]
    Iy = missile_parameters["Iy"]
    l_tvc = missile_parameters["l_tvc"]

    V, alpha, q, theta, z = state["V"], state["alpha"], state["q"], state["theta"], state["z"]
    gamma = theta - alpha
    rho = air_density(z)
    T = thrust_profile(t)
    F_tvc = tvc_profile(t, t_burst)

    qbar = 0.5 * rho * V**2
    CL, CD, Cm = missile_coefficent_calculator(state, fin)
    L = qbar * S * CL
    D = qbar * S * CD
    M = qbar * S * c * Cm

    Vdot = (T * np.cos(alpha) - F_tvc * np.sin(alpha) - D) / m - g * np.sin(theta - alpha)

    alphadot = q - (L + T * np.sin(alpha) + F_tvc * np.cos(alpha)
                    - m * g * np.cos(theta - alpha)) / (m * V)

    qdot = (M + l_tvc * F_tvc) / Iy

    thetadot = q
    xdot = V * np.cos(gamma)
    zdot = V * np.sin(gamma)

    state_dot = {
        "Vdot": Vdot,
        "alphadot": alphadot,
        "qdot": qdot,
        "thetadot": thetadot,
        "xdot": xdot,
        "zdot": zdot
    }
    return state_dot


def missile_truth_propagation(initial_state_flag, state, t, dt, t_burst, fin):

    if initial_state_flag:
        state = missile_initial_state()

    state_dot = missile_true_model(state, t, t_burst, fin)

    new_state = {
        "V": state["V"] + state_dot["Vdot"] * dt,
        "alpha": state["alpha"] + state_dot["alphadot"] * dt,
        "q": state["q"] + state_dot["qdot"] * dt,
        "theta": state["theta"] + state_dot["thetadot"] * dt,
        "x": state["x"] + state_dot["xdot"] * dt,
        "z": state["z"] + state_dot["zdot"] * dt,
    }
    return new_state
