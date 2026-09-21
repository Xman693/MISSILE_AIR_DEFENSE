import numpy as np

def get_simulation_parameters():
    dt = 0.0005
    t_max = 100
    iter = int(t_max / dt)
    sim_params = { 
        "dt": dt,
        "t_max": t_max,
        "iter": iter
        
    }
    return sim_params
    
    
def get_radar_parameters():
    update_freq = 20  # radar updates 20 times per second
    measurement_interval = 1.0 / update_freq  # time interval between radar measurements
    max_range = 100000.0  # maximum radar range
    fov_max = np.deg2rad(75.0)
    beam_width = np.deg2rad(5.0)  # radar beam width
    beamwidth = np.deg2rad(5.0)
    nominal_radar_pitch_angle = np.deg2rad(30)
    true_radar_pitch_angle = np.deg2rad(30)
    gimble_change_interval = 1/1000
    dwell_time = measurement_interval

    radar_params = {
        "update_freq": update_freq,
        "measurement_interval": measurement_interval,
        "max_range": max_range,
        "fov_max": fov_max,
        "beam_width": beam_width,
        "beamwidth": beamwidth,
        "radar_measurement_noise": np.array([np.deg2rad(5), 50.0, 20.0]),
        "gimble_change_interval": gimble_change_interval,
        "nominal_radar_pitch_angle": nominal_radar_pitch_angle,
        "true_radar_pitch_angle": true_radar_pitch_angle,
        "dwell_time": dwell_time,
    }
    return radar_params
    
def get_missile_parameters():
    missile_parameters = {
        "m": 100.0,             # mass, kg
        "S": 0.05,              # reference area, m^2
        "c": 0.3,               # reference length, m
        "Iy": 200,             # pitch moment of inertia, kg*m^2
        "l_tvc": 1.0,           # TVC nozzle moment arm from CG, m
        "Tmax": 15000.0,        # max thrust, N
        "thrust_ramp": 1.0,     # thrust ramp time constant, s
        "thrust_hold": 5.0,     # thrust hold time, s
        "tvc_force": 1000.0,    # default TVC pulse magnitude, N
    }
    return missile_parameters

def get_missile_aero():
    # nonlinear CL/CD/Cm expansions:
    #   CL = CL_max*tanh(CLalpha*alpha/CL_max) + CLdelta*delta
    #   CD = CD0 + CDalpha2*alpha^2 + CDalpha4*alpha^4 + CDdelta2*delta^2
    #   Cm = Cm0 + Cmalpha*alpha + Cmalpha3*alpha^3 + Cmq*(q*c/2V) + Cmdelta*delta
    missile_aero = {
        "CL_max": 1.5,       # lift coefficient saturation magnitude
        "CLalpha": 2.0,      # lift coefficient slope wrt alpha at alpha=0, per rad
        "CLdelta": 0.5,      # lift coefficient slope wrt fin deflection, per rad
        "CD0": 0.3,          # zero-alpha drag coefficient
        "CDalpha2": 2.5,     # drag coefficient slope wrt alpha^2
        "CDalpha4": 5.0,     # drag coefficient slope wrt alpha^4
        "CDdelta2": 0.3,     # drag coefficient slope wrt delta^2
        "Cm0": 0.0,          # zero-alpha pitching moment coefficient
        "Cmalpha": -8.0,     # pitching moment coefficient slope wrt alpha (statically stable)
        "Cmalpha3": -15.0,   # pitching moment coefficient slope wrt alpha^3
        "Cmq": -80.0,        # pitch damping moment coefficient
        "Cmdelta": -0.5,     # pitching moment coefficient slope wrt fin deflection
    }
    return missile_aero

def get_environment_parameters():
    environment_parameters = {
        "rho0": 1.225,           # sea-level air density, kg/m^3
        "scale_height": 8500.0,  # atmospheric scale height, m
        "g": 9.81,               # gravitational acceleration, m/s^2
    }
    return environment_parameters
    
def get_target_true_parameters():
    pos = np.array([80000.0, -50000])   # target starts at 100 km, 10 km down/up RELATVIE TO RADAR
    vel = np.array([-700, 0])
    acc = np.array([0.0, 0.0])
    return np.concatenate([pos, vel, acc])

def get_covariance_matrices():
    P = np.diag([100000.0**2, 100000.0**2, 1000.0**2, 1000.0**2])
    Q = np.zeros((4, 4))
    R = np.diag([np.deg2rad(0.5)**2, 5.0**2, 2.0**2])
    
    covar_matrices = {
        "P": P,
        "Q": Q,
        "R": R
    }
    
    return covar_matrices

def get_fc_params():
    fc_params = {
        "processing_freq": 100,
        "processing_interval": 1/fc_params["processing_freq"],
      
    }
    return fc_params