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
    measurement_noise_std = np.array([np.deg2rad(2.0), 100, 20])
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
        "gimble_change_interval": gimble_change_interval,
        "measurement_noise_std": measurement_noise_std,
        "nominal_radar_pitch_angle": nominal_radar_pitch_angle,
        "true_radar_pitch_angle": true_radar_pitch_angle,
        "dwell_time": dwell_time,
    }
    return radar_params
    
def get_missile_parameters():
    pass
    
def get_aero_parameters():
    pass
    
def get_environment_parameters():
    pass
    
def get_target_true_parameters():
    pos = np.array([80000.0, -50000])   # target starts at 100 km, 10 km down/up RELATVIE TO RADAR
    vel = np.array([-700, 0])
    acc = np.array([0.0, 0.0])
    return np.concatenate([pos, vel, acc])