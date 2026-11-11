# goal is test some signal processing algorithms to for use in the full sim. 

import numpy as np 
from Paramater import RadarParams, SimParams, TargetParams
from Helpers import coordinate_transformation
import matplotlib.pyplot as plt

radar = RadarParams() 
sim = SimParams()
target = TargetParams()


# load true target trajectory from CSV (columns: x, z, vx, vz)
# load target trajectory from CSV (columns: x, z, vx, vz)
target_traj = np.loadtxt("target_traj.csv", delimiter=",", skiprows=1) # N x 4
num_steps = target_traj.shape[0]

RCS = target.RCS


# P  = P_t * G^2 * lambda*2 * sigma / ((4 * pi)^3 * R^4)


freq = radar.carrier_freq
c = 3e8
lamdba = c / freq
Pt = radar.peak_power
G_max = radar.antenna_efficiency * 4 * np.pi * radar.aperature_area / lamdba**2
tau = 1 / radar.PRF * radar.Duty_cycle / 100  # pulse duration 
radar_pitch = radar.pitch_angle
radar_beamwidth = radar.radar_beamwidth
R = radar.antenna_resistance



def get_radar_measurement(): 
    
    if 




