
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ThrustParams:
    T: float = 15000
    Thold: float = 15
    Tramp: float = 0.1


@dataclass(frozen=True)
class AeroParams:
    cl0: float = 0.0
    cl_alpha: float = 0.0
    cd0: float = 0.1
    cd_alpha: float = 0.05  
    
@dataclass(frozen=True)
class VehicleParams:
    m: float = 100
    d: float = 0.25
    S: float = np.pi * (0.25 ** 2)/4
    
@dataclass(frozen=True)
class LauncherParams:
    launcher_emplacement_x: float = 1000
    launcher_emplacement_z: float = 0
    launcher_angle: float = 45
    
        
@dataclass(frozen=True)
class SimParams:
    sim_freq: float = 100
    t_max: float = 300
    dt_sim: float = 1 / sim_freq
    num_steps: int = int(t_max / (1 / sim_freq))
    
@dataclass(frozen=True)
class AssetParams:
    x: float = 20000
    z: float = 0.0
    radius: float = 1000
   
@dataclass(frozen=True)
class TargetParams:
    x: float = 100000
    z: float = 2000
    speed: float = 300
    RCS: float = 1.0
    
@dataclass(frozen=True)
class RadarParams:
    range_noise_std: float = 10.0
    azimuth_noise_std: float = 0.01
    carrier_freq: float = 10e9
    PRF: float = 1000
    Duty_cycle: float = 10 # % percent
    peak_power: float = 100000
    aperature_area: float = 5 # m^2 
    antenna_efficiency: float = 1
    antenna_resistance: float = 50 # ohms
    pitch_angle: float = np.deg2rad(30)
    radar_beamwidth: float = np.deg2rad(5)  # example beamwidth in radians
    
    
@dataclass(frozen=True)
class GroundParams:
    dt_extrap: float = 0.2
    
    
    
    
    
    
    
    

    
    
    
    
   
    
    
