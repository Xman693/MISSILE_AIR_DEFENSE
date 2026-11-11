import numpy as np
from dataclasses import dataclass

@dataclass
class SimParams:
    sim_time_step: float = 0.01
    max_time: float = 100.0

@dataclass
class MissileParams:
    mass: float = 100.0
    length: float = 5.0
    diameter: float = 0.5

@dataclass
class AeroParams:
    CD_alpha: float = 0.5
    CL_alpha: float = 0.1

@dataclass
class Enviroment:
    gravity: float = 9.81
    air_density: float = 1.225
    

@dataclass
class RadarParams:
    emplacement_lat: float = 30 # degrees
    emplacement_alt: float = 0
    pitch_angle: float = np.deg2rad(30)
    range_std: float = 100
    range_rate_std: float = 20
    los_std: float = np.deg2rad(1)
    beamwidth: float = np.deg2rad(5)
    PRI: float = 1/1000

    
@dataclass
class AssetParams:
    x : float = 10000 # meters
    z : float = 0
    radius: float = 1000 # meters
    
@dataclass
class GroundParams:
    extrap_time_step: float = 0.1
    TSEE_period = 1

@dataclass(frozen=True)
class ThrustParams:
    T: float = 15000
    Thold: float = 15
    Tramp: float = 0.1


@dataclass(frozen=True)
class AeroParams:
    cl0: float = 0.0
    CL_alpha: float = 8
    cd0: float = 0.1
    CD_alpha: float = 0.5
    Cm_alpha: float = 1 # slighy staticly unstable
    Cm0: float = 0.0
    Cm_q: float = 15 # pitch damping coefficient
    
    
    
@dataclass(frozen=True)
class MissileParams:
    m: float = 100
    d: float = 0.25
    S: float = np.pi * (0.25 ** 2)/4 # m^2
    c: float = 1 # m
    
@dataclass(frozen=True)
class LauncherParams:
    launcher_emplacement_x: float = 1000
    launcher_emplacement_z: float = 0
    launcher_angle: float = np.deg2rad(30)
    emplacement_lat: float = 30.1 # degrees
    emplacement_alt: float = 0.0
    
@dataclass(frozen=True)
class ThrustParams:
    T: float = 25000 # Netwons
    Thold: float = 5 # sec
    Tramp: float = 0.3 # sec
   
    
    
    
    
    



