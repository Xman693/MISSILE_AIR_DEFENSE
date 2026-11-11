import numpy as np
from types import SimpleNamespace

from Paramaters import SimParams, EstMissileParams, EstAeroParams, Enviroment, EstLauncherParams, GuidanceParams
import Helpers



sim = SimParams()
missile = EstMissileParams()
aero = EstAeroParams()
env = Enviroment()
launcher= EstLauncherParams()
guidance = GuidanceParams() 







class IntegrateIMU:
    def __init__(self):
        self.sim = sim
        self.missile = missile
        self.aero = aero
        self.env = env
        self.launcher = launcher
        self.guidance = guidance



    def integrate_missile_state(self, missile_state, normal_acc_cmd):
        # extract state variables
        x = missile_state.x
        z = missile_state.z
        vx = missile_state.vx
        vz = missile_state.vz
        gamma = missile_state.gamma
        lat = missile_state.lat
        alt = missile_state.alt
        
        h = -missile_state.z
        v = np.linalg.norm([vx, vz])
        
        # meridian radius of curvature
        a = 6378137.0  # Earth's equatorial radius in meters
        e = 0.0818191908425  # Earth's eccentricity
        Rm = a * (1 - e**2) / (1 - e**2 * np.sin(lat)**2)**(3/2)

        # integrate position and velocity using simple kinematics
        dt = self.sim.IMU_time_step
        
        gammadot = normal_acc_cmd / v + -self.env.gravity * np.cos(gamma) / v # gamma_dot = an / V
        vdot = -self.env.gravity * np.sin(gamma)  # vertical acceleration due to gravity in NED frame
        Latdot = vx / (Rm + h)  # rate of change of latitude
        hdot = -vz
        
        
        
        
        
        v = v + vdot * dt
        gamma = gamma + gammadot * dt
        lat = lat + Latdot * dt
        h = h + hdot * dt
        
        vx = v * np.cos(gamma)   # will have to upedate this to include corilos effects
        vz = -v * np.sin(gamma) # NED
        
        x = x + vx * dt
        z = z + vz * dt
        

        # update missile state
        missile_state.x = x
        missile_state.z = z
        missile_state.vx = vx
        missile_state.vz = vz
        missile_state.gamma = gamma
        missile_state.lat = lat
        missile_state.alt = h

        return missile_state
