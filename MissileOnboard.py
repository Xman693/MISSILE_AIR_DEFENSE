import numpy as np
from types import SimpleNamespace

from Paramaters import SimParams, EstMissileParams, EstAeroParams, Enviroment, EstLauncherParams, GuidanceParams
from Seeker import Seeker
import Helpers
from IntegrateIMU import IntegrateIMU



sim = SimParams()
missile = EstMissileParams()
aero = EstAeroParams()
env = Enviroment()
launcher= EstLauncherParams()
guidance = GuidanceParams() 


class MissileOnboard:
    def __init__(self):
        self.aero = aero
        self.sim = sim
        self.missile = missile
        self.env = env
        self.launcher = launcher
        self.guidance = guidance
        
        
def get_seeker_measurement(target_state_truth):

    target_state_estimate = target_state_truth
    
    range = np.linalg.norm(target_state_estimate[:2])
    
    seeker = SimpleNamespace(
        range=range
        # add on later
        
    )

    return seeker, target_state_estimate
         
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
def initialize_missile_onboard(self):
    
    lat = self.launcher.emplacement_lat 
    alt = self.launcher.emplacement_alt
    gamma = self.launcher.emplacement_pitch
     
    x_NED = 1000 # relatvie to radar 
    z_NED = 0.0
    
    # 30 m/s coming off rail (but for now dont model thrust / drag so start it very high)
    vx_NED = 1000 * np.cos(gamma) ## UPDATE -------
    vz_NED = 1000 * np.sin(gamma)
    
    missile_state = SimpleNamespace(
        x=x_NED,
        z=z_NED,
        vx=vx_NED,
        vz=vz_NED,
        gamma=gamma,
        lat=lat,
        alt=alt
    )
    
    status = SimpleNamespace(
        missile_away=False,
        tube_cleared=False,
        initial_turn_completed=False,
        midcourse_completed=False,
        homing_guidance_completed=False,
        final_lunge_completed=False,
        self_destruct=False
    )
   
    return missile_state, status


def get_guidance_cmd(self, prelaunch_solution, uplink_data, missile_state, phase, tof):
    
    if phase == "MISSILE_AWAY":
        normal_acc_cmd = 0
    elif phase == "MISSILE_TAIL_OUT_OF_TUBE":
        normal_acc_cmd = 0
    elif phase == "INITIAL_TURN" or phase == "MIDCOURSE": 
        
        if uplink_data.replan == True:
            
            coeff_ecef_x = uplink_data.shaping_coefficients_ecef_x # NUM OF SPLINES x (DEG OF POLYNOMIAL + 1)
            coeff_ecef_z = uplink_data.shaping_coefficients_ecef_z 
        
        
        coeff_ecef_x = prelaunch_solution.shaping_coefficients_ecef_x # NUM OF SPLINES x (DEG OF POLYNOMIAL + 1)
        coeff_ecef_z = prelaunch_solution.shaping_coefficients_ecef_z # NUM OF SPLINES x (DEG OF POLYNOMIAL + 1)
        intial_phase_coeff_ecef_x = coeff_ecef_x[0, :]
        intial_phase_coeff_ecef_z = coeff_ecef_z[0, :]
        
        
        # grab way point at tof + 5 seconds
        x_ecef_waypoint = np.polyval(intial_phase_coeff_ecef_x, tof + 5)
        z_ecef_waypoint = np.polyval(intial_phase_coeff_ecef_z, tof + 5)
        
        # convert from ecef to local ned 
        
        x = missile_state.x
        z = missile_state.z
        lat = missile_state.lat
        alt = missile_state.alt  
        vx = missile_state.vx
        vz = missile_state.vz
        
        [x_wp_ned, z_wp_ned] = Helpers.ecef2ned(x_ecef_waypoint, z_ecef_waypoint, self.radar.emplacement_lat, self.radar.emplacement_alt)
        
        deltaz = z_wp_ned - z
        deltax = x_wp_ned - x
        
        gamma_des = -np.arctan2(deltaz, deltax)  # negative because of NED convetion
        gamma_current = -np.arctan2(vz, vx)
        
        gamma_error = gamma_des - gamma_current
        
        if phase == "INITIAL_TURN":
            gamma_rate = self.guidance.intial_turn_gain * gamma_error # higher gain to achieve rapid initial turn
        else:
            gamma_rate = self.guidance.midcourse_gain * gamma_error
        
        normal_acc_cmd = np.linalg.norm([vx, vz]) * gamma_rate # an = V * gamma_dot
        
    elif phase == "ACQURING_TARGET": 

        # hold gamma angle until target is acquired via seeker
        
        # call seeker detection logic here from seeker class (gimbal seeker based on acqustion cue volume from )
        seeker_state, target_state_estimate = Seeker.get_seeker_measurement()
        
        # compute guidance in the meantime 
        normal_acc_cmd = 0  # hold gamma angle until target is acquired
    elif phase == "HOMING":
        
        # PN guidance logic based on seeker measurements
        seeker_state, target_state_estimate = Seeker.get_seeker_measurement()
        
        # ** seeker derived state estimate are in the relative frame of the missile
        
        x_rel = target_state_estimate[0] # 
        z_rel = target_state_estimate[1]
        vx_rel = target_state_estimate[2]
        vz_rel = target_state_estimate[3]
        
        # compute PN and rotate into inertial frame
        
    elif phase == "FINAL_LUNGE":
    
    # more aggresive PN gain 
       

        x_rel = target_state_estimate[0]
        z_rel = target_state_estimate[1]
        vx_rel = target_state_estimate[2]
        vz_rel = target_state_estimate[3]

        # compute PN and rotate into inertial frame
      
    return normal_acc_cmd
    
  

def propogate_onboard_missile(self, prelaunch_solution, uplink_data, missile_state, phase, tof, status, seeker):
    
    if uplink_data.self_destruct == 0:
    
    # determine what guidance phase the missile is in
    
        
        normal_acc_cmd = self.get_guidance_cmd(prelaunch_solution, uplink_data, missile_state, phase, tof)
        missile_state = IntegrateIMU.integrate_missile_state(missile_state, normal_acc_cmd)
    
        tof = tof + self.sim.IMU_time_step
        
        # determines status of enagement 
        status = get_status(missile_state, tof, prelaunch_solution, seeker, phase, status)
        
        # if uplink_data cotains self destruct command
        if uplink_data.self_destruct == 1:
            status.self_destruct = True
        
        # for now just downlink the status
        downlink = status
   
    
    return missile_state, status, downlink, tof

def get_status(missile_state, tof, prelaunch_solution, seeker, phase, status): 
    
    if phase == "MISSILE_TAIL_OUT_OF_TUBE":
        
        
        # check if missile has cleared the launch tube based on its state
        XPOS_MISSILE = missile_state.x
        ZPOS_MISSILE = missile_state.z
        
        DISTANCE_TRAVELLED = np.sqrt(XPOS_MISSILE**2 + ZPOS_MISSILE**2)
        
        if DISTANCE_TRAVELLED > launcher.TUBE_LENGTH:  # assuming missile has cleared the launch tube if it has move
            status.tube_cleared = True

    elif phase == "INITIAL_TURN":
        
        if tof > 3:
            status.initial_turn_completed = True
    elif phase == "MIDCOURSE":
        
        if tof >= prelaunch_solution.acquistion_time:
            status.midcourse_completed = True
    elif phase == "ACQURING_TARGET":
        
        if seeker.target_acquired == True:
            status.target_acquired = True
            
    elif phase == "HOMING":
        
        if seeker.tgo_estimate < 2.0:
            status.homing_guidance_completed = True
        
    elif phase == "FINAL_LUNGE":
     
        if seeker.range_estimate < 0.5: 
            status.final_range = seeker.range_estimate
        
        
  
    return status

