import numpy as np


import matplotlib.pyplot as plt

from PARAMATERS import *
from TRUTH import Truth
from RADAR_V1 import Radar
#from FIRE_CONTROL import FireControl
from LOGGER import Logger


def Run_Simulation(sim_params, true_target_state_prev, radar_params): 
    
  
   
    # --------------------------------- radar initialization -------------------------------------------------------------------
    time_since_last_radar_measurement = 0.0 # initialize the time since the last radar measurement
    time_since_last_gimble_change = 0.0 # initialize the time since the last gimble change
    noisy_radar_measurement_prev = np.array([0.0, 0.0, 0.0])
    true_radar_measurement = None
    noisy_radar_measurement = None
    mode = "search"
    beam_angle = 0.0
    cmd_iter = 0
    counter = 0

    # this version has no estimator yet, so the target state estimate is just the true target state
    target_state_estimate = true_target_state_prev

    logger = Logger()

    for i in range(sim_params["iter"]):
        t = i * sim_params["dt"]

        # -------------------------------------- call True Target --------------------------------------------------------------------------------------------------------
        truth = Truth(dt_sim=sim_params["dt"], true_target_state_prev=true_target_state_prev)
        true_target_state_prev = truth.get_true_target() # updates the target state for the current simulation step

        # this version has no estimator yet, so the target state estimate is just the true target state
        target_state_estimate = true_target_state_prev

        # ----------------------------------------call Radar ----------------------------------------------------------------------------------------------------------
        radar = Radar(range=radar_params["max_range"], beamwidth = radar_params["beamwidth"], fov=radar_params["fov_max"], dt_sim=sim_params["dt"], time_since_last_radar_measurement=time_since_last_radar_measurement, measurement_interval=radar_params["measurement_interval"], true_radar_pitch_angle=radar_params["true_radar_pitch_angle"])
        radar_measurement_processing_available, time_since_last_radar_measurement = radar.radar_measurement_processing_available() # time clock logic 
        
       # --------------------------------------- beam steering logic -------------------------------------------------------------------
        gimble_change_available, time_since_last_gimble_change = radar.gimble_change_available(time_since_last_gimble_change, radar_params["gimble_change_interval"], radar_params["dwell_time"])
      
      
        if gimble_change_available:
            if mode == "track":
                beam_angle = radar.beam_steering_track(target_state_estimate[:4], beam_angle, radar_params["gimble_change_interval"])
            else:
                beam_angle, cmd_iter = radar.beam_steering_search(beam_angle, radar_params["fov_max"], radar_params["nominal_radar_pitch_angle"], cmd_iter)
            
       
     
        # ------------------------------------ obtain radar measurements -------------------------------------------------------------------
        if radar_measurement_processing_available:
    
           
            mode = radar.set_mode(true_target_state_RNED=true_target_state_prev[:4], beam_angle=beam_angle, true_radar_pitch_angle=radar_params["true_radar_pitch_angle"]) # detection logic / set mode logic
            if mode == "track":
                true_radar_measurement = radar.get_true_measurement(true_target_state_prev[:4], radar_params["true_radar_pitch_angle"], beam_angle)
                noisy_radar_measurement = radar.get_noisy_measurement(true_radar_measurement, radar_params["measurement_noise_std"])
               
            else:
                true_radar_measurement = None
                noisy_radar_measurement = None
            
       
         # --------------------------------- compute fire control solution -------------------------------------------------------------------
        #fire_control = FireControl()
      
       # fire_control_processing_available = radar.fire_control_processing_available(time_since_last_fire_control_processing, fire_control_params["fire_control_processing_interval"])
        
        #if fire_control_processing_available:
        
        #    target_state_estimate = fire_control.estimate_target_state(noisy_radar_measurement, radar_params["nominal_radar_pitch_angle"])
        
        
            
     
            
            
            
            
            





        # ------------------------------------ log everything -------------------------------------------------------------------
        logger.log_step(
            t=t,
            true_target_state=true_target_state_prev,
            target_state_estimate=target_state_estimate,
            mode=mode,
            beam_angle=beam_angle,
            true_radar_measurement=true_radar_measurement,
            noisy_radar_measurement=noisy_radar_measurement,
            radar_measurement_available=radar_measurement_processing_available,
            gimble_change_available=gimble_change_available,
            cmd_iter=cmd_iter,
        )

    logger.plot_results(radar_pitch_angle=radar_params["true_radar_pitch_angle"])
    logger.plot_radar_measurement_available()
    anim = logger.animate_results(radar_pitch_angle=radar_params["true_radar_pitch_angle"], beamwidth=radar_params["beamwidth"], dt_sim=sim_params["dt"])
    return logger, anim


if __name__ == "__main__":
    
    # load paramaters ----------------------------------------------------------------
    sim_params = get_simulation_parameters()
    true_target_state_prev = get_target_true_parameters() # gets initial target state
    radar_params = get_radar_parameters() # gets initial radar parameters
    
    # may run sim in another monte carlo script 
    Run_Simulation(sim_params, true_target_state_prev, radar_params)
