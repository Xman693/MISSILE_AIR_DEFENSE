import numpy as np


import matplotlib.pyplot as plt

from PARAMATERS import *
from TARGET_TRUTH import Truth
from RADAR_V1 import Radar
from FireControl import FireControl, state_lookup, target_state_extrapolation
from LOGGER import Logger
from INITIAL_TURN_STATE_TABLE import get_initial_turn_state_table
from Scheduler import update_clocks

#state_table = get_initial_turn_state_table()
fc_params = get_fc_params()
covar_matrices = get_covariance_matrices()


def Run_Simulation(sim_params, true_target_state, radar_params): 
    truth = Truth(
        dt_sim=sim_params["dt"],
        target_state=true_target_state,
        t_max=sim_params["t_max"],
    )
    true_target_trajectory = truth.get_target_trajectory()
    
    
    
    # ---------------Clock initialization --------------------------------
    radar_beam_change_clock = 0.0
    radar_update_clock = 0.0
    fc_update_clock = 0.0
    
    # --------------------------------- radar initialization -------------------------------------------------------------------
    noisy_radar_measurement_prev = np.array([0.0, 0.0, 0.0])
    true_radar_measurement = None
    noisy_radar_measurement = None
    mode = "search"
    beam_angle = 0.0
    cmd_iter = 0
    counter = 0
    
    # --------------------------------- fire control initialization -------------------------------------------------------------------
    target_extrapolated_trajectory = None
    intercept_point = None
    launch_decision = False
    gamma_initial_turn = np.nan
    t_burst = np.nan
    time_of_flight = np.nan
    launch_cmd = False

   
    # -------------------------------- target state estimate initialization --------------------------------
    target_state_estimate = np.zeros(4)
    
       # -------------------------------------inititialize classses ------------------------------
    radar = Radar(range=radar_params["max_range"], beamwidth = radar_params["beamwidth"], fov=radar_params["fov_max"],  true_radar_pitch_angle=radar_params["true_radar_pitch_angle"])
    fire_control = FireControl(P_initial=covar_matrices["P"], Q=covar_matrices["Q"], R=covar_matrices["R"])
            

    # --------------------------------- logger initialization --------------------------------
    logger = Logger()



















    for i in range(sim_params["iter"]):
        t = i * sim_params["dt"]
        
        # --------------------------- get scheduler flags for radar and fire control -------------------------------
        radar_beam_change_clock, radar_update_clock, fc_update_clock, radar_beam_change_due, radar_update_due, fc_update_due = update_clocks(radar_beam_change_clock, radar_update_clock, fc_update_clock) # 
        
     
    
        # -------------------------------------- call True Target --------------------------------------------------------------------------------------------------------
        true_target_state = truth.get_target_at_time(t, true_target_trajectory)
        target_state_estimate = true_target_state[:4]

       
       # --------------------------------------- beam steering logic -------------------------------------------------------------------
       
        if radar_beam_change_due:
            if mode == "track":
                beam_angle = radar.beam_steering_track(target_state_estimate, beam_angle, radar_params["radar_beam_change_interval"])
            else:
                beam_angle, cmd_iter = radar.beam_steering_search(beam_angle, radar_params["fov_max"], radar_params["nominal_radar_pitch_angle"], cmd_iter)
            
       
     
        # ------------------------------------ obtain radar measurements -------------------------------------------------------------------
        if radar_update_due:
    
           
            mode = radar.set_mode(true_target_state_RNED=true_target_state[:4], beam_angle=beam_angle, true_radar_pitch_angle=radar_params["true_radar_pitch_angle"]) # detection logic / set mode logic
            if mode == "track":
                true_radar_measurement = radar.get_true_measurement(true_target_state[:4], radar_params["true_radar_pitch_angle"], beam_angle)
                noisy_radar_measurement = radar.get_noisy_measurement(true_radar_measurement, radar_params["radar_measurement_noise"])
               
            else:
                true_radar_measurement = None
                noisy_radar_measurement = None
            
       
         # --------------------------------- Fire Control (Estimate, Extrapolation, Guidance Cmds) -------------------------------------------------------------------
      
        
    
        
    #    # if fc_update_due:
    #         if launch_cmd == False:
    #          # pre launch guidance, time_since_launch_cmd_sent
            
    #         elif launch_cmd == True and missile_tof > 1:
    #             # midcourse guidance, gamma_cmd_uplink time_since_uplink_sent (IFA goes here for future)
                
                
        # --------------------------- missile guidance -------------------------------------------
        
        # if time_since_launch_sent >= launch_delay and PHASE = "INTIAL":
        # missiles uses open loop pre compued t burst as control input
        
        
        
        
        # if time_since_uplink_sent >= uplink_delay and PHASE = "midcourse":
            # gamma_cmd_recieved = gamma_cmd_uplink # MISSILE RECIEVES UPLINK GAMMA CMD AND USES IT IN AUTOPILOT
        
        # if PHASE = "endgame" or "homing":
        # missile ignores any ground command and uses its own seeeker to guide itself to the target
        
        # ----------------- missile autopilot -------------------------------------------
        
                 # USE CMDS FROM ABOVE
        
                 # -------------- missile truth -------------------------------------------------

                    # integrate missile truth according to applied inputs
        
                 # ------------ missile state estimator ------------------------------------------

                    # estimate missile state from IMU/KF --> feedback to autopilot
                    
                    
        # send missile downlink 
        
        
        
    
        
        
        
                
                
                
                
                
              
        
            
     
            
            
            
            
            





        # ------------------------------------ log everything -------------------------------------------------------------------
        logger.log_step(
            t=t,
            true_target_state=true_target_state,
            target_state_estimate=target_state_estimate,
            mode=mode,
            beam_angle=beam_angle,
            true_radar_measurement=true_radar_measurement,
            noisy_radar_measurement=noisy_radar_measurement,
            radar_update_due=radar_update_due,
            radar_beam_change_due=radar_beam_change_due,
            cmd_iter=cmd_iter,
            launch_decision=launch_decision,
            gamma_initial_turn=gamma_initial_turn,
            t_burst=t_burst,
            time_of_flight=time_of_flight,
        )

    logger.plot_results(radar_pitch_angle=radar_params["true_radar_pitch_angle"])
    logger.plot_scheduler_clocks()
    logger.plot_pre_launch_guidance()
    anim = logger.animate_results(radar_pitch_angle=radar_params["true_radar_pitch_angle"], beamwidth=radar_params["beamwidth"], dt_sim=sim_params["dt"])
    return logger, anim


if __name__ == "__main__":
    
    # load paramaters ----------------------------------------------------------------
    sim_params = get_simulation_parameters()
    true_target_state = get_target_true_parameters() # gets initial target state
    radar_params = get_radar_parameters() # gets initial radar parameters
    fc_params = get_fc_params() # gets initial flight computer parameters
    
    # may run sim in another monte carlo script 
    Run_Simulation(sim_params, true_target_state, radar_params)
