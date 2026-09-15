import numpy as np


class Radar:
    def __init__(self, range, fov, dt_sim, time_since_last_radar_measurement, measurement_interval, beamwidth=None, true_radar_pitch_angle=None):
        self.range = range
        self.fov = fov
        self.dt_sim = dt_sim
        self.time_since_last_radar_measurement = time_since_last_radar_measurement
        self.measurement_interval = measurement_interval
        self.beamwidth = beamwidth
        self.true_radar_pitch_angle = true_radar_pitch_angle
        self.mode = "search"

    def radar_measurement_processing_available(self):
        # checks if enough time has passed since the last radar measurement to process a new one ()
        
        
        if self.time_since_last_radar_measurement >= self.measurement_interval:
            return True, 0.0
        else:
            return False, self.time_since_last_radar_measurement + self.dt_sim
        

    def gimble_change_available(self, time_since_last_gimble_change, gimble_change_interval, dwell_time):
        # checks if the radar gimbal can change its orientation based on the elapsed time and dwell time 
        
        
        if time_since_last_gimble_change >= gimble_change_interval + dwell_time:
            return True, 0.0
        else:
            return False, time_since_last_gimble_change + self.dt_sim
        
        
    def set_mode(self,true_target_state_RNED, beam_angle, true_radar_pitch_angle ):
        # sets the radar mode to "track" if the target is detected, otherwise "search"
        if self.target_detected_true(true_target_state_RNED, beam_angle, true_radar_pitch_angle):
            self.mode = "track"
        else:
            self.mode = "search"
        
        return self.mode
    


    def beam_steering_track(self, target_state_estimate, beam_angle, gimble_dt, dwell_time):
        
        # this logic tries to steer the radar beam to track the target based on the estimated target state
        
        # uses estimated target state since radar doesnt have direct access to the true target state
        
        # convert target_state to RADAR frame
        target_state_RADAR = self.Radar_to_NED(1, target_state_estimate, self.true_radar_pitch_angle, beam_angle)
        POS_TARGET_RADAR = target_state_RADAR[0:2]
        VEL_TARGET_RADAR = target_state_RADAR[2:4]
        
        LOS_RADAR = -np.atan2(POS_TARGET_RADAR[1], POS_TARGET_RADAR[0])
        
        dt_total = gimble_dt + dwell_time
        
        # drive beam angle to track target line of sight
        BEAM_DOT = 5.0 * LOS_RADAR
        
        return beam_angle + BEAM_DOT * dt_total
    
    def beam_steering_search(self, beam_angle, dwell_time, time_since_dwell, fov_max, nominal_radar_pitch_angle, cmd_iter):

        # this logic generates a sequence of beam commands within the field of view for search mode (all radians)
        delta_cmd_seq = np.linspace(-nominal_radar_pitch_angle, fov_max, 15)

        if time_since_dwell >= dwell_time:
            # dwell time elapsed on current angle, advance to the next angle in the sweep
            cmd_iter = (cmd_iter + 1) % len(delta_cmd_seq)
            beam_angle = delta_cmd_seq[cmd_iter]
            time_since_dwell = 0.0
        else:
            # hold current beam angle, accumulate dwell time
            time_since_dwell += self.dt_sim

        return beam_angle, cmd_iter, time_since_dwell
    
        
        
        
        
        
        
        

    def target_detected_true(self, true_target_state_RNED, beam_angle, true_radar_pitch_angle):
        
        # determines if the true target is within the radar's detection range and beamwidth
   
        target_state_RADAR = self.Radar_to_NED(1, true_target_state_RNED, true_radar_pitch_angle, beam_angle)
        POS_TARGET_RADAR = target_state_RADAR[0:2]
        VEL_TARGET_RADAR = target_state_RADAR[2:4]
        
        # los angle kept in radians, matching self.beamwidth's units
        los_RADAR = -np.atan2(POS_TARGET_RADAR[1], POS_TARGET_RADAR[0])
        range = np.linalg.norm(POS_TARGET_RADAR)
        range_rate = np.dot(POS_TARGET_RADAR, VEL_TARGET_RADAR) / range

        if range >= self.range:
            target_acquired = False
            return target_acquired
        elif abs(los_RADAR) > self.beamwidth:
            target_acquired = False
            return target_acquired
        else:
            target_acquired = True
            return target_acquired, range, los_RADAR, range_rate
        
    def Radar_to_NED(self, direction, target_state, radar_pitch_angle, beam_angle):
        # converts target state between Radar frame (boresight X_R, cross-range Z_R) and NED frame (North X, Down Z)
        # 0 --> RADAR TO NED
        # 1 --> NED TO RADAR
        
        

        theta_rad = beam_angle + radar_pitch_angle  # elevation angle above horizon (rad)
        theta = theta_rad

        # rotation matrix from Radar to NED frame
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta),  np.cos(theta)]])

        pos = target_state[0:2]
        vel = target_state[2:4]

        if direction == 0:  # RADAR TO NED
            pos_out = R.T @ pos
            vel_out = R.T @ vel
            return np.concatenate([pos_out, vel_out])
        elif direction == 1:  # NED TO RADAR
            pos_out = R @ pos
            vel_out = R @ vel
            return np.concatenate([pos_out, vel_out])
    



    def get_true_measurement(self, true_target_state, true_radar_pitch_angle, beam_angle):
        
        # gets the true radar measurement if the target is detected
        
        if self.target_detected_true(true_target_state, beam_angle, true_radar_pitch_angle):
            
            # true target state is in the Radar NED frame
            
            TRUE_TARGET_STATE_RADAR = self.Radar_to_NED(1, true_target_state, true_radar_pitch_angle, beam_angle)
            
            POS_TARGET_RNED = TRUE_TARGET_STATE_RADAR[0:2]
            VEL_TARGET_RNED = TRUE_TARGET_STATE_RADAR[2:4]
            
            # los angle kept in radians to match other radar angle parameters
            LOS_RADAR = -np.atan2(POS_TARGET_RNED[1], POS_TARGET_RNED[0])
            RANGE_RADAR = np.linalg.norm(POS_TARGET_RNED)
            RANGE_RATE = np.dot(POS_TARGET_RNED, VEL_TARGET_RNED) / RANGE_RADAR
            
            TRUE_MEASUREMENT = np.array([LOS_RADAR, RANGE_RADAR, RANGE_RATE])
            return TRUE_MEASUREMENT
        else:
            return None
            
            
        
            

    def get_noisy_measurement(self, true_radar_measurement, measurement_noise_std):
        # gets true radar measurement with added noise
    
        if true_radar_measurement is None:
            return None
        else:
            noisy_measurement = true_radar_measurement + np.random.randn(*true_radar_measurement.shape) * measurement_noise_std
            return noisy_measurement
    
    

    

        
    
       
         
    
    