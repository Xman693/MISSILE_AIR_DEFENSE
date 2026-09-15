import numpy as np


class FireControl:
    def __init__(self):
        pass
    
    
    def KalmanFilter(self, measurement, R, Q, F, P, x):
        pass
    
    def RadartoNED(self, target_state_estimate_RADAR, nominal_radar_pitch_angle, beam_angle):
        
        delta = beam_angle
        theta = nominal_radar_pitch_angle
        
        phi = delta + theta 
        
        R = np.array([[np.cos(phi), -np.sin(phi)],
                      [np.sin(phi),  np.cos(phi)]])  # rotation matrix from Radar to NED frame
        
        target_pos_estimate_NED = R @ target_state_estimate_RADAR[0:2]
        target_vel_estimate_NED = R @ target_state_estimate_RADAR[2:4]

        target_state_estimate_NED = np.concatenate([target_pos_estimate_NED, target_vel_estimate_NED])
        return target_state_estimate_NED
        
        
        
        
        
    
    

    def estimate_target_state(self, noisy_radar_measurement, nominal_radar_pitch_angle):
        # Placeholder for the actual target state estimation logic
        # For now, just return the noisy measurement as the estimate
        
        target_state_estimate_RADAR = self.KalmanFilter(noisy_radar_measurement)
        
        
        
        
        return target_state_estimate