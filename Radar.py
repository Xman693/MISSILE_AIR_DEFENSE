from Paramaters import RadarParams
from Helpers import RadarNEDtoAntenna, RadarNEDtoFace

import numpy as np
from types import SimpleNamespace




class Radar:
    def __init__(self):
        self.radar = RadarParams()

    def execute_radar_logic(self, target_state_truth_NED):
        
        # radar measures range, range rate, los in antenna frame
        
        # Example: only position [x, y, z]
        
        # 11/7 Assume radar has already found and tracking target. 
        
        # first step is to convert the NED target state into the radar's fae frame so we can find the 
        # desired beam angle
        target_state_radar_face  = RadarNEDtoFace(self.radar.pitch_angle,  target_state_truth_NED, 0)
        
  
        # angle of target above anntenna (where we should point the beam)
        los = -np.atan2(target_state_radar_face[1], target_state_radar_face[0]) 
        beam_angle = los
        
        beam_angle_inertial = beam_angle + self.radar.pitch_angle
        
        
        # here we would check if the true target is within beam_angle_inertial +/- beamwidth 
        # 11/7: Assume this is true for now so we can skip the beam scanning logic for now 
        
        # 
        target_state_antenna  = RadarNEDtoAntenna(beam_angle_inertial,  target_state_truth_NED, 0)
        
        
        range = np.linalg.norm(target_state_antenna[:2])
        range_rate = np.dot(target_state_antenna[:2], target_state_antenna[2:4])/range
        los = -np.arctan2(target_state_antenna[1], target_state_antenna[0]) 
        
        range_meas = range + np.random.randn() * self.radar.range_std
        range_rate_meas = range_rate + np.random.randn() * self.radar.range_rate_std
        los_meas = los + np.random.randn() * self.radar.los_std

        radar_measurement = SimpleNamespace(
            range=range_meas,
            range_rate=range_rate_meas,
            los = los_meas, 
            beam_angle = beam_angle
        )
        
        status = SimpleNamespace(
            target_detected=True
        )
        
        
        
        
        
        return radar_measurement, status