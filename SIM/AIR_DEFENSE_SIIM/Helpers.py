import numpy as np
from types import SimpleNamespace

def RadarNEDtoFace(pitch_angle, target_state, direction):
    
    # convert namespace to array to do math 
    target_state = np.array([target_state.x, target_state.z, target_state.vx, target_state.vz])
    
    if direction == 0:
        # radar ned to face rotation
    # 2D rotation in the plane defined by pitch_angle
        R = np.array([
            [np.cos(pitch_angle),  -np.sin(pitch_angle)],
            [np.sin(pitch_angle), np.cos(pitch_angle)]
        ])
        target_state[:2] = R @ target_state[:2]
        target_state[2:] = R @ target_state[2:]
    else:
        # face to radar ned rotation
        R = np.array([
            [np.cos(pitch_angle),  -np.sin(pitch_angle)],
            [np.sin(pitch_angle), np.cos(pitch_angle)]
        ]).T
        
        target_state[:2] = R @ target_state[:2]
        target_state[2:] = R @ target_state[2:]

   
    
    return target_state

def RadarNEDtoAntenna(beam_angle_inertial, target_state, direction):
    
        target_state = np.array([target_state.x, target_state.z, target_state.vx, target_state.vz])
        
    
      
        if direction == 0:
            # radar ned to face rotation
        # 2D rotation in the plane defined by pitch_angle
            R = np.array([
                [np.cos(beam_angle_inertial),  -np.sin(beam_angle_inertial)],
                [np.sin(beam_angle_inertial), np.cos(beam_angle_inertial)]
            ])
            target_state[:2] = R @ target_state[:2]
            target_state[2:] = R @ target_state[2:]
        else:
            # face to radar ned rotation
            R = np.array([
                [np.cos(beam_angle_inertial),  -np.sin(beam_angle_inertial)],
                [np.sin(beam_angle_inertial), np.cos(beam_angle_inertial)]
            ]).T
            
            target_state[:2] = R @ target_state[:2]
            target_state[2:] = R @ target_state[2:]
            
    
    
    
    
        return target_state