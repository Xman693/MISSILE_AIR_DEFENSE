
from types import SimpleNamespace
import numpy as np

from Paramaters import *

sim = SimParams()
asset = AssetParams()
ground = GroundParams()

class TargetEstimationExtrap:

    def __init__(self):
        self.sim = sim
        self.asset = asset
        self.ground = ground
        
# def target_estimation(self, radar_measurements):
    
#     # Kalman Filter 
#     # 11/7 : skip target estimation for now, just use radar measurements as estimated state
    
#     # The z = h(x) need to take the ned pos and rotate into antenna frame since measurmerns are made 
#     # in antenna frame, but the model is in the ned frame 
#        estimated_target_state = radar_measurements[:6]  # Example: only position [x, y, z]
# return estimated_target_state

    def target_extrapolation(self, target_state):
    
        target_traj = []

        while target_state.x > self.asset.x + self.asset.radius:
            target_state.x = target_state.x + target_state.vx * self.ground.extrap_time_step
            target_state.z = target_state.z + target_state.vz * self.ground.extrap_time_step
        
            target_traj.append(SimpleNamespace(**vars(target_state)))
            
            
            
    
        return target_traj
        
        
    
    def get_trimmed_traj(self, target_traj, target_time_to_asset):
    
    # trim full target trajectory from current point to the tgo to asset,
    # flipped so row 0 is the END of the window (closest to the asset)
        
        num_points = int(target_time_to_asset / self.ground.extrap_time_step)
        
        # keep the final window of the trajectory in forward time order:
        # row 0 = start of window, last row = end (closest to asset)
        target_traj_trimmed = target_traj[-num_points:]
    
        return target_traj_trimmed
    
    
