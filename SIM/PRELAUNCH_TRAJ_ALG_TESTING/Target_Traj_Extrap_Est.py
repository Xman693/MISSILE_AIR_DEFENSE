import numpy as np
from Paramater import AssetParams, SimParams

# Estimates/ Extrapolates target trajectory based on current state and velocity

# Generates extrapolated target to asset based on current state and velocity
# Also generates trimmed trajecotry for supporting optimal launch timing
# Currently uses true target traj for now

target_traj = np.loadtxt("target_traj.csv", delimiter=",", skiprows=1)  # skiprows=1 skips the header row
asset = AssetParams()
sim = SimParams()





def extrapolate_target_traj(target_state, dt, asset):
    
    target_traj = [] # X AND Z 
    target_traj.append([target_state[0], target_state[1], target_state[2], target_state[3]])  # initialize the first row with zeros




    
    i = 0
   
    while target_traj[i][0] >= asset.x:  # Extrapolate until the target reaches the asset's x position
       
        target_traj.append([
            target_traj[i][0] + target_traj[i][2] * dt,
            target_traj[i][1] + target_traj[i][3] * dt,
            target_traj[i][2],
            target_traj[i][3]
        ])
        
        i += 1
    target_traj = np.array(target_traj)
    return target_traj















def get_trimmed_traj(target_traj, dt, time_to_asset_desired):
    
    # trim full target trajectory from current point to the tgo to asset,
    # flipped so row 0 is the END of the window (closest to the asset)
        
        num_points = int(time_to_asset_desired / dt)
        
        # keep the final window of the trajectory in forward time order:
        # row 0 = start of window, last row = end (closest to asset)
        target_traj_trimmed = target_traj[-num_points:, :]
            
        return target_traj_trimmed
    
    
    
    
target_traj_trimmed = get_trimmed_traj(target_traj,dt=sim.dt_sim, time_to_asset_desired=30) 
        
        
np.savetxt(
    "target_traj_trimmed.csv",
    target_traj_trimmed,
    delimiter=",",
    header="x,z,vx,vz",
    comments="",          # keep the header clean (no '#' prefix)
    fmt="%.6f",
)