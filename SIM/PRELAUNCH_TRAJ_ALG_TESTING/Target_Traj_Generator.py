
from Paramater import TargetParams, SimParams, AssetParams
import numpy as np



sim = SimParams()
target = TargetParams()
asset = AssetParams()  


## UPDATE TO ONLY EXTRAP TO ASSET


target_traj = [] # X AND Z 
target_traj.append([target.x, target.z, -target.speed, 0])  # initialize the first row with zeros






# SIMPLE CONSTANT VELOCITY MODEL FOR TARGET TRAJECTORY 

# --- generates csv of  full target trajectory for use in simulations ---- #  (current in rned for future better to be in ecef/lla)
## UPDATE TO ONLY EXTRAP TO ASSET
def get_target_trajectory(target_traj, sim):

    i = 0
    while target_traj[i][0] >= asset.x + asset.radius:  # Extrapolate until the target reaches the asset's x position
        ## UPDATE TO ONLY EXTRAP TO ASSET
        target_traj.append([
            target_traj[i][0] + target_traj[i][2] * sim.dt_sim,
            target_traj[i][1] + target_traj[i][3] * sim.dt_sim,
            target_traj[i][2],
            target_traj[i][3]
        ])
        print(f"Step {i}: Target position = {target_traj[i+1][0:2]}")
        
        i += 1
    target_traj = np.array(target_traj)
    return target_traj

target_traj = get_target_trajectory(target_traj, sim)

# save to CSV (readable in Excel / any text editor)
np.savetxt(
    "target_traj.csv",
    target_traj,
    delimiter=",",
    header="x,z,vx,vz",
    comments="",          # keep the header clean (no '#' prefix)
    fmt="%.6f",
)





