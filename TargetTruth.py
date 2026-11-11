import numpy as np
from types import SimpleNamespace
from Paramaters import SimParams, AssetParams


class TargetTruth:
    def __init__(self):
        self.sim = SimParams()
        self.asset = AssetParams()
        self.target_x = 100000
        self.target_z = -30000
        self.target_vx = -300
        self.target_vz = 0

        # build the trajectory as soon as the class is constructed
        self.target_state, self.target_traj = self.generate_target_truth()

    def generate_target_truth(self):
        target_state = SimpleNamespace(
            x=self.target_x,
            z=self.target_z,
            vx=self.target_vx,
            vz=self.target_vz
        )
        target_traj = []

        while target_state.x > self.asset.x + self.asset.radius:
            target_state.x = target_state.x + target_state.vx * self.sim.sim_time_step
            target_state.z = target_state.z + target_state.vz * self.sim.sim_time_step

            target_traj.append(SimpleNamespace(**vars(target_state)))
            
        

        return target_state, target_traj
    
    
target = TargetTruth()







