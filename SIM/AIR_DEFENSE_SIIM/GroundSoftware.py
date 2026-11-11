from types import SimpleNamespace
import numpy as np

from Paramaters import *
from TargetEstimationExtrap import TargetEstimationExtrap
from PrelaunchSolution import PrelaunchSolution



class GroundSoftware:

    def __init__(self):
        self.TSEE = TargetEstimationExtrap()
        self.PreLaunchSolution = PrelaunchSolution()
        
        
    def prelaunch_paramaters(self, extrapolated_target_state, validity_time):
        # Example: set simulation time step to 0.1 seconds
        prelaunch_paramaters = self.PreLaunchSolution.get_prelaunch_solution(extrapolated_target_state, validity_time)
        return prelaunch_paramaters