
import numpy as np

def get_launch_cmd(best_launch_time, sim_clock):
    
    if sim_clock >= best_launch_time:
        return "Launch"
    else:
        return "Hold"