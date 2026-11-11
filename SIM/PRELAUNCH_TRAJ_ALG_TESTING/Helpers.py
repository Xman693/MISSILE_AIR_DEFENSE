import numpy as np

def coordinate_transformation(theta, direction, vector):
    
    # assumes Z direction is pointing down in both frames
    
    if direction == "Antenna2RadarENU":
        # rotation about Y-axis by theta
        R = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])
        return R @ vector
    elif direction == "RadarENU2Antenna":
        # rotation about Y-axis by -theta
        R = np.array([
            [np.cos(-theta), np.sin(-theta)],
            [-np.sin(-theta), np.cos(-theta)]
        ])
        return R @ vector
    else:
        raise ValueError("Invalid direction. Must be 'Antenna2RadarENU' or 'RadarENU2Antenna'.")
