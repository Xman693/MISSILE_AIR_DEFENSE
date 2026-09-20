import numpy as np 

from PARAMATERS import get_covariance_matrices




def get_KF_matrices():    
    covar_matrices = get_covariance_matrices()
    P = covar_matrices["P"]
    Q = covar_matrices["Q"]
    R = covar_matrices["R"]
    
    
    F = np.array([[1, 0, 1, 0],
                  [0, 1, 0, 1],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    
    return P, Q, R, F
    
def 
    
    
    
    
    

