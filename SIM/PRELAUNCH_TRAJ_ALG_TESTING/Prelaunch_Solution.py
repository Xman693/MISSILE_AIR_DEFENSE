# SCRIPT ATTEMPTS TO TEST PRELAUNCH TRAJECTORY PLANNING ALGORITHM FOR IMPLEMENTATION INTO 
# FULL ENGAGEMENT LOOP

# TARGET TRUTH USED IN THIS SCRIPT
# FIRST ITERATION WILL USE VERY SIMPLE VEHICLE KINEMATIC MODEL WITH CONSTANT DRAG, A SIMPLE THRUST PROFILE, AND A BALLISITC COAST



import numpy as np
from dataclasses import dataclass
from Target_Traj_Extrap_Est import get_trimmed_traj
import math
from Paramater import LauncherParams

launcher = LauncherParams()

@dataclass(frozen=True)
class EngagementStats:
    range: np.ndarray          # vehicle-to-target range vs time [m]
    range_rate: np.ndarray     # closing rate vs time [m/s]
    speed_ratio: np.ndarray    # vehicle speed / target speed vs time
    crossing_angle: np.ndarray # angle between velocity vectors vs time [rad]
    speed_percent_loss: np.ndarray  # percent of burnout speed lost vs time [%]
    index_percent: np.ndarray       # percent of target trajectory remaining vs time [%]



#---------------------- Wrapper that generates prelaunch solution -----------------------
def prelaunch_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj):
    
    

    best_state_logging, best_time_logging, best_stats, best_cost, best_launcher_angle, best_tgo = get_prelaunch_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj)
        

    return best_state_logging, best_time_logging, best_stats, best_cost, best_launcher_angle, best_tgo



#---------------------- Vehicle propogator simulates the vehicle trajectory given initial state and parameters -----------------------

def vehicle_propogator(state, aero, thrust, vehicle, dt, launcher_angle, target_traj):
    
    time = 0.0
    state = state.astype(float).copy()

    # initial state for vehicle off rail
    v = 30 # speed off rail

    state = np.array([launcher.launcher_emplacement_x, launcher.launcher_emplacement_z, v * np.cos(np.deg2rad(launcher_angle)), v * np.sin(np.deg2rad(launcher_angle))], dtype=float) # x, z, vx, vz
    
    
    launcher_angle_rad = np.deg2rad(launcher_angle)
    v = np.sqrt(state[2]**2 + state[3]**2)
    gamma = launcher_angle_rad 
    
    state_log = [state.copy()]
    time_log = [time]
    
    
    while state[1] >= 0.0:
            
        rho = np.exp(-state[1]/10000)  # simple exponential atmosphere model based on altitude (z)
        
        qinf = 0.5 * rho * v**2  # dynamic pressure based on velocity magnitude
        
        D = qinf * vehicle.S* (aero.cd0 + aero.cd_alpha * np.deg2rad(2.5) ** 2)
        
        # simple thrust profile (ramps up to Tramp, holds for Thold sec, then ramps down)
        if time < thrust.Tramp:
            Thrust = thrust.T * time / thrust.Tramp
        elif time < thrust.Thold:
            Thrust = thrust.T
        elif time < thrust.Thold + thrust.Tramp:
            Thrust = thrust.T * (1.0 - (time - thrust.Thold) / thrust.Tramp)
        else:
            Thrust = 0.0
        
        vdot = Thrust / vehicle.m - D / vehicle.m - 9.81 * np.sin(gamma)
        gammadot = -9.81 * np.cos(gamma) / v
    
                
        v = v + vdot * dt
        gamma = gamma + gammadot * dt
        state[2] = v * np.cos(gamma)
        state[3] = v * np.sin(gamma)
        
        state[0] = state[0] + state[2] * dt
        state[1] = state[1] + state[3] * dt

        time += dt
        
        state_log.append(state.copy())
        time_log.append(time)
        
        # scalar engagement stats at this step, for the stopping condition
        step = len(time_log) - 1
        tidx = min(step, target_traj.shape[0] - 1)
        prel = target_traj[tidx, 0:2] - state[0:2]
        vrel = target_traj[tidx, 2:4] - state[2:4]
        rng = np.linalg.norm(prel)
        range_rate = np.dot(prel, vrel) / max(rng, 1e-9)
        
        
        if range_rate >= 0:
            break
        
       
    
    state_logging = np.array(state_log).T  # shape (4, N)
    time_logging = np.array(time_log)
    stats = get_stats(state_logging, target_traj)

    return state_logging, time_logging, stats
    
    
    
    
    #------------------------- helper functions to support prelaunch solution ----------------------------------
  
def get_stats(state_logging, target_traj):
     
     # state logging: x, z, vx, vz   RENU
     # target trajectory: x, z, vx, vz RENU
     
     x = state_logging[0, :]
     z = state_logging[1, :]
     vx = state_logging[2, :]
     vz = state_logging[3, :]
     
     speed = np.sqrt(vx**2 + vz**2)
     
     # align target trajectory with vehicle timesteps (clamp if vehicle flies longer)
     n = state_logging.shape[1]
     idx = np.minimum(np.arange(n), target_traj.shape[0] - 1)
     
     # find the index of maximum speed (burnout)
     idx_bo = np.argmax(speed)
     speed_bo = speed[idx_bo]
     
     
     x_target = target_traj[idx, 0]
     z_target = target_traj[idx, 1]
     vx_target = target_traj[idx, 2]
     vz_target = target_traj[idx, 3]
     
     speed_percent_loss = (speed_bo - speed) / np.maximum(speed_bo, 1e-9) * 100
     
     # estimate time target tgo to asset (aka to end of trajectory)
     index_max = target_traj.shape[0] - 1
     
     index_percent = 100 - ((index_max - idx) / np.maximum(index_max, 1e-9)) * 100 # fraction of trajectory remaining
     
     
          
     
     target_vel =np.array([vx_target, vz_target])
     vel = np.array([vx, vz])
     
    
     target_pos = np.array([x_target, z_target])
     pos = np.array([x, z])
     
     prel = target_pos - pos
     vrel = target_vel - vel
     
     
     dx = x - x_target
     dz = z - z_target
     
     range_ = np.linalg.norm(prel, axis=0)  # per-timestep range
     range_rate = np.sum(prel * vrel, axis=0) / np.maximum(range_, 1e-9)  # per-timestep dot product
     
     speed = np.sqrt(vx**2 + vz**2)
     target_speed = np.sqrt(vx_target**2 + vz_target**2)
     speed_ratio = speed / np.maximum(target_speed, 1e-9)
     
     
     gamma_vehicle = np.arctan2(vz, vx)
     gamma_target = np.arctan2(vz_target, vx_target)
     
     crossing_angle_desired = np.deg2rad(-45)
     crossing_angle_error= gamma_vehicle - crossing_angle_desired

     

     return EngagementStats(
         range=range_,
         range_rate=range_rate,
         speed_ratio=speed_ratio,
         crossing_angle=crossing_angle_error,
         speed_percent_loss=speed_percent_loss,
         index_percent=index_percent,
         
     )
     
     
def get_cost(stats):
    
    w_POCA = 1
    w_speed = 0.3
    w_time = 0.7
    
    # index of closest approach (minimum range) — all stats sampled there
    i_poca = np.argmin(stats.range)

    POCA = stats.range[i_poca]
    speed_percent_loss = stats.speed_percent_loss[i_poca]
    index_percent = stats.index_percent[i_poca]
    
    cost_inner_loop = POCA**2
    cost_outer_loop =  w_speed * speed_percent_loss**2 + w_time * index_percent**2
    return cost_inner_loop, cost_outer_loop, i_poca





def get_best_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj):
    
    delta_angle = 0.01
    iter = 0
    iter_max = 20
    angle_diff = 1000
    
    while angle_diff > 1e-3 and iter < iter_max:  
        
        state_logging_0, time_logging_0, stats_0 =  vehicle_propogator(state, aero, thrust, vehicle, dt, launcher_angle, target_traj)
        state_logging_plus, time_logging_plus, stats_plus =  vehicle_propogator(state, aero, thrust, vehicle, dt, launcher_angle + delta_angle, target_traj)
        state_logging_minus, time_logging_minus, stats_minus =  vehicle_propogator(state, aero, thrust, vehicle, dt, launcher_angle - delta_angle, target_traj)

        cost_inner_0, cost_outer_0, i_poca_0 = get_cost(stats_0)
        cost_inner_plus, cost_outer_plus, i_poca_plus = get_cost(stats_plus)
        cost_inner_minus, cost_outer_minus, i_poca_minus = get_cost(stats_minus)
        
       
        
        d1 = (cost_inner_plus - cost_inner_minus) / (2 * delta_angle)
        d2 = (cost_inner_plus - 2 * cost_inner_0 + cost_inner_minus) / (delta_angle**2)

        if d2 != 0:
            m = - d1 / d2
            angle_diff = abs(d1 / d2)
        else:
            m = 0
            angle_diff = 0

        launcher_angle = launcher_angle + m
        
        iter = iter + 1



    return state_logging_0, time_logging_0, stats_0, cost_inner_0, cost_outer_0, i_poca_0, launcher_angle


def get_shaping_coefficents(state_logging, time_logging):
        # fits piecewise cubic polynomials to the trajectory for shaping,
        # broken into three time segments: boost [0, 5], midcourse [5, tof/2], terminal [tof/2, tof]
        
        x_coord = state_logging[0, :]
        z_coord = state_logging[1, :]
        t_coord = time_logging
        
        tof = t_coord[-1]
        t_breaks = [0.0, 5.0, tof / 2.0, tof]
        
        coeff_segments = []
        for t0, t1 in zip(t_breaks[:-1], t_breaks[1:]):
            mask = (t_coord >= t0) & (t_coord <= t1)
            n = np.count_nonzero(mask)
            deg = min(3, n - 1)   # drop degree if a segment has too few points
            if deg < 0:
                continue
            coeffs_x = np.polyfit(t_coord[mask], x_coord[mask], deg)
            coeffs_z = np.polyfit(t_coord[mask], z_coord[mask], deg)
            coeff_segments.append((coeffs_x, coeffs_z, t0, t1))

        return coeff_segments
    
    
def get_launch_time(target_traj, dt, sim_clock, time_to_asset_desired): 
        
    tgo = target_traj.shape[0] * dt
        
    launch_time = sim_clock + tgo - time_to_asset_desired
    return launch_time
        
        
    

def get_prelaunch_solution(state, aero, thrust, vehicle, dt, sim_clock, launcher_angle_guess, target_traj):
    
    tgo_max = target_traj.shape[0] * dt # estimate of time to go until the target reaches asset
    

    iter = 0
    iter_max = 10
    tgo_diff = 1000
    delta_tgo = 2
    tgo_min = 10
    POCA_THRESHOLD = 1000  # define a threshold for acceptable POCA
    cost_outer_best = np.inf
    
    
    # create a list of candidate launch times (tgo_max - 5, tgo_max - 10 ... tgo_min))
    # ex : launchng when target is x seconds away from the asset
    candidate_launch_times = np.arange(tgo_max - 5, tgo_min, -5)
    
    
    for launch_time_candidate in candidate_launch_times:
        
        # find the launch angle that minimizes POCA for a given candidate launch time
        
        target_traj_candidate = get_trimmed_traj(target_traj, dt, launch_time_candidate)
        
        state_logging, time_logging, stats, cost_inner, cost_outer, i_poca, launcher_angle_optimal = get_best_solution(state, aero, thrust, vehicle, dt, launcher_angle_guess, target_traj_candidate)
        
        POCA = np.sqrt(cost_inner)
        
        
        # only if POCA is within the acceptable threshold, we consider this launch time candidate
        # and if the outer cost is better than the best found so far
        if POCA <= POCA_THRESHOLD and cost_outer < cost_outer_best:
            state_logging_best = state_logging
            time_logging_best = time_logging
            stats_best = stats
            cost_inner_best = cost_inner
            cost_outer_best = cost_outer
            i_poca_best = i_poca
            launcher_angle_best = launcher_angle_optimal
            launch_time_best = launch_time_candidate
            
            
    # use the launch time and launcher angle to warm start the optimization
    
    if cost_outer_best == np.inf:
        raise RuntimeError("No launch time candidate met the POCA threshold")
    
    launch_time = launch_time_best
    launcher_angle = launcher_angle_best
    time_to_asset_desired = launch_time_best
    
    while tgo_diff > 0.01 and iter < iter_max:
        
        target_traj_0 = get_trimmed_traj(target_traj, dt, launch_time)
        target_traj_plus  = get_trimmed_traj(target_traj, dt, launch_time + delta_tgo)
        target_traj_minus = get_trimmed_traj(target_traj, dt, launch_time - delta_tgo) 
        
    
        state_logging_0, time_logging_0, stats_0, cost_inner_0, cost_outer_0, i_poca_0, launcher_angle = get_best_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj_0)
        state_logging_plus, time_logging_plus, stats_plus, cost_inner_plus, cost_outer_plus, i_poca_plus, launcher_angle = get_best_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj_plus)
        state_logging_minus, time_logging_minus, stats_minus, cost_inner_minus, cost_outer_minus, i_poca_minus, launcher_angle = get_best_solution(state, aero, thrust, vehicle, dt, launcher_angle, target_traj_minus)
    
        d1 = (cost_outer_plus - cost_outer_minus) / (2 * delta_tgo)
        d2 = (cost_outer_plus - 2 * cost_outer_0 + cost_outer_minus) / (delta_tgo**2)

        if d2 != 0:
            dtgo_update = - d1 / d2
            tgo_diff = abs(d1 / d2)
          
        else:
           
            dtgo_update = 0
            tgo_diff = 0

        time_to_asset_desired = time_to_asset_desired + dtgo_update
        launch_time = launch_time + dtgo_update
    
        iter = iter + 1
        
    x_poca = state_logging_0[0,i_poca_0]
    z_poca = state_logging_0[1, i_poca_0]
    tof_poca = time_logging_0[i_poca_0]
    
    pred_pip = np.array([x_poca, z_poca])
    pred_tof = tof_poca
    
    coeff_segments = get_shaping_coefficents(state_logging_0, time_logging_0)
    
    launch_time = get_launch_time(target_traj, dt, sim_clock, time_to_asset_desired)
    

    return state_logging_0, time_logging_0, stats_0, cost_inner_0, cost_outer_0, launcher_angle, time_to_asset_desired, target_traj_0, pred_pip, pred_tof, coeff_segments, launch_time








