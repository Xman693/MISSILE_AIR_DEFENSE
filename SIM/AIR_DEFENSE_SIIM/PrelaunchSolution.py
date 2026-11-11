
from types import SimpleNamespace
import numpy as np

from Paramaters import *
from TargetEstimationExtrap import TargetEstimationExtrap

sim = SimParams()
asset = AssetParams()
ground = GroundParams()
aero = AeroParams()
thrust = ThrustParams()
missile = MissileParams()
TSEE = TargetEstimationExtrap()
launcher = LauncherParams()
env = Enviroment()
thrust = ThrustParams()

class PrelaunchSolution:

    def __init__(self):
        self.sim = sim
        self.asset = asset
        self.ground = ground
        self.missile = missile
        self.thrust = thrust
        self.aero = aero
        self.TSEE = TSEE
        self.launcher = launcher
        self.env = env
        self.alpha_tau = 0.5  # short-period (pitch) time constant for the weathercock alpha model, seconds
        self.thrust = thrust
        
        
        
    def get_prelaunch_solution(self, target_traj, validity_time):
        
        
        ## target traj is a collection of numspaces
        tgo_max = (len(target_traj) - 1) * self.ground.extrap_time_step # estimate of time to go until the target reaches asset
        

        # --- initialize optimization variables
        iter = 0
        iter_max = 10
        tgo_diff = 1000
        delta_tgo = 2
        tgo_min = 10
    
        # define a threshold for acceptable POCA (can be higher than required to not eliminate potential candidates)
        # Also there is no terminal guidance considered in this prelaunch solution, so miss distance at POCA might be larger than the actual intercept distance
        poca_threshold = 1000  
        cost_secondary_opt = np.inf
        launcher_angle_guess = np.deg2rad(45)
        
        
        # create a list of candidate launch times (tgo_max - 5, tgo_max - 10 ... tgo_min))
        # ex : launching when target is x seconds away from the asset
        candidate_launch_times = np.arange(tgo_max - 5, tgo_min, -5)
        
        
        for launch_time_candidate in candidate_launch_times:
            
            # find the launch angle that minimizes POCA for a given candidate launch time
            
            target_traj_candidate = self.TSEE.get_trimmed_traj(target_traj, launch_time_candidate)
            
            state_logging, time_logging, poca_stats, cost, launcher_angle_optimal = self.get_optimal_trajectory(launcher_angle_guess, target_traj_candidate, tgo_max)
            
            poca = poca_stats.miss_distance
            
            
            
            
            # only if POCA is within the acceptable threshold, we consider this launch time candidate
            # and if the outer cost is better than the best found so far
            
            if poca <= poca_threshold and cost.secondary < cost_secondary_opt:
                state_logging_cand = state_logging
                time_logging_cand = time_logging
                stats_best_cand = poca_stats
                cost_primary_opt = cost.primary
                cost_secondary_opt = cost.secondary
                launcher_angle_opt = launcher_angle_optimal
                launch_time_best = launch_time_candidate #  == target tgo to asset at missile launch
                
                
        # use the launch time and launcher angle to warm start the optimization
        
        if cost_secondary_opt == np.inf:
            raise RuntimeError("No launch time candidate met the POCA threshold")
        
        
        ## initialize the launch time and angles for the iterative optimization
        launch_time = launch_time_best
        launch_angle = launcher_angle_opt
        launch_angle_plus = launch_angle
        launch_angle_minus = launch_angle
    
        
        while tgo_diff > 0.01 and iter < iter_max:
            
            target_traj_0 = self.TSEE.get_trimmed_traj(target_traj, launch_time)
            target_traj_plus  = self.TSEE.get_trimmed_traj(target_traj, launch_time + delta_tgo)
            target_traj_minus = self.TSEE.get_trimmed_traj(target_traj, launch_time - delta_tgo) 
            
            state_logging_0, time_logging_0, poca_stats_0, cost_0, launch_angle= self.get_optimal_trajectory(launch_angle, target_traj_0, tgo_max)
            state_logging_plus, time_logging_plus, poca_stats_plus, cost_plus, launch_angle_plus = self.get_optimal_trajectory(launch_angle_plus, target_traj_plus, tgo_max)
            state_logging_minus, time_logging_minus, poca_stats_minus, cost_minus, launch_angle_minus = self.get_optimal_trajectory(launch_angle_minus, target_traj_minus, tgo_max)
        
            # numerical derivative of the secondary cost with respect to launch time
            d1 = (cost_plus.secondary - cost_minus.secondary) / (2 * delta_tgo)
            d2 = (cost_plus.secondary - 2 * cost.secondary + cost_minus.secondary) / (delta_tgo**2)

            # ----  Newton Raphson update for launch time -----
            if d2 != 0:
                dtgo_update = - d1 / d2
                tgo_diff = abs(d1 / d2)
            
            else:
            
                dtgo_update = 0
                tgo_diff = 0

            
            launch_time = launch_time + dtgo_update
            ## launch time is the target tgo to asset at missile launch not the absolute simulation time
            iter = iter + 1
          
          
        
        # Grab inital pip, pred tof,   
        x_poca = state_logging_0[-1].x
        z_poca = state_logging_0[-1].z
        tof_poca = time_logging_0[-1].time
        
        pred_pip = SimpleNamespace(x=x_poca, z=z_poca)
        pred_tof = tof_poca
        
        coeff_segments = self.get_shaping_coefficents(state_logging_0, time_logging_0)
        
        
        # use the full target traj here 
        simulation_time_at_launch = self.get_launch_time(target_traj, validity_time, launch_time)
        
        prelaunch_solution = SimpleNamespace(state_logging=state_logging_0, time_logging=time_logging_0, poca_stats=poca_stats_0, cost=cost_0, launch_angle=launch_angle, target_traj=target_traj_0, pred_pip=pred_pip, pred_tof=pred_tof, coeff_segments=coeff_segments, target_tgo_at_launch=launch_time, time_at_launch=simulation_time_at_launch)
        
    
    

        return prelaunch_solution


    def get_optimal_trajectory(self, launch_angle, target_traj, tgo_max): 
        
     
        # ------- initialization for optimization ----------------------------
        delta_angle = 0.01 # small change in launch angle for numerical derivative
        iter = 0
        iter_max = 20
        angle_diff = 1000
    
        while angle_diff > 1e-3 and iter < iter_max:  
            
            state_logging_0, time_logging_0, poca_stats_0 =  self.missile_propogator(launch_angle, target_traj, tgo_max)
            state_logging_plus, time_logging_plus, poca_stats_plus =  self.missile_propogator(launch_angle + delta_angle, target_traj, tgo_max)
            state_logging_minus, time_logging_minus, poca_stats_minus =  self.missile_propogator(launch_angle - delta_angle, target_traj, tgo_max)

            cost_0= self.get_cost(poca_stats_0)
            cost_plus = self.get_cost(poca_stats_plus)
            cost_minus = self.get_cost(poca_stats_minus)
            
            # primary is miss distance, # secondary is energy/time based
            # numerical derivatives 
            d1 = (cost_0.primary - cost_minus.primary) / (2 * delta_angle)
            d2 = (cost_plus.primary - 2 * cost_0.primary + cost_minus.primary) / (delta_angle**2)

            # Newton-Raphson update for launch angle
            if d2 != 0:
                m = - d1 / d2
                angle_diff = abs(d1 / d2)
            else:
                m = 0
                angle_diff = 0

            launch_angle = launch_angle + m
            
            iter = iter + 1



        return state_logging_0, time_logging_0, poca_stats_0, cost_0, launch_angle
            
    
    def missile_propogator(self, launch_angle, target_traj, tgo_max):
        
        ## simulates the missile flyout to find the trajectory and relevant statistics based on the launch angle and target trajectory
        
        time = 0.0
        iter = 0
            
        
        # initial state for vehicle off rail
            
        

        v = 30 # 
        
        missile_state = SimpleNamespace(x = self.launcher.launcher_emplacement_x,
                                            z = self.launcher.launcher_emplacement_z,
                                            vx = v * np.cos(launch_angle),
                                            vz = -v * np.sin(launch_angle),
                                            alpha = np.deg2rad(5))
            
        
        # initial gamma 
        gamma = launch_angle
            
        state_log = [SimpleNamespace(**vars(missile_state))]
        time_log = [SimpleNamespace(time=time)]
     
            
            
        while missile_state.z <= 0.0:
            
                time+= self.ground.extrap_time_step
                iter += 1
                
                #  Compute Aero Forces and Thrust 
                
                T = self.get_thrust(time)
                AeroForces = self.get_aero_forces(missile_state)
                
                D = AeroForces.Drag
                L = AeroForces.Lift
                
                
                
                vdot = T*np.cos(missile_state.alpha)/self.missile.m -self.env.gravity * np.sin(gamma) - D/self.missile.m
                gammadot =  (T*np.sin(missile_state.alpha) + L)/(self.missile.m*v) - self.env.gravity * np.cos(gamma) / v
            
                # weathercock approximation of the short-period mode: alpha is stable and tracks
                # its trim value (lift balancing the normal gravity component) with time constant tau
                alpha_trim = self.missile.m * self.env.gravity * np.cos(gamma) / max(AeroForces.qinf * self.missile.S * self.aero.CL_alpha, 1e-9)
                alpha_trim = np.clip(alpha_trim, -np.deg2rad(20), np.deg2rad(20))
                alphadot = (alpha_trim - missile_state.alpha) / self.alpha_tau
                v = v + vdot * self.ground.extrap_time_step
                gamma = gamma + gammadot * self.ground.extrap_time_step
                missile_state.vx = v * np.cos(gamma)
                missile_state.vz = -v * np.sin(gamma)
                
                missile_state.alpha = missile_state.alpha + alphadot * self.ground.extrap_time_step
                missile_state.x = missile_state.x + missile_state.vx * self.ground.extrap_time_step
                missile_state.z = missile_state.z + missile_state.vz * self.ground.extrap_time_step
        
               
    
                
                if iter >= len(target_traj):
                    # target window ended before POCA / impact; stop at the last known target state
                    break
                
                target_state = target_traj[iter]
                
                state_log.append(SimpleNamespace(**vars(missile_state)))
                time_log.append(SimpleNamespace(time=time))
                
                # scalar engagement stats at this step, for the stopping condition
                
                prel = np.array([target_state.x, target_state.z]) - np.array([missile_state.x, missile_state.z])
                vrel = np.array([target_state.vx, target_state.vz]) - np.array([missile_state.vx, missile_state.vz])
                rng = np.linalg.norm(prel)
                range_rate = np.dot(prel, vrel) / max(rng, 1e-9)
                
                # gets poca stats 
                
                
                
                if range_rate >= 0:
                    poca_stats = self.get_poca_stats(state_log, target_traj, tgo_max)
                    break
                
            
        poca_stats = self.get_poca_stats(state_log, target_traj, tgo_max)        
       
               
            
                
        
        return state_log, time_log, poca_stats
        
    def get_poca_stats(self, state_log, target_traj, tgo_max):
    # state logging: x, z, vx, vz   RENU
         # target trajectory: x, z, vx, vz RENU
         
         x = np.array([s.x for s in state_log])
         z = np.array([s.z for s in state_log])
         vx = np.array([s.vx for s in state_log])
         vz = np.array([s.vz for s in state_log])
         
         
         ## this is because the state log ends at poca due to the stopping condition in the simulation loop
         # clamp to the target trajectory length in case the state log outran the target window
         poca_idx = min(len(x) - 1, len(target_traj) - 1)
         
         ## calculates the speed at all iters
         speed = np.sqrt(vx**2 + vz**2)
         
         
         ## -------------------- Energy Loss At POCA ---------------------------
         # find the index of maximum speed (which is basically burnout (but with no thrust/drag bo is at min height))
         
         # 11/8 ** This is not useful for a pure ballistic case since energy loss is 0
         # but here when drag/thrust is modeled later
         idx_bo = np.argmax(speed)
         speed_bo = speed[idx_bo]
         
         h_bo = abs(z[idx_bo]) ## z will be negative in the RENU frame
         h_poca = abs(z[poca_idx])
         
         ## specific mechanical energy at burnout
         energy_bo = 0.5 * speed_bo**2 + self.env.gravity * h_bo 
         
         ## specific mechanical energy at poca 
         energy_poca = 0.5 * speed[poca_idx]**2 + self.env.gravity * h_poca
         
        
         energy_percent_loss = (energy_bo - energy_poca) / np.maximum(energy_bo, 1e-9) * 100
         
         ## --------------------- Miss Distance ---------------------------
         
         x_target = np.array([s.x for s in target_traj])
         z_target = np.array([s.z for s in target_traj])

         dx = x[poca_idx] - x_target[poca_idx]
         dz = z[poca_idx] - z_target[poca_idx]
         miss_distance = np.sqrt(dx**2 + dz**2)
         
         ## ----- Target TGO at Intercept -----------------
         
            # Find how long target TGO to asset at POCA (maximize this!)
         target_tgo_at_missile_launch = (len(target_traj) - 1)*self.ground.extrap_time_step
     
        
        # want to minimize launch time and tgo_max (basically launch as early as you can )
         tgo_percent = abs(tgo_max - target_tgo_at_missile_launch) / np.maximum(tgo_max, 1e-9) * 100
        
         
    
         PocaStats = SimpleNamespace(
             miss_distance=miss_distance,
             energy_percent_loss=energy_percent_loss,
             tgo_percent=tgo_percent
         )
         return PocaStats
         
         
    def get_cost(self, stats):
        
        ## define cost weights ---------
        
        w_miss_distance = 1.0
        
        w_energy_percent_loss = 0.5
        w_tgo_percent = 0.5
        
        cost_primary = w_miss_distance * stats.miss_distance**2
        cost_secondary = (w_energy_percent_loss * stats.energy_percent_loss**2 +
                          w_tgo_percent * stats.tgo_percent**2)

        cost = SimpleNamespace(
            primary=cost_primary,
            secondary=cost_secondary,
            energy_loss = stats.energy_percent_loss,
            miss_distance = stats.miss_distance,
            tgo_percent = stats.tgo_percent
        )

        return cost
    
    
    def get_shaping_coefficents(self, state_logging, time_logging):
        # fits piecewise cubic polynomials to the trajectory for shaping,
        # broken into three time segments: boost [0, 5], midcourse [5, tof/2], terminal [tof/2, tof]
        
        # **** Need to make the interpolation smooth between the segments
        
        x_coord = np.array([s.x for s in state_logging])  # there are the x and z coordinates of the prelaunch traj
        z_coord = np.array([s.z for s in state_logging])
        t_coord = np.array([s.time for s in time_logging])
        tof = time_logging[-1].time
        
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
    
    
    
    def get_launch_time(self, target_traj, validity_time, target_tgo_criteria): 
        
        ## checks to see if launch should occur based on target trajectory and desired time to asset
        
        
        ## current estimated time to asset
        target_tgo_current = (len(target_traj) - 1)*self.ground.extrap_time_step
        
        time_at_launch = validity_time + target_tgo_current - target_tgo_criteria
        
        
        
        return time_at_launch
    
    
    def get_thrust(self, time):
        
    # simple thrust profile (ramps up to Tramp, holds for Thold sec, then ramps down)
        if time < self.thrust.Tramp:
            Thrust = self.thrust.T * time / self.thrust.Tramp
        elif time < self.thrust.Thold:
            Thrust = self.thrust.T
        elif time < self.thrust.Thold + self.thrust.Tramp:
            Thrust = self.thrust.T * (1.0 - (time - self.thrust.Thold) / self.thrust.Tramp)
        else:
            Thrust = 0.0

        return Thrust
    
    
                        
        
        
        
        
    
    def get_aero_forces(self, missile_state): 
        
        rho = 1.225*np.exp(missile_state.z/8500)  # NED: altitude = -z, so density decays as z goes negative
        v = np.linalg.norm([missile_state.vx, missile_state.vz])
        
        qinf = 0.5 * rho * v**2
        S = self.missile.S 
        
        CL_alpha = self.aero.CL_alpha
        CD_alpha = self.aero.CD_alpha
        
        alpha = missile_state.alpha 
        
        L = qinf * S * CL_alpha * alpha
        D = qinf * S * CD_alpha * alpha**2
        
        aeroforces = SimpleNamespace(Drag = D, Lift = L, qinf = qinf)
        
        return aeroforces
        
    
    
    
    
    
  
    
    