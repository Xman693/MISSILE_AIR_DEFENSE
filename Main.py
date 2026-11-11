# exectutes simulation


import numpy as np
import csv
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from numpy import log

import Radar
import GroundSoftware
# import MissileTruth
# import MissileOnboard
# import Launchers
from Paramaters import SimParams, RadarParams, GroundParams, AssetParams, AssetParams
import TargetTruth


CSV_FILE = "sim_log.csv"

# clear the log from any previous run
open(CSV_FILE, "w").close()

def log(message):
    """Print to console and append the message to the CSV log."""
   
    with open(CSV_FILE, "a", newline="") as f:
        csv.writer(f).writerow([clock, message])

ground = GroundSoftware.GroundSoftware()
# launcher = Launchers.get_launcher()
sim = SimParams()
radar_params = RadarParams()
ground_params = GroundParams()
asset_params = AssetParams()
# missile_truth = MissileTruth.MissileTruth()
# missile_onboard = MissileOnboard.MissileOnboard()
radar = Radar.Radar()
asset = AssetParams()





# missile_true_state = None
# missile_onboard_state = None

# ## --------- Initialization --------- ##
clock = 0.0
radar_clock = 0.0
TSEE_clock = 0
# iter = 0.0
# Missile_Hit = False
# Missile_Self_Destruct = False
# missile_away = False
# missile_tail_out_of_tube = False
# uplink_simulated_delay = 0
# uplink_data = None
# prelaunch_paramaters = None
# launch = None
# downlink_decoded = None
# uplink = None
# tof = 0
# tof_onboard = 0
radar_measurements = None
prelaunch_paramaters = None
target_extrapolation = None
launch_cmd_issued = False




## grab true full target traj extrapolated to asset ##
target_traj = TargetTruth.target.target_traj

beam_hist_log = np.zeros(len(target_traj))






# while Missile_Hit == False and Missile_Self_Destruct == False:
for i in range(len(target_traj)):
#     # extract target state from real trajectory
    target_state = target_traj[i]


  #------- execute radar logic -----------
    if radar_clock > radar_params.PRI:
      radar_measurements, status = radar.execute_radar_logic(target_state)
      
      if status.target_detected:
        log(f"Target detected by radar. Time = {clock:.2f}")
        log(f"Detection Range: {radar_measurements.range}, Range Rate: {radar_measurements.range_rate}, LOS: {np.rad2deg(radar_measurements.los)}, Beam Angle: {np.rad2deg(radar_measurements.beam_angle) }")
        radar_clock = 0
              
    radar_clock+= sim.sim_time_step
    


#     # send radar measuremnts to ground software

#     #  ---- Missile Air Defense Software ---- (houses all of the ground software)

    if radar_measurements is not None:
        
        # 11/7 : skip target estimation for now,
        
        
        #estimated_target_state = ground.TSEE.target_estimation(radar_measurements)
        #if TSEE_clock > ground_params.TSEE_period:
            
            ## 11/7 Update this so the target state is estimated at high rate 
                # even when measurement is not available
            ## 11/7 figure out better way to determine when a new extrapolation is needed
                # possilby based on if 
        if target_extrapolation == None:
            target_extrapolation = ground.TSEE.target_extrapolation(target_state)
            validity_time = clock
            log(f"Target estimation and extrapolation completed. Time = {clock:.2f}")
            #TSEE_clock = 0
            TSEE_clock += sim.sim_time_step


   # if not launcher.launchcmd and estimated_target_state is not None:
        if prelaunch_paramaters is None and target_extrapolation is not None:
            prelaunch_paramaters = ground.prelaunch_paramaters(target_extrapolation, validity_time)
            log(f"Prelaunch solution computed. Time = {clock:.2f}")
           
        
       

        if prelaunch_paramaters is not None and clock >= prelaunch_paramaters.time_at_launch and not launch_cmd_issued:
            
            log(f"Launch command issued. Time = {clock:.2f}")
            log(f"Prelaunch Parameters. PIP: {prelaunch_paramaters.pred_pip}, Pred TOF: {prelaunch_paramaters.pred_tof}, Launch Angle: {np.rad2deg(prelaunch_paramaters.launch_angle)} , Time at Launch: {prelaunch_paramaters.time_at_launch}, Miss Distance: {prelaunch_paramaters.poca_stats.miss_distance}"   )
            launch_cmd_issued = True
        
#     if downlink_decoded is not None:
#         log(f"Downlink decoded. Time = {clock:.2f}")

    
#     if downlink_decoded.status != "INITIAL_TURN":
#         # dont start sending uplink until the missile has completed its initial turn
        
#         # for now, just practice simple back and forth logic based on downlink status
#         uplink = ground.generate_uplink_data(downlink_decoded)
#         log(f"Uplink data generated. Time = {clock:.2f}")
        
#     if uplink is not None:
#         uplink_simulated_delay += sim.dt
#         if uplink_simulated_delay >= sim.UPLINK_DELAY:
#             # uplink data has reached the missile
#             uplink_simulated_delay = 0
#             log(f"Uplink data recieived by missile. Time = {clock:.2f}")
#             uplink_data = uplink
#             uplink = None
            
        


#     # send launch command to missile and propogate missile 
#     if launch.launchcmd and launch is not None: 
        
#         if missile_away == False:
            
#             # only do the initializae is missile has not came out of tube
            
        
        
#             log(f"Launch command issued. Initializing Missile IMU and Loading Engagement Solution. Time = {launch.launchtime:.2f}")
            
#             # initialize missile truth (could be different from onboard due to emplacement)
#             missile_true_state = missile_truth.initialize_missile_truth() # true x,z, theta
            
#             # initialize missile IMU
#             missile_onboard_state, status = missile_onboard.initialize_missile_onboard() # x, z, theta
            
#             # start missile truth propagation
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "MISSILE_AWAY", tof)
            
#             # start missile onboard propagation
#             missile_onboard_state, status, tof_onboard  = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "MISSILE_AWAY", tof_onboard, status)
            
#             # status here is not a downlink, its onboard missile status so it knows to transition between different stages of missile flight (e.g., tube clearance, initial turn, etc.)
            
#             # turn missile_away flag to true
#             missile_away = True
#         elif missile_tail_out_of_tube == False:
#             log(f"Missile tail has not yet left the tube. Waiting for it to clear before firing ACM's. Time = {clock:.2f}")
            
            
#             # missile truth propagation (no guidance, only thrust)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "MISSILE_TAIL_OUT_OF_TUBE", tof)
                    
#             # missile onboard propagation (no guidance, only thrust)
#             missile_onboard_state, status, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "MISSILE_TAIL_OUT_OF_TUBE", tof_onboard, status)
            
#             # check if missile has traveled length of tube 
#             if status.tube_cleared:
#                 missile_tail_out_of_tube = True
#                 log(f"Missile tail has cleared the tube. Time = {clock:.2f}")
                
#         elif initial_turn_completed == False: 
            
#             # missile truth propagation (initial turn guidance)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "INITIAL_TURN", tof)
                            
#             # missile onboard propagation (initial turn guidance )
#             missile_onboard_state, status, downlink, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "INITIAL_TURN", tof_onboard)
            
#             if status.initial_turn_completed:
#                 initial_turn_completed = True
#                 log(f"Initial turn completed. Time = {clock:.2f}")
#         elif midcourse_completed == False:
            
#             # missile truth propagation (midcourse guidance)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "MIDCOURSE", tof)
                                    
#             # missile onboard propagation (midcourse guidance )
#             missile_onboard_state, status, downlink, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "MIDCOURSE", tof_onboard, status)
            
#             if downlink.self_destruct:
#                 log(f"Self destruct initiated. Time = {clock:.2f}")
#                 Missile_Self_Destruct = True
#                 break
#             elif status.midcourse_completed:
#                 midcourse_completed = True
#                 log(f"Midcourse completed. Time = {clock:.2f}")
                
#         elif target_acquired_via_seeker == False:
            
#             # hold gamma angle unti target is acquired via seeker
            
#             # missile truth propagation (midcourse guidance)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "ACQURING_TARGET", tof)
                                            
#             # missile onboard propagation (midcourse guidance )
#             missile_onboard_state, status, downlink, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "ACQURING_TARGET", tof_onboard, status)
            
#             if downlink.self_destruct:
#                 log(f"Self destruct initiated. Time = {clock:.2f}")
#                 Missile_Self_Destruct = True
#                 break
#             elif status.target_acquired:
#                 target_acquired_via_seeker = True
#                 log(f"Seeker has acquired target. Starting homing guidance. Time = {clock:.2f}")
                
#         elif homing_guidance_completed == False:
            
#             # missile truth propagation (homing guidance)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "HOMING", tof)
                                            
#             # missile onboard propagation (homing guidance )
#             missile_onboard_state, status, downlink, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "HOMING", tof_onboard, status)
#             if downlink.self_destruct:
#                 log(f"Self destruct initiated. Time = {clock:.2f}")
#                 Missile_Self_Destruct = True
#                 break
#             elif status.homing_guidance_completed:
#                 homing_guidance_completed = True
#                 log(f"Homing guidance completed. Final Lunge Starting. Time = {clock:.2f}")
                
#         elif final_lunge_completed == False:
            
#             # missile truth propagation (final lunge)
#             missile_true_state, tof = missile_truth.propogate_missile_truth(prelaunch_paramaters, uplink_data, missile_true_state, "FINAL_LUNGE", tof)
                                            
#             # missile onboard propagation (final lunge)
#             missile_onboard_state, status, downlink, tof_onboard = missile_onboard.propogate_onboard_missile(prelaunch_paramaters, uplink_data, missile_onboard_state, "FINAL_LUNGE", tof_onboard, status)
            
        
#             if status.final_range <= 0.5:
#                 final_lunge_completed = True
#                 Missile_Hit = True
#                 log(f"Final lunge completed. Missile has reached the target. Miss: {status.final_range:.2f} Time = {clock:.2f}")
#             elif downlink.self_destruct:
#             # can self destruct if ground says to in uplink or if missile passes target
#                 log(f"Self destruct initiated. Miss Distance = {status.final_range:.2f} Time = {clock:.2f}")
#                 Missile_Self_Destruct = True
#                 break
            
                
        
#     elif not launch.launchcmd:
#         log(f"Launch command not issued. Time = {clock:.2f}")
        
        
#     if downlink is not None:
#         simulated_downlink_delay += sim.dt
#         if simulated_downlink_delay >= sim.downlink_delay:
#             downlink_decoded = ground.decode_downlink(downlink)
#             simulated_downlink_delay = 0
#             downlink = None 

    if radar_measurements is not None:
        beam_hist_log[i] = radar_measurements.beam_angle
    
    



    clock += sim.sim_time_step
    i+= 1
    
# animate the beam angle + radar_face_pitch angle +- beamwidth cone
radar_face_pitch = radar.radar.pitch_angle
beamwidth = radar.radar.beamwidth

# radar emplacement in NED (radar sits at the sim origin)
radar_x, radar_z = 0.0, 0.0

traj_x = np.array([state.x for state in target_traj])
traj_z = np.array([state.z for state in target_traj])
beam_length = 1.05 * np.max(np.hypot(traj_x - radar_x, traj_z - radar_z))

# missile flyout from the prelaunch solution (time_logging is seconds since launch)
if prelaunch_paramaters is not None:
    msl_x = np.array([s.x for s in prelaunch_paramaters.state_logging])
    msl_z = np.array([s.z for s in prelaunch_paramaters.state_logging])
    msl_t = np.array([t.time for t in prelaunch_paramaters.time_logging])
    time_at_launch = prelaunch_paramaters.time_at_launch
else:
    msl_x = msl_z = msl_t = None
    time_at_launch = None

def ray_endpoint(angle):
    # NED: z is down-positive, so a beam angle measured positive-up gives z = -sin(angle)
    return (radar_x + beam_length * np.cos(angle),
            radar_z - beam_length * np.sin(angle))

fig, ax = plt.subplots()
ax.plot(traj_x, traj_z, color="gray", linewidth=1, label="Target trajectory")
ax.plot(radar_x, radar_z, "ks", markersize=8, label="Radar emplacement")
# asset location shown as a transparent red vertical column spanning asset.x +- radius
ax.axvspan(asset_params.x - asset_params.radius, asset_params.x + asset_params.radius,
           color="red", alpha=0.15, label="Asset")

center_line, = ax.plot([], [], "b--", linewidth=1, label="Beam center")
lower_line, = ax.plot([], [], "b-", linewidth=1, label="Beam bounds")
upper_line, = ax.plot([], [], "b-", linewidth=1)
beam_fill = ax.fill([radar_x], [radar_z], color="lightblue", alpha=0.6)[0]
target_marker, = ax.plot([], [], "rx", markersize=10, markeredgewidth=2, label="Target")

# missile flyout: planned path drawn faintly once launch cmd is issued,
# marker + trail advance along time_logging vs state_logging
missile_path, = ax.plot([], [], color="black", linewidth=1, alpha=0.4, label="Missile planned path")
missile_trail, = ax.plot([], [], color="black", linewidth=1.5, label="Missile flyout")
missile_marker, = ax.plot([], [], "k^", markersize=9, label="Missile")

def update(frame):
    frame_time = frame * sim.sim_time_step
    center_angle = radar_face_pitch + beam_hist_log[frame]
    upper_cone = center_angle + beamwidth / 2
    lower_cone = center_angle - beamwidth / 2

    x_c, z_c = ray_endpoint(center_angle)
    x_l, z_l = ray_endpoint(lower_cone)
    x_u, z_u = ray_endpoint(upper_cone)

    center_line.set_data([radar_x, x_c], [radar_z, z_c])
    lower_line.set_data([radar_x, x_l], [radar_z, z_l])
    upper_line.set_data([radar_x, x_u], [radar_z, z_u])
    beam_fill.set_xy([(radar_x, radar_z), (x_l, z_l), (x_u, z_u)])
    target_marker.set_data([traj_x[frame]], [traj_z[frame]])

    if launch_cmd_issued and msl_t is not None:
        missile_path.set_data(msl_x, msl_z)
        if frame_time >= time_at_launch:
            flyout_time = frame_time - time_at_launch
            idx = int(np.searchsorted(msl_t, flyout_time, side="right")) - 1
            idx = min(max(idx, 0), len(msl_t) - 1)
            missile_marker.set_data([msl_x[idx]], [msl_z[idx]])
            missile_trail.set_data(msl_x[:idx + 1], msl_z[:idx + 1])

    ax.set_title(f"Radar Beam - Time = {frame_time:.1f} s")
    return center_line, lower_line, upper_line, beam_fill, target_marker, missile_path, missile_trail, missile_marker

# step through the log so playback is faster than real time
frame_step = 10
# end the animation at the end of the missile flyout time log (if launched)
if launch_cmd_issued and msl_t is not None:
    end_frame = int(np.ceil((time_at_launch + msl_t[-1]) / sim.sim_time_step))
    end_frame = min(end_frame, len(beam_hist_log))
else:
    end_frame = len(beam_hist_log)

ani = FuncAnimation(fig, update, frames=range(0, end_frame, frame_step),
                    interval=20, blit=False, repeat=False)

ax.set_xlim(radar_x - 0.05 * beam_length, radar_x + 1.05 * beam_length)
# NED: z is down-positive; reversed limits put ground (z=0) at the bottom and altitude (negative z) up
z_top = min(traj_z.min(), msl_z.min() if msl_z is not None else 0.0, 0.0)
ax.set_ylim(0, 1.1 * z_top)
# equal aspect so 1 m on x looks the same as 1 m on z (no stretched scale)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("North x (m)")
ax.set_ylabel("Down z (m)")
ax.grid(True)

# save the animation next to this script so it can be opened from the sidebar
ANI_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "radar_animation.gif")
ani.save(ANI_FILE, writer=PillowWriter(fps=50))
print(f"Animation saved to {ANI_FILE}")

# missile flyout profiles from the prelaunch solution: velocity and angle of attack vs time
if prelaunch_paramaters is not None:
    flyout_t = np.array([t.time for t in prelaunch_paramaters.time_logging])
    flyout_v = np.array([np.hypot(s.vx, s.vz) for s in prelaunch_paramaters.state_logging])
    flyout_aoa = np.array([np.rad2deg(s.alpha) for s in prelaunch_paramaters.state_logging])

    fig2, (ax_v, ax_a) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax_v.plot(flyout_t, flyout_v, color="black")
    ax_v.set_ylabel("Velocity (m/s)")
    ax_v.grid(True)
    ax_a.plot(flyout_t, flyout_aoa, color="black")
    ax_a.set_ylabel("Angle of attack (deg)")
    ax_a.set_xlabel("Time since launch (s)")
    ax_a.grid(True)
    fig2.suptitle("Prelaunch missile flyout")

plt.show()


    
    
    

