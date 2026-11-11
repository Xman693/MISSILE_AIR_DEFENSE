from prelaunch_paramaters import get_prelaunch_paramaters
from Paramater import ThrustParams, AeroParams, VehicleParams, LauncherParams, SimParams, AssetParams, TargetParams, RadarParams, GroundParams
from Target_Traj_Extrap_Est import extrapolate_target_traj
from Launcher import get_launch_cmd
from Missile import get_acc_cmd, update_missile_state

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
   
   
# Initialize Parameters

aero = AeroParams()
thrust = ThrustParams()
vehicle = VehicleParams()
launcher = LauncherParams()
sim = SimParams()
asset = AssetParams()
target = TargetParams()
radar = RadarParams()
ground = GroundParams()




# load target trajectory from CSV (columns: x, z, vx, vz)
target_traj = np.loadtxt("target_traj.csv", delimiter=",", skiprows=1)  # skiprows=1 skips the header row

# initialize arrays
state_logging = np.zeros((4, sim.num_steps)) # x, z, vx, vz

# initialize clock
sim_clock = 0
tsee_clock = 0
prelaunch_paramaters_clock = 0 # computes a new solution every s seconds unless solution is locked in and then the target extrap 
# must result in the target being outside the pip area
iter = 0

# 30 m/s is the initial rail exit speed
missile_state = np.array([launcher.launcher_emplacement_x, launcher.launcher_emplacement_z, 30*np.cos(np.deg2rad(launcher.launcher_angle)), 30*np.sin(np.deg2rad(launcher.launcher_angle))]) # x, z, vx, vz

missile_state_log = [missile_state.copy()]
gamma_log = []
gamma_cmd_log = []
missile_tof_log = []
engagement_solution = "None"
missile_tof = 0

## add schduler
 
# if track is formed then run ground processijg


# create method that tracks if the target has entered the launch point or if needing to replan initial solutuon

# package necessary infromation for sending to launcher 


while iter < sim.num_steps:
        
    sim_clock += sim.dt_sim
            
            
        

    iter += 1
    
    # stop if we run past the end of the target truth data
    if iter >= target_traj.shape[0]:
        print(f"Target truth data exhausted at t = {sim_clock:.2f} s; ending sim.")
        break
    
        
        # Grab true target state at current iter
        
    true_target_state = target_traj[iter, :]  # x, z, vx, vz
    if iter == 1:
        target_extrap =  extrapolate_target_traj(true_target_state, ground.dt_extrap, asset)
        
    if engagement_solution == "None":
        best_state_logging, best_time_logging, best_stats, best_cost_inner, best_cost_outer, best_launcher_angle, best_time_to_asset_desired, best_target_traj, pred_pip, pred_tof, coeff_segments, best_launch_time = get_prelaunch_paramaters(missile_state, aero, thrust, vehicle, ground.dt_extrap, sim_clock, launcher.launcher_angle, target_extrap)
        engagement_solution = "Computed"  

    if engagement_solution == "Computed":
        launch_cmd = get_launch_cmd(best_launch_time, sim_clock)
        
    if launch_cmd == "Launch":
        engagement_solution = "Launched"
        
        # uses simple gammma pointing using waypoints
        normal_acc_cmd, gamma_cmd = get_acc_cmd(missile_state, missile_tof, coeff_segments)

        # log flight path angle vs command (while on the rail v = 0, so gamma is the launcher angle)
        v_now = np.linalg.norm(missile_state[2:4])
        gamma = np.arctan2(missile_state[3], missile_state[2]) if v_now > 1e-6 else np.deg2rad(best_launcher_angle)
        gamma_log.append(gamma)
        gamma_cmd_log.append(gamma_cmd)
        missile_tof_log.append(missile_tof)


        # propogates missile state using gamma guidance
        missile_state_log = update_missile_state(missile_state_log, normal_acc_cmd, sim.dt_sim, missile_tof, vehicle, aero, thrust, best_launcher_angle)
        missile_state = missile_state_log[-1]
        missile_tof += sim.dt_sim
    
    
    















































































# Piecewise cubic shaping-fit trajectory (one polynomial set per segment)
x_fit = np.full(best_time_logging.shape, np.nan)
z_fit = np.full(best_time_logging.shape, np.nan)
for coeffs_x, coeffs_z, t0, t1 in coeff_segments:
    seg = (best_time_logging >= t0) & (best_time_logging <= t1)
    x_fit[seg] = np.polyval(coeffs_x, best_time_logging[seg])
    z_fit[seg] = np.polyval(coeffs_z, best_time_logging[seg])

# Plot each state vs time
fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

# State 0: Position X
axs[0, 0].plot(best_time_logging, best_state_logging[0, :], 'b-', linewidth=1.5, label='Position X (m)')
axs[0, 0].plot(best_time_logging, x_fit, 'k:', linewidth=1.5, label='Cubic Fit')
axs[0, 0].set_ylabel('Position X [m]')
axs[0, 0].set_title('State 0: Downrange Position (X) vs Time')
axs[0, 0].grid(True)
axs[0, 0].legend()

# State 1: Altitude Z
axs[0, 1].plot(best_time_logging, best_state_logging[1, :], 'g-', linewidth=1.5, label='Altitude Z (m)')
axs[0, 1].plot(best_time_logging, z_fit, 'k:', linewidth=1.5, label='Cubic Fit')
axs[0, 1].set_ylabel('Altitude Z [m]')
axs[0, 1].set_title('State 1: Altitude (Z) vs Time')
axs[0, 1].grid(True)
axs[0, 1].legend()

# State 2: Velocity Vx
axs[1, 0].plot(best_time_logging, best_state_logging[2, :], 'r-', linewidth=1.5, label='Velocity Vx (m/s)')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Velocity Vx [m/s]')
axs[1, 0].set_title('State 2: Velocity Vx vs Time')
axs[1, 0].grid(True)
axs[1, 0].legend()

# State 3: Velocity Vz
axs[1, 1].plot(best_time_logging, best_state_logging[3, :], 'm-', linewidth=1.5, label='Velocity Vz (m/s)')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Velocity Vz [m/s]')
axs[1, 1].set_title('State 3: Velocity Vz vs Time')
axs[1, 1].grid(True)
axs[1, 1].legend()

plt.tight_layout()
fig.savefig('states_vs_time.png', dpi=150)

# Plot trajectory: X vs Z (Downrange vs Altitude)
fig_traj = plt.figure(figsize=(10, 6))
plt.plot(best_state_logging[0, :], best_state_logging[1, :], 'b-', linewidth=2, label='Vehicle Trajectory')

# Cubic shaping-fit trajectory overlaid on the true one
plt.plot(x_fit, z_fit, 'k:', linewidth=2, label='Piecewise Cubic Fit (shaping coeffs)')

# Full target trajectory, with the target marked at predicted intercept
plt.plot(target_traj[:, 0], target_traj[:, 1], 'r--', linewidth=2, label='Target Trajectory (full)')
t_intercept = best_launch_time + pred_tof
i_intercept = min(max(int(t_intercept / sim.dt_sim), 0), target_traj.shape[0] - 1)
plt.plot(target_traj[i_intercept, 0], target_traj[i_intercept, 1], 'rs', markersize=8, label='Target at Intercept')

# Asset: solid line at center, dashed lines at +/- radius, shaded zone between
plt.axvspan(asset.x - asset.radius, asset.x + asset.radius, color='r', alpha=0.1, label='Asset Zone')
plt.axvline(asset.x, color='r', linewidth=2, label='Asset')
plt.axvline(asset.x - asset.radius, color='r', linestyle='--', linewidth=1)
plt.axvline(asset.x + asset.radius, color='r', linestyle='--', linewidth=1)

plt.plot(best_state_logging[0, 0], best_state_logging[1, 0], 'go', markersize=8, label='Launch Point')
plt.plot(best_state_logging[0, -1], best_state_logging[1, -1], 'rx', markersize=8, label='POCA')
plt.xlabel('Downrange Position X [m]')
plt.ylabel('Altitude Z [m]')
plt.title(f'Vehicle Trajectory: Downrange (X) vs Altitude (Z) | Best Launcher Angle: {best_launcher_angle} deg')
plt.grid(True)
plt.legend()
plt.tight_layout()
fig_traj.savefig('trajectory_xz.png', dpi=150)

# Animate the simulated missile flight (missile_state_log) on an absolute time
# axis; the missile only appears once sim time reaches best_launch_time
msl = np.asarray(missile_state_log)
veh_x, veh_z = msl[:, 0], msl[:, 1]
veh_t_abs = best_launch_time + np.arange(len(msl)) * sim.dt_sim  # missile clock starts at launch
tgt_x, tgt_z = target_traj[:, 0], target_traj[:, 1]

fig_anim, ax_anim = plt.subplots(figsize=(10, 6))
ax_anim.set_xlim(min(veh_x.min(), tgt_x.min()) - 500, max(veh_x.max(), tgt_x.max()) + 500)
ax_anim.set_ylim(min(0, veh_z.min()) - 500, max(veh_z.max(), tgt_z.max()) + 500)
ax_anim.set_xlabel('Downrange Position X [m]')
ax_anim.set_ylabel('Altitude Z [m]')
ax_anim.set_title(f'Missile Flight Animation | Best Launcher Angle: {best_launcher_angle} deg | Launch at t = {best_launch_time:.1f} s')
ax_anim.grid(True)

veh_line, = ax_anim.plot([], [], 'b-', linewidth=1.5, label='Missile')
veh_dot, = ax_anim.plot([], [], 'bo', markersize=8)
tgt_line, = ax_anim.plot([], [], 'r--', linewidth=1.5, label='Target')
tgt_dot, = ax_anim.plot([], [], 'rs', markersize=8)

# Asset: solid line at center, dashed lines at +/- radius, shaded zone between
ax_anim.axvspan(asset.x - asset.radius, asset.x + asset.radius, color='r', alpha=0.1, label='Asset Zone')
ax_anim.axvline(asset.x, color='r', linewidth=2, label='Asset')
ax_anim.axvline(asset.x - asset.radius, color='r', linestyle='--', linewidth=1)
ax_anim.axvline(asset.x + asset.radius, color='r', linestyle='--', linewidth=1)

ax_anim.legend()

t_end = min(veh_t_abs[-1], (target_traj.shape[0] - 1) * sim.dt_sim)
t_anim = np.arange(0, t_end, sim.dt_sim)

frame_step = max(1, len(t_anim) // 400)  # cap animation at ~400 frames
frames = np.arange(0, len(t_anim), frame_step)

def update(f):
    t_now = t_anim[f]
    k_t = min(int(t_now / sim.dt_sim), target_traj.shape[0] - 1)
    tgt_line.set_data(tgt_x[:k_t+1], tgt_z[:k_t+1])
    tgt_dot.set_data([tgt_x[k_t]], [tgt_z[k_t]])
    if t_now >= best_launch_time:
        k_v = min(np.searchsorted(veh_t_abs, t_now, side='right') - 1, len(veh_x) - 1)
        veh_line.set_data(veh_x[:k_v+1], veh_z[:k_v+1])
        veh_dot.set_data([veh_x[k_v]], [veh_z[k_v]])
    else:
        veh_line.set_data([], [])
        veh_dot.set_data([], [])
    return veh_line, veh_dot, tgt_line, tgt_dot

ani = FuncAnimation(fig_anim, update, frames=frames, interval=30, blit=False)
ani.save('trajectory_animation.gif', writer=PillowWriter(fps=30))

# Plot flight path angle (gamma) vs commanded gamma over missile time of flight
fig_gamma, ax_gamma = plt.subplots(figsize=(10, 6))
ax_gamma.plot(missile_tof_log, np.rad2deg(gamma_log), 'b-', linewidth=1.5, label='Gamma (actual)')
ax_gamma.plot(missile_tof_log, np.rad2deg(gamma_cmd_log), 'r--', linewidth=1.5, label='Gamma Cmd')
ax_gamma.set_xlabel('Missile Time of Flight [s]')
ax_gamma.set_ylabel('Flight Path Angle [deg]')
ax_gamma.set_title('Gamma vs Gamma Command')
ax_gamma.grid(True)
ax_gamma.legend()
plt.tight_layout()
fig_gamma.savefig('gamma_vs_gamma_cmd.png', dpi=150)

# Plot engagement stats vs time
fig2, axs2 = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

axs2[0, 0].plot(best_time_logging, best_stats.range, 'b-', linewidth=1.5, label='Range')
axs2[0, 0].set_ylabel('Range [m]')
axs2[0, 0].set_title('Vehicle-to-Target Range vs Time')
axs2[0, 0].grid(True)
axs2[0, 0].legend()

axs2[0, 1].plot(best_time_logging, best_stats.range_rate, 'g-', linewidth=1.5, label='Range Rate')
axs2[0, 1].set_ylabel('Range Rate [m/s]')
axs2[0, 1].set_title('Range Rate (Closing Speed) vs Time')
axs2[0, 1].grid(True)
axs2[0, 1].legend()

axs2[1, 0].plot(best_time_logging, best_stats.speed_ratio, 'r-', linewidth=1.5, label='Speed Ratio')
axs2[1, 0].set_xlabel('Time [s]')
axs2[1, 0].set_ylabel('Speed Ratio [-]')
axs2[1, 0].set_title('Vehicle/Target Speed Ratio vs Time')
axs2[1, 0].grid(True)
axs2[1, 0].legend()

axs2[1, 1].plot(best_time_logging, np.rad2deg(best_stats.crossing_angle), 'm-', linewidth=1.5, label='Crossing Angle')
axs2[1, 1].set_xlabel('Time [s]')
axs2[1, 1].set_ylabel('Crossing Angle [deg]')
axs2[1, 1].set_title('Crossing Angle vs Time')
axs2[1, 1].grid(True)
axs2[1, 1].legend()

plt.tight_layout()
fig2.savefig('stats_vs_time.png', dpi=150)

plt.show()

print("Engagement Stats:")
print("POCA (m):", np.min(best_stats.range))
print("Range Rate (m/s):", best_stats.range_rate[np.argmin(best_stats.range)])
print("Speed Ratio:", best_stats.speed_ratio[np.argmin(best_stats.range)])
print("Crossing Angle (rad):", best_stats.crossing_angle[np.argmin(best_stats.range)])
print("Cost Inner (POCA^2):", best_cost_inner)
print("Cost Outer (speed loss + time):", best_cost_outer)
print("Best Launcher Angle (deg):", best_launcher_angle)
print("Best Time to Asset Desired (s):", best_time_to_asset_desired)
print("Predicted PIP [x, z] (m):", pred_pip)
print("Predicted TOF (s):", pred_tof)
print("Best Launch Time (s):", best_launch_time)
print("Shaping Coefficients (piecewise segments):")
for seg_i, (cx, cz, t0, t1) in enumerate(coeff_segments):
    print(f"  Segment {seg_i} [t = {t0:.2f} .. {t1:.2f} s]:")
    print(f"    coeffs_x = {cx}")
    print(f"    coeffs_z = {cz}")

