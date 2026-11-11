# STANDALONE SWEEP OF time_to_asset_desired VS OUTER COST (Cost E&T)
#
# For each candidate time_to_asset_desired, the target trajectory is trimmed to
# that window (get_trimmed_traj), the INNER loop (get_best_solution) finds the
# best launcher angle for that window, and the resulting inner/outer costs are
# recorded. This maps out the cost surface the OUTER Newton iteration sees.
#
# Edit the SWEEP CONFIGURATION block below to change the range/resolution.

import numpy as np
import matplotlib.pyplot as plt

from Prelaunch_Solution import get_best_solution
from Target_Traj_Extrap_Est import extrapolate_target_traj, get_trimmed_traj
from Paramater import ThrustParams, AeroParams, VehicleParams, LauncherParams, AssetParams, GroundParams

#------------------------- SWEEP CONFIGURATION -------------------------
TTAD_MIN_S = 30.0        # smallest time-to-asset-desired to test [s]
TTAD_MAX_S = None        # largest to test [s] (None = tgo_max - 1)
NUM_POINTS = 40          # sweep resolution
ANGLE_SEED_DEG = 45.0    # cold-start seed for the inner angle loop at every point
#-----------------------------------------------------------------------

aero = AeroParams()
thrust = ThrustParams()
vehicle = VehicleParams()
launcher = LauncherParams()
asset = AssetParams()
ground = GroundParams()

initial_state = np.array([launcher.launcher_emplacement_x, launcher.launcher_emplacement_z, 0.0, 0.0])

target_traj = np.loadtxt("target_traj.csv", delimiter=",", skiprows=1)
target_extrap = np.asarray(extrapolate_target_traj(target_traj[1, :], ground.dt_extrap, asset))
tgo_max = target_extrap.shape[0] * ground.dt_extrap

ttad_max = TTAD_MAX_S if TTAD_MAX_S is not None else tgo_max - 1.0
ttad_sweep = np.linspace(TTAD_MIN_S, ttad_max, NUM_POINTS)

cost_inner_sweep = np.full(NUM_POINTS, np.nan)
cost_outer_sweep = np.full(NUM_POINTS, np.nan)
poca_sweep = np.full(NUM_POINTS, np.nan)
tof_sweep = np.full(NUM_POINTS, np.nan)
angle_sweep = np.full(NUM_POINTS, np.nan)

for i, ttad in enumerate(ttad_sweep):
    try:
        trimmed = get_trimmed_traj(target_extrap, ground.dt_extrap, ttad)
        state_logging, time_logging, stats, cost_inner, cost_outer, i_poca, conv_angle = \
            get_best_solution(initial_state, aero, thrust, vehicle, ground.dt_extrap, ANGLE_SEED_DEG, trimmed)
        cost_inner_sweep[i] = cost_inner
        cost_outer_sweep[i] = cost_outer
        poca_sweep[i] = stats.range[i_poca]
        tof_sweep[i] = time_logging[i_poca]
        angle_sweep[i] = conv_angle
        print(f"ttad = {ttad:7.2f} s | cost E&T = {cost_outer:12.4f} | POCA = {stats.range[i_poca]:10.2f} m "
              f"| tof = {time_logging[i_poca]:6.2f} s | angle = {conv_angle:9.3f} deg", flush=True)
    except Exception as exc:
        print(f"ttad = {ttad:7.2f} s | ERROR ({type(exc).__name__}: {exc})", flush=True)

# best point on the swept grid
i_best = np.nanargmin(cost_outer_sweep)
print(f"\nBest on grid: ttad = {ttad_sweep[i_best]:.2f} s | cost E&T = {cost_outer_sweep[i_best]:.4f} "
      f"| POCA = {poca_sweep[i_best]:.2f} m | tof = {tof_sweep[i_best]:.2f} s | angle = {angle_sweep[i_best]:.3f} deg")
print(f"(launch_time = tgo_max - ttad = {tgo_max - ttad_sweep[i_best]:.2f} s on the sim clock)")

#------------------------- PLOT -------------------------
fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

axs[0].plot(ttad_sweep, cost_outer_sweep, 'b.-', linewidth=1.5, label='Cost E&T (outer)')
axs[0].plot(ttad_sweep[i_best], cost_outer_sweep[i_best], 'r*', markersize=14, label=f'Grid min ({ttad_sweep[i_best]:.1f} s)')
axs[0].set_ylabel('Cost E&T [-]')
axs[0].set_title('Outer Cost (speed loss + time) vs Time-to-Asset-Desired')
axs[0].grid(True)
axs[0].legend()

axs[1].semilogy(ttad_sweep, poca_sweep, 'g.-', linewidth=1.5, label='POCA (inner-optimized)')
axs[1].axhline(100, color='k', linestyle='--', label='Pass threshold (100 m)')
axs[1].set_ylabel('POCA [m] (log)')
axs[1].set_title('Achievable Miss Distance vs Time-to-Asset-Desired')
axs[1].grid(True)
axs[1].legend()

axs[2].plot(ttad_sweep, tof_sweep, 'm.-', linewidth=1.5, label='Predicted TOF at POCA')
axs[2].plot(ttad_sweep, ttad_sweep, 'k:', linewidth=1, label='TOF = ttad (window limit)')
axs[2].set_xlabel('Time-to-Asset-Desired [s]')
axs[2].set_ylabel('Predicted TOF [s]')
axs[2].set_title('Missile Time of Flight vs Window Length')
axs[2].grid(True)
axs[2].legend()

plt.tight_layout()
fig.savefig('cost_outer_vs_ttad.png', dpi=150)
print("\nPlot saved to cost_outer_vs_ttad.png")
