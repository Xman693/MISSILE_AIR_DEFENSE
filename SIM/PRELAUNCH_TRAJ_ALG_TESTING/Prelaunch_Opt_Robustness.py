# STANDALONE ROBUSTNESS TESTER FOR THE PRELAUNCH TRAJECTORY OPTIMIZER
#
# Completely separate from Main.py / the engagement sim. This script runs ONLY
# the optimization step (get_prelaunch_solution) over a grid of test cases:
#   - initial launcher angle guesses (seeds the per-candidate angle optimization)
#   - target speed scalings (varies the engagement geometry)
#
# get_prelaunch_solution now sweeps candidate launch times on a 5 s grid
# internally (keeping those under its POCA threshold, ranked by outer cost),
# then warm-starts a Newton refinement on launch time from the best candidate.
# Each case is therefore slower than a single angle-only solve.
#
# For each case it reports converged angle, predicted TOF/PIP, launch time,
# POCA, cost, and wall-clock runtime, flags pass/fail, and saves a summary plot.
#
# Edit the TEST CONFIGURATION block below to change the sweep.

import time
import numpy as np
import matplotlib.pyplot as plt

from Prelaunch_Solution import get_prelaunch_solution
from Target_Traj_Extrap_Est import extrapolate_target_traj
from Paramater import ThrustParams, AeroParams, VehicleParams, LauncherParams, AssetParams, SimParams, GroundParams

#------------------------- TEST CONFIGURATION -------------------------
LAUNCHER_ANGLE_GUESSES = [0, 1,2,3,4,5,6,7,8,9,10,15, 16, 17,18,19, 20, 25, 35, 45, 55, 65, 66, 67, 68,  75, 76, 77, 78]  # initial angle seeds [deg]
TARGET_SPEED_SCALES = [1.0]                            # multipliers on target velocity
POCA_SUCCESS_M = 100.0                                 # POCA threshold for a "pass" [m]
#----------------------------------------------------------------------

# Parameters (same sources as the sim, but instantiated independently)
aero = AeroParams()
thrust = ThrustParams()
vehicle = VehicleParams()
launcher = LauncherParams()
asset = AssetParams()
sim = SimParams()
ground = GroundParams()

# Initial vehicle state at the launcher (x, z, vx, vz)
initial_state = np.array([launcher.launcher_emplacement_x, launcher.launcher_emplacement_z, 0.0, 0.0])

# Load target truth and seed the extrapolation the same way the sim does (row at iter == 1)
target_traj = np.loadtxt("target_traj.csv", delimiter=",", skiprows=1)
true_target_state = target_traj[1, :]


def run_case(angle_guess_deg, speed_scale):
    """Run one optimization. Returns a dict of results (or the exception)."""
    seed_state = true_target_state.copy()
    seed_state[2:4] = seed_state[2:4] * speed_scale
    target_extrap = np.asarray(extrapolate_target_traj(seed_state, ground.dt_extrap, asset))

    result = {
        "angle_guess": angle_guess_deg,
        "speed_scale": speed_scale,
        "status": "OK",
        "runtime_s": np.nan,
    }

    t_start = time.perf_counter()
    try:
        (state_logging, time_logging, stats, cost_inner, cost_outer, launcher_angle,
         time_to_asset_desired, target_traj_used, pred_pip, pred_tof,
         coeff_segments, launch_time) = get_prelaunch_solution(
            initial_state, aero, thrust, vehicle, ground.dt_extrap,
            sim.dt_sim, angle_guess_deg, target_extrap)
        result["runtime_s"] = time.perf_counter() - t_start
        result["conv_angle"] = launcher_angle
        result["time_to_asset_desired"] = time_to_asset_desired
        result["pred_pip"] = pred_pip
        result["pred_tof"] = pred_tof
        result["launch_time"] = launch_time
        result["poca"] = float(np.min(stats.range))
        result["cost_POCA"] = cost_inner
        result["cost_energy_and_time"] = cost_outer
        result["state_logging"] = state_logging
        result["target_extrap"] = target_extrap
        if not np.isfinite(result["poca"]) or not np.isfinite(cost_inner):
            result["status"] = "FAIL (NaN/Inf)"
        elif result["poca"] > POCA_SUCCESS_M:
            result["status"] = f"FAIL (POCA > {POCA_SUCCESS_M:.0f} m)"
    except Exception as exc:
        result["runtime_s"] = time.perf_counter() - t_start
        result["status"] = f"ERROR ({type(exc).__name__}: {exc})"
        result["conv_angle"] = np.nan
        result["time_to_asset_desired"] = np.nan
        result["pred_pip"] = np.array([np.nan, np.nan])
        result["pred_tof"] = np.nan
        result["launch_time"] = np.nan
        result["poca"] = np.nan
        result["cost_POCA"] = np.nan
        result["cost_energy_and_time"] = np.nan

    return result


#------------------------- RUN THE SWEEP -------------------------
results = []
for speed_scale in TARGET_SPEED_SCALES:
    for angle_guess in LAUNCHER_ANGLE_GUESSES:
        print(f"Running: angle seed = {angle_guess:>5.1f} deg, target speed x{speed_scale:.2f} ...", flush=True)
        res = run_case(angle_guess, speed_scale)
        results.append(res)
        print(f"    -> {res['status']} | POCA = {res['poca']:.2f} m | angle = {res['conv_angle']:.3f} deg "
              f"| tof = {res['pred_tof']:.2f} s | runtime = {res['runtime_s']:.1f} s", flush=True)

#------------------------- SUMMARY TABLE -------------------------
print("\n" + "=" * 140)
print(f"{'Seed Ang':>9} {'Spd x':>6} {'Conv Ang':>9} {'Ttgo Des':>9} {'Pred TOF':>9} {'Launch t':>9} "
      f"{'POCA [m]':>12} {'Cost POCA':>14} {'Cost E&T':>14} {'Runtime':>8}  Status")
print("-" * 140)
for r in results:
    pip = r["pred_pip"]
    print(f"{r['angle_guess']:>9.1f} {r['speed_scale']:>6.2f} {r['conv_angle']:>9.3f} "
          f"{r['time_to_asset_desired']:>9.2f} {r['pred_tof']:>9.2f} {r['launch_time']:>9.2f} "
          f"{r['poca']:>12.2f} {r['cost_POCA']:>14.3e} {r['cost_energy_and_time']:>14.3e} {r['runtime_s']:>7.1f}s  {r['status']}")

n_pass = sum(1 for r in results if r["status"] == "OK")
print("-" * 140)
print(f"PASSED {n_pass}/{len(results)} cases (POCA <= {POCA_SUCCESS_M:.0f} m)")
print("=" * 140)

#------------------------- SUMMARY PLOT -------------------------
fig, (ax_traj, ax_poca) = plt.subplots(1, 2, figsize=(16, 7))

# Left: converged trajectories overlaid on the (nominal) target trajectory
ax_traj.plot(target_traj[:, 0], target_traj[:, 1], 'r--', linewidth=2, label='Target (nominal)')
for r in results:
    if "state_logging" not in r:
        continue
    label = f"seed {r['angle_guess']:.0f} deg x{r['speed_scale']:.2f}"
    if r["status"] == "OK":
        ax_traj.plot(r["state_logging"][0, :], r["state_logging"][1, :], 'g-', linewidth=1.2, alpha=0.8, label=f"{label} PASS")
    else:
        ax_traj.plot(r["state_logging"][0, :], r["state_logging"][1, :], 'k:', linewidth=1.2, alpha=0.8, label=f"{label} FAIL")
    ax_traj.plot(r["pred_pip"][0], r["pred_pip"][1], 'x', markersize=6)
# Asset: solid line at center, dashed lines at +/- radius, shaded zone between
ax_traj.axvspan(asset.x - asset.radius, asset.x + asset.radius, color='r', alpha=0.1, label='Asset Zone')
ax_traj.axvline(asset.x, color='r', linewidth=2, label='Asset')
ax_traj.axvline(asset.x - asset.radius, color='r', linestyle='--', linewidth=1)
ax_traj.axvline(asset.x + asset.radius, color='r', linestyle='--', linewidth=1)

ax_traj.plot(initial_state[0], initial_state[1], 'go', markersize=8, label='Launch Point')
ax_traj.set_xlabel('Downrange Position X [m]')
ax_traj.set_ylabel('Altitude Z [m]')
ax_traj.set_title('Converged Prelaunch Trajectories (green = pass, dotted = fail, x = predicted PIP)')
ax_traj.grid(True)
ax_traj.legend(fontsize=8)

# Right: POCA per case (log scale) with the pass threshold
case_labels = [f"{r['angle_guess']:.0f}°\nx{r['speed_scale']:.2f}" for r in results]
pocas = [max(r["poca"], 1e-3) for r in results]  # floor for log scale
colors = ['g' if r["status"] == "OK" else 'r' for r in results]
ax_poca.bar(range(len(results)), pocas, color=colors)
ax_poca.axhline(POCA_SUCCESS_M, color='k', linestyle='--', label=f'Pass threshold ({POCA_SUCCESS_M:.0f} m)')
ax_poca.set_yscale('log')
ax_poca.set_xticks(range(len(results)))
ax_poca.set_xticklabels(case_labels, fontsize=8)
ax_poca.set_xlabel('Case (seed angle / target speed scale)')
ax_poca.set_ylabel('POCA [m] (log)')
ax_poca.set_title('Point of Closest Approach per Case')
ax_poca.grid(True, axis='y')
ax_poca.legend()

plt.tight_layout()
fig.savefig('prelaunch_robustness.png', dpi=150)
print("\nSummary plot saved to prelaunch_robustness.png")
