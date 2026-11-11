import numpy as np
from Prelaunch_Solution import vehicle_propogator

def get_acc_cmd(missile_state, missile_tof, coeff_segments):
    

    
    # first determine which segment the missile is currently in based on missile_tof
    # (fall back to the last segment if missile_tof runs past the final segment)
    seg_coeffs_x, seg_coeffs_z = coeff_segments[-1][0], coeff_segments[-1][1]
    for coeffs_x, coeffs_z, t0, t1 in coeff_segments:
        if t0 <= missile_tof <= t1:
            seg_coeffs_x = coeffs_x
            seg_coeffs_z = coeffs_z
            break

    x = missile_state[0]
    z = missile_state[1]
    vx = missile_state[2]
    vz = missile_state[3]

    x_des = np.polyval(seg_coeffs_x, missile_tof)
    z_des = np.polyval(seg_coeffs_z, missile_tof)

    deltax = x_des - x

    deltaz = z_des - z

    gamma_des = np.atan2(deltaz, deltax)

    gamma = np.atan2(vz, vx)


    k = 3
    gammdot_des = -k * (gamma - gamma_des)

    acc_normal = np.linalg.norm([vx, vz]) * gammdot_des


    return acc_normal, gamma_des


def update_missile_state(missile_state_log, normal_acc_cmd, dt, time, vehicle, aero, thrust, launcher_angle_deg):
    missile_state = missile_state_log[-1].copy()
    v = np.linalg.norm([missile_state[2], missile_state[3]])

    # while at rest on the rail, fly at the launcher angle until thrust builds speed
    on_rail = v < 1e-6
    if on_rail:
        gamma = np.deg2rad(launcher_angle_deg)
    else:
        gamma = np.arctan2(missile_state[3], missile_state[2])

    rho = np.exp(-missile_state[1]/10000)  # simple exponential atmosphere model based on altitude (z)

    qinf = 0.5 * rho * np.linalg.norm([missile_state[2], missile_state[3]])**2  # dynamic pressure based on velocity magnitude

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


    v = max(v + vdot * dt, 0.0)  # rail constrains the missile from sliding backward
    if v > 1e-6:
        gamma = gamma + (normal_acc_cmd / v - 9.81 * np.cos(gamma)/v) * dt
    missile_state[2] = v * np.cos(gamma)
    missile_state[3] = v * np.sin(gamma)
            
    missile_state[0] = missile_state[0] + missile_state[2] * dt
    missile_state[1] = missile_state[1] + missile_state[3] * dt
    
            
            
    missile_state_log.append(missile_state.copy())
            
            
    
         
       
       
    
    return missile_state_log
        

