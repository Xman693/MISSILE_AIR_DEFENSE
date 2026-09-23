import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Logger:
    def __init__(self):
        self.time_hist = []
        self.true_target_state_hist = []
        self.target_state_estimate_hist = []
        self.mode_hist = []
        self.beam_angle_hist = []
        self.true_radar_measurement_hist = []
        self.noisy_radar_measurement_hist = []
        self.radar_update_due_hist = []
        self.radar_beam_change_due_hist = []
        self.cmd_iter_hist = []
        self.launch_decision_hist = []
        self.gamma_initial_turn_hist = []
        self.t_burst_hist = []
        self.time_of_flight_hist = []

    def log_step(self, t, true_target_state, target_state_estimate, mode, beam_angle,
                 true_radar_measurement, noisy_radar_measurement,
                 radar_update_due, radar_beam_change_due, cmd_iter,
                 launch_decision=False, gamma_initial_turn=np.nan,
                 t_burst=np.nan, time_of_flight=np.nan):
        self.time_hist.append(t)
        self.true_target_state_hist.append(np.array(true_target_state, dtype=float))
        self.target_state_estimate_hist.append(np.array(target_state_estimate, dtype=float))
        self.mode_hist.append(1 if mode == "track" else 0)
        self.beam_angle_hist.append(beam_angle)
        self.true_radar_measurement_hist.append(true_radar_measurement)
        self.noisy_radar_measurement_hist.append(noisy_radar_measurement)
        self.radar_update_due_hist.append(radar_update_due)
        self.radar_beam_change_due_hist.append(radar_beam_change_due)
        self.cmd_iter_hist.append(cmd_iter)
        self.launch_decision_hist.append(launch_decision)
        self.gamma_initial_turn_hist.append(gamma_initial_turn)
        self.t_burst_hist.append(t_burst)
        self.time_of_flight_hist.append(time_of_flight)

    def plot_pre_launch_guidance(self):
        time_hist = np.asarray(self.time_hist, dtype=float)
        gamma_hist = np.asarray(self.gamma_initial_turn_hist, dtype=float)
        t_burst_hist = np.asarray(self.t_burst_hist, dtype=float)
        tof_hist = np.asarray(self.time_of_flight_hist, dtype=float)
        launch_hist = np.asarray(self.launch_decision_hist, dtype=bool)

        figure, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        axes[0].plot(time_hist, gamma_hist, color="tab:blue")
        axes[0].set_ylabel("initial gamma (deg)")
        axes[1].plot(time_hist, t_burst_hist, color="tab:orange")
        axes[1].set_ylabel("t_burst (s)")
        axes[2].plot(time_hist, tof_hist, color="tab:green")
        axes[2].set_ylabel("time of flight (s)")
        axes[3].step(time_hist, launch_hist.astype(int), where="post", color="tab:red")
        axes[3].set_ylabel("launch decision")
        axes[3].set_xlabel("time (s)")
        axes[3].set_yticks([0, 1])

        for axis in axes:
            axis.grid(True)
        figure.suptitle("Pre-launch Guidance Outputs")
        figure.tight_layout()
        plt.show()

    def plot_radar_measurement_available(self):
        time_hist = np.array(self.time_hist)
        available_hist = np.array([1 if a else 0 for a in self.radar_update_due_hist])

        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(time_hist, available_hist, drawstyle='steps-post', color='tab:red')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('radar measurement available (0/1)')
        ax.set_yticks([0, 1])
        ax.set_ylim(-0.2, 1.2)
        ax.grid(True)
        fig.tight_layout()
        plt.show()

    def plot_results(self, radar_pitch_angle=0.0):
        time_hist = np.array(self.time_hist)
        true_target_state_hist = np.array(self.true_target_state_hist)
        target_state_estimate_hist = np.array(self.target_state_estimate_hist)
        mode_hist = np.array(self.mode_hist)
        beam_angle_hist = np.array(self.beam_angle_hist)

        fig, axes = plt.subplots(4, 2, figsize=(13, 12))

        # state is [downrange, down, vdownrange, vdown, ...] in radar NED; altitude = -down
        axes[0, 0].plot(time_hist, true_target_state_hist[:, 0], label='true downrange')
        axes[0, 0].plot(time_hist, -true_target_state_hist[:, 1], label='true altitude')
        axes[0, 0].plot(time_hist, target_state_estimate_hist[:, 0], '--', label='est downrange')
        axes[0, 0].plot(time_hist, -target_state_estimate_hist[:, 1], '--', label='est altitude')
        axes[0, 0].set_xlabel('time (s)')
        axes[0, 0].set_ylabel('position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].plot(true_target_state_hist[:, 0], -true_target_state_hist[:, 1], label='true')
        axes[0, 1].plot(target_state_estimate_hist[:, 0], -target_state_estimate_hist[:, 1], '--', label='est')
        axes[0, 1].set_xlabel('downrange (m)')
        axes[0, 1].set_ylabel('altitude (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        axes[1, 0].plot(time_hist, mode_hist, drawstyle='steps-post', color='tab:green')
        axes[1, 0].set_xlabel('time (s)')
        axes[1, 0].set_ylabel('mode (0=search, 1=track)')
        axes[1, 0].set_yticks([0, 1])
        axes[1, 0].set_ylim(-0.2, 1.2)
        axes[1, 0].grid(True)

        axes[1, 1].plot(time_hist, np.rad2deg(beam_angle_hist), color='tab:purple')
        axes[1, 1].set_xlabel('time (s)')
        axes[1, 1].set_ylabel('beam angle (deg)')
        axes[1, 1].grid(True)

        # Plot true vs noisy measurements in radar frame during track mode (starting from time of track)
        track_indices = np.where(mode_hist == 1)[0]
        if len(track_indices) > 0:
            track_time = time_hist[track_indices]
            true_pos_ned = true_target_state_hist[track_indices, :2]
            true_vel_ned = true_target_state_hist[track_indices, 2:4]
            track_beam = beam_angle_hist[track_indices]

            # angles (radar_pitch_angle, track_beam) are already in radians
            theta_rad = track_beam + radar_pitch_angle

            cos_theta = np.cos(theta_rad)
            sin_theta = np.sin(theta_rad)

            # Transform NED to Radar frame: X_R, Z_R and Vx_R, Vz_R
            x_r = cos_theta * true_pos_ned[:, 0] - sin_theta * true_pos_ned[:, 1]
            z_r = sin_theta * true_pos_ned[:, 0] + cos_theta * true_pos_ned[:, 1]

            vx_r = cos_theta * true_vel_ned[:, 0] - sin_theta * true_vel_ned[:, 1]
            vz_r = sin_theta * true_vel_ned[:, 0] + cos_theta * true_vel_ned[:, 1]

            true_range_radar = np.hypot(x_r, z_r)
            true_range_rate_radar = (x_r * vx_r + z_r * vz_r) / true_range_radar
            # LOS converted to degrees only for plotting
            true_los_radar = np.rad2deg(-np.arctan2(z_r, x_r))

            # Extract discrete noisy measurement samples taken during track mode
            meas_times = []
            noisy_los = []
            noisy_range = []
            noisy_range_rate = []

            for idx in track_indices:
                meas = self.noisy_radar_measurement_hist[idx]
                if meas is not None and len(meas) == 3 and self.radar_update_due_hist[idx]:
                    meas_times.append(time_hist[idx])
                    noisy_los.append(np.rad2deg(meas[0]))  # LOS stored in radians; convert for plot
                    noisy_range.append(meas[1])
                    noisy_range_rate.append(meas[2])

            meas_times = np.array(meas_times)
            noisy_los = np.array(noisy_los)
            noisy_range = np.array(noisy_range)
            noisy_range_rate = np.array(noisy_range_rate)

            # Range plot
            axes[2, 0].plot(track_time, true_range_radar, label='true range', color='tab:blue', linewidth=1.8)
            if len(meas_times) > 0:
                axes[2, 0].plot(meas_times, noisy_range, 'r.', label='noisy measurement', markersize=4, alpha=0.7)
            axes[2, 0].set_xlabel('time (s)')
            axes[2, 0].set_ylabel('range (m)')
            axes[2, 0].set_xlim(track_time[0], track_time[-1])
            axes[2, 0].grid(True)
            axes[2, 0].legend()

            # LOS plot
            axes[2, 1].plot(track_time, true_los_radar, label='true LOS', color='tab:blue', linewidth=1.8)
            if len(meas_times) > 0:
                axes[2, 1].plot(meas_times, noisy_los, 'r.', label='noisy measurement', markersize=4, alpha=0.7)
            axes[2, 1].set_xlabel('time (s)')
            axes[2, 1].set_ylabel('LOS angle (deg)')
            axes[2, 1].set_xlim(track_time[0], track_time[-1])
            axes[2, 1].grid(True)
            axes[2, 1].legend()

            # Range rate plot
            axes[3, 0].plot(track_time, true_range_rate_radar, label='true range rate', color='tab:blue', linewidth=1.8)
            if len(meas_times) > 0:
                axes[3, 0].plot(meas_times, noisy_range_rate, 'r.', label='noisy measurement', markersize=4, alpha=0.7)
            axes[3, 0].set_xlabel('time (s)')
            axes[3, 0].set_ylabel('range rate (m/s)')
            axes[3, 0].set_xlim(track_time[0], track_time[-1])
            axes[3, 0].grid(True)
            axes[3, 0].legend()

            # Range error plot
            if len(meas_times) > 0:
                interp_true_range = np.interp(meas_times, track_time, true_range_radar)
                range_error = noisy_range - interp_true_range
                axes[3, 1].plot(meas_times, range_error, 'm.', label='range noise (m)', markersize=4)
                axes[3, 1].set_xlabel('time (s)')
                axes[3, 1].set_ylabel('range error (m)')
                axes[3, 1].set_xlim(track_time[0], track_time[-1])
                axes[3, 1].grid(True)
                axes[3, 1].legend()
            else:
                axes[3, 1].axis('off')

        else:
            for row in [2, 3]:
                for col in [0, 1]:
                    axes[row, col].text(0.5, 0.5, 'No track mode data', transform=axes[row, col].transAxes, ha='center', va='center')
                    axes[row, col].set_xlabel('time (s)')
                    axes[row, col].grid(True)

        fig.tight_layout()
        plt.show()

    def animate_results(self, radar_pitch_angle, beamwidth, dt_sim, playback_speed=1.0, fps=30):
        # radar_pitch_angle, beamwidth, and beam_angle_hist are all in radians already; only used directly in trig
        time_hist = np.array(self.time_hist)
        true_target_state_hist = np.array(self.true_target_state_hist)
        beam_angle_hist = np.array(self.beam_angle_hist)

        # compute step to achieve real-time (or playback_speed) playback at desired fps
        sim_dt_per_frame = (1.0 / fps) * playback_speed
        step = max(1, int(round(sim_dt_per_frame / dt_sim)))
        interval = (step * dt_sim / playback_speed) * 1000.0

        # target position shown as altitude vs downrange (state's second component is NED down)
        x = true_target_state_hist[:, 0]
        y = -true_target_state_hist[:, 1]
        frame_idx = np.arange(0, len(time_hist), step)

        cone_length = max(np.max(np.hypot(x, y)), 1.0) * 1.1
        beamwidth_half_rad = beamwidth / 2.0
        arc_angles_rad = np.linspace(-beamwidth_half_rad, beamwidth_half_rad, 20)

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-cone_length * 0.1, cone_length)
        ax.set_ylim(-cone_length * 0.1, cone_length)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('downrange (m)')
        ax.set_ylabel('altitude (m)')

        # draw radar dish and pedestal to physical 10m x 10m scale
        dish_scale = 10.0
        base_h = 5.0
        base_w = 5.0
        hinge_pos = np.array([0.0, base_h])

        # triangular base pedestal
        base_x = [-base_w, base_w, 0.0, -base_w]
        base_y = [0.0, 0.0, base_h, 0.0]
        ax.plot(base_x, base_y, 'k-', linewidth=2.0, label='radar (10m scale)')
        ax.plot(hinge_pos[0], hinge_pos[1], 'ko', markersize=3)

        # dish bowl pitched up at fixed radar_pitch_angle
        phi0 = radar_pitch_angle
        rot_dish = np.array([[np.cos(phi0), -np.sin(phi0)],
                             [np.sin(phi0),  np.cos(phi0)]])

        u = np.linspace(-1.0, 1.0, 30)
        dish_y_local = 5.0 * u
        dish_x_local = 2.5 * (u**2)
        dish_pts = hinge_pos[:, None] + rot_dish @ np.vstack([dish_x_local, dish_y_local])
        ax.plot(dish_pts[0], dish_pts[1], 'k-', linewidth=2.0)

        # feed stalk and horn
        feed_tip_local = np.array([4.0, 0.0])
        feed_tip = hinge_pos + rot_dish @ feed_tip_local
        feed_stalk = np.vstack([hinge_pos, feed_tip])
        ax.plot(feed_stalk[:, 0], feed_stalk[:, 1], 'k-', linewidth=1.5)
        ax.plot(feed_tip[0], feed_tip[1], 'ko', markersize=3)

        # boresight indicator line along dish normal
        ax.plot(
            [feed_tip[0], feed_tip[0] + cone_length * 0.15 * np.cos(phi0)],
            [feed_tip[1], feed_tip[1] + cone_length * 0.15 * np.sin(phi0)],
            'k:', linewidth=1.2, alpha=0.5, label='dish boresight'
        )

        target_trail, = ax.plot([], [], 'r--', linewidth=1.0, alpha=0.6, label='target trail')
        target_dot, = ax.plot([], [], 'ro', markersize=6, label='target')
        beam_line, = ax.plot([], [], 'b-', linewidth=1.5, label='RF beam')
        cone_upper, = ax.plot([], [], 'b--', linewidth=1.0)
        cone_lower, = ax.plot([], [], 'b--', linewidth=1.0)
        cone_fill = ax.fill([], [], color='tab:blue', alpha=0.2)[0]
        time_text = ax.text(0.03, 0.95, '', transform=ax.transAxes, fontsize=10,
                            bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        ax.legend(loc='upper right')

        trail_stride = max(1, step)
        beam_origin = feed_tip

        def update(frame):
            i = frame_idx[frame]
            # beam angle in inertial frame = radar pitch angle + instantaneous beam angle (radians)
            theta = radar_pitch_angle + beam_angle_hist[i]

            target_dot.set_data([x[i]], [y[i]])
            target_trail.set_data(x[:i + 1:trail_stride], y[:i + 1:trail_stride])
            beam_line.set_data([beam_origin[0], beam_origin[0] + cone_length * np.cos(theta)],
                               [beam_origin[1], beam_origin[1] + cone_length * np.sin(theta)])

            theta_upper = theta + beamwidth_half_rad
            theta_lower = theta - beamwidth_half_rad
            cone_upper.set_data([beam_origin[0], beam_origin[0] + cone_length * np.cos(theta_upper)],
                                [beam_origin[1], beam_origin[1] + cone_length * np.sin(theta_upper)])
            cone_lower.set_data([beam_origin[0], beam_origin[0] + cone_length * np.cos(theta_lower)],
                                [beam_origin[1], beam_origin[1] + cone_length * np.sin(theta_lower)])

            arc = theta + arc_angles_rad
            fill_x = np.concatenate([[beam_origin[0]], beam_origin[0] + cone_length * np.cos(arc), [beam_origin[0]]])
            fill_y = np.concatenate([[beam_origin[1]], beam_origin[1] + cone_length * np.sin(arc), [beam_origin[1]]])
            cone_fill.set_xy(np.column_stack([fill_x, fill_y]))

            time_text.set_text(f"t = {time_hist[i]:.2f} s")
            return target_dot, target_trail, beam_line, cone_upper, cone_lower, cone_fill, time_text

        anim = animation.FuncAnimation(fig, update, frames=len(frame_idx), interval=interval, blit=True)
        plt.show()
        return anim
