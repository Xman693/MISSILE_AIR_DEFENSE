import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import warnings
warnings.filterwarnings('ignore')
warnings.filterwarnings('always', category=RuntimeWarning)


class Guidance:
	def __init__(self, dt=0.01, num_steps=3000,
				 alpha0=0.0, gamma0_deg=38.0, speed0=30.0, q0=0.0, x0=0.0, z0=0.0,
				 x_t0=10000.0, z_t0=10000.0, vx_t0=-250.0, vz_t0=0.0,
				 gamma_cmd_deg=None, launcher_clear_time=0.5, maneuver_time=1.0, maneuver_gain=1.0,
				 constants=None):
		self.dt = dt
		self.num_steps = int(round(t_max / dt)) + 1

		self.tvc_force = tvc_force        # TVC pulse magnitude, N
		self.tvc_duration = tvc_duration  # TVC burn time, s
		self.delta = 0.0                  # deflections zeroed for this step

		# state arrays
		self.V = np.zeros(self.num_steps)      # speed, m/s
		self.alpha = np.zeros(self.num_steps)   # angle of attack, rad
		self.q = np.zeros(self.num_steps)       # pitch rate, rad/s
		self.theta = np.zeros(self.num_steps)   # pitch angle, rad
		self.x = np.zeros(self.num_steps)       # downrange position, m
		self.z = np.zeros(self.num_steps)       # altitude position, m
		self.gamma = np.zeros(self.num_steps)   # flight path angle, rad (derived: theta - alpha)

		# initial conditions
		self.V[0] = V0
		self.alpha[0] = np.deg2rad(alpha0_deg)
		self.q[0] = q0
		self.theta[0] = np.deg2rad(theta0_deg)
		self.x[0] = x0
		self.z[0] = z0
		self.gamma[0] = self.theta[0] - self.alpha[0]

		self.state = np.array([self.V[0], self.alpha[0], self.q[0], self.theta[0], self.x[0], self.z[0]])

		self.param = constants if constants is not None else self.get_constants()
		self.steps_completed = self.num_steps

	@staticmethod
	def get_constants():
		constants = {}

		# environment
		constants["rho0"] = 1.225           # sea-level air density, kg/m^3
		constants["scale_height"] = 8500.0  # atmospheric scale height, m
		constants["g"] = 9.81               # gravitational acceleration, m/s^2

		# aerodynamic coefficients (nonlinear expansions in alpha)
		constants["CL_max"] = 1.5     # lift coefficient saturation magnitude
		constants["CLalpha"] = 2.0    # lift coefficient slope wrt alpha at alpha=0, per rad
		constants["CLdelta"] = 0.5    # lift coefficient slope wrt fin deflection, per rad (unused, delta=0)
		constants["CD0"] = 0.3        # zero-alpha drag coefficient
		constants["CDalpha2"] = 2.5   # drag coefficient slope wrt alpha^2
		constants["CDalpha4"] = 5.0   # drag coefficient slope wrt alpha^4
		constants["CDdelta2"] = 0.3   # drag coefficient slope wrt delta^2 (unused, delta=0)
		constants["Cm0"] = 0.0        # zero-alpha pitching moment coefficient
		constants["Cmalpha"] = -8.0   # pitching moment coefficient slope wrt alpha (statically stable)
		constants["Cmalpha3"] = -15.0 # pitching moment coefficient slope wrt alpha^3
		constants["Cmq"] = -80.0      # pitch damping moment coefficient
		constants["Cmdelta"] = -0.5   # pitching moment coefficient slope wrt fin deflection (unused, delta=0)

		# geometry / mass properties
		constants["m"] = 100.0       # mass, kg
		constants["S"] = 0.05        # reference area, m^2
		constants["b"] = 0.3         # reference length, m
		constants["Tmax"] = 15000.0  # thrust, N
		constants["thrust_ramp"] = 1.0  # thrust ramp time constant, s
		constants["thrust_hold"] = 5.0  # thrust hold time, s

		return constants

	def air_density(self, z):
		"""Exponential atmosphere approximation, kg/m^3"""
		rho0 = self.param["rho0"]
		scale_height = self.param["scale_height"]
		return rho0 * np.exp(-z / scale_height)

	def thrust_profile(self, t):
		Tmax = self.param["Tmax"]
		thrust_ramp = self.param["thrust_ramp"]
		thrust_hold = self.param["thrust_hold"]

		if t < thrust_ramp:
			return Tmax * (t / thrust_ramp)
		elif t < (thrust_ramp + thrust_hold):
			return Tmax
		else:
			t_end_hold = thrust_ramp + thrust_hold
			if t < (t_end_hold + thrust_ramp):
				return Tmax * (1 - (t - t_end_hold) / thrust_ramp)
			else:
				return 0.0

	def tvc_profile(self, t):
		"""Fixed-magnitude TVC pulse right off the rail, then off."""
		return self.tvc_force if t < self.tvc_duration else 0.0

	def compute_aero_coefficients(self, alpha, V, q):
		"""Nonlinear CL/CD/Cm expansions (aero only; excludes thrust/TVC contributions)."""
		c = self.param["c"]
		delta = self.delta

		CL_max = self.param["CL_max"]
		CLalpha = self.param["CLalpha"]
		CLdelta = self.param["CLdelta"]
		CD0 = self.param["CD0"]
		CDalpha2 = self.param["CDalpha2"]
		CDalpha4 = self.param["CDalpha4"]
		CDdelta2 = self.param["CDdelta2"]
		Cm0 = self.param["Cm0"]
		Cmalpha = self.param["Cmalpha"]
		Cmalpha3 = self.param["Cmalpha3"]
		Cmq = self.param["Cmq"]
		Cmdelta = self.param["Cmdelta"]

		CL = CL_max * np.tanh(CLalpha * alpha / CL_max) + CLdelta * delta
		CD = CD0 + CDalpha2 * alpha**2 + CDalpha4 * alpha**4 + CDdelta2 * delta**2
		Cm = Cm0 + Cmalpha * alpha + Cmalpha3 * alpha**3 + Cmq * (q * c / (2 * V)) + Cmdelta * delta

		return CL, CD, Cm

	def missile_odes(self, state, t):
		V, alpha, q, theta, x, z = state

		g = self.param["g"]
		m = self.param["m"]
		S = self.param["S"]
		c = self.param["c"]
		Iy = self.param["Iy"]
		l_tvc = self.param["l_tvc"]

		gamma = theta - alpha
		rho = self.air_density(z)
		T = self.thrust_profile(t)
		F_tvc = self.tvc_profile(t)

		qbar = 0.5 * rho * V**2
		CL, CD, Cm = self.compute_aero_coefficients(alpha, V, q)
		L = qbar * S * CL
		D = qbar * S * CD
		M = qbar * S * c * Cm

		Vdot = (T * np.cos(alpha) - F_tvc * np.sin(alpha) - D) / m - g * np.sin(theta - alpha)

		alphadot = q - (L + T * np.sin(alpha) + F_tvc * np.cos(alpha)
						- m * g * np.cos(theta - alpha)) / (m * V)

		qdot = (M + l_tvc * F_tvc) / Iy

		thetadot = q
		xdot = V * np.cos(gamma)
		zdot = V * np.sin(gamma)

		return np.array([Vdot, alphadot, qdot, thetadot, xdot, zdot])

	def missile_rk4(self, state, t, dt):
		k1 = self.missile_odes(state, t)
		k2 = self.missile_odes(state + 0.5 * dt * k1, t + 0.5 * dt)
		k3 = self.missile_odes(state + 0.5 * dt * k2, t + 0.5 * dt)
		k4 = self.missile_odes(state + dt * k3, t + dt)
		return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

		return state_next

	def maneuver_step(self, i, t):
		"""Force gamma toward gamma_cmd via a commanded gammadot, inverting gammadot = f(alpha) for the alpha needed to fly it"""
		g = self.param["g"]
		m = self.param["m"]
		S = self.param["S"]
		CD0 = self.param["CD0"]
		CDalpha = self.param["CDalpha"]
		CLalpha = self.param["CLalpha"]
		Tmax = self.param["Tmax"]
		thrust_ramp = self.param["thrust_ramp"]
		thrust_hold = self.param["thrust_hold"]

		gamma_prev = self.gamma[i]
		alpha_prev = self.alpha[i]
		speed_prev = self.speed[i]
		x_prev = self.x[i]
		z_prev = self.z[i]
		theta_prev = gamma_prev + alpha_prev

		gammadot_cmd = -self.maneuver_gain * (gamma_prev - self.gamma_cmd)
		gamma_next = gamma_prev + gammadot_cmd * self.dt

		# gammadot = (T*sin(alpha) + L(alpha) - m*g*sin(gamma)) / (m*speed); L is linear in alpha so
		# solve numerically (Newton) for alpha to hit gammadot_cmd, starting from the previous alpha
		T = self.thrust_profile(Tmax, t, thrust_ramp, thrust_hold)
		rho = self.air_density(z_prev)
		qinf = 0.5 * rho * speed_prev**2
		rhs = m * speed_prev * gammadot_cmd + m * g * np.sin(gamma_prev)

		alpha_next = alpha_prev
		for _ in range(10):
			f = T * np.sin(alpha_next) + qinf * S * CLalpha * alpha_next - rhs
			fp = T * np.cos(alpha_next) + qinf * S * CLalpha
			fp = fp if abs(fp) > 1e-9 else 1e-9
			alpha_next = alpha_next - f / fp
		alpha_next = np.clip(alpha_next, -np.pi/2, np.pi/2)

		theta_next = gamma_next + alpha_next
		q_next = (theta_next - theta_prev) / self.dt

		D = qinf * S * (CD0 + CDalpha * alpha_next**2)
		dspeed_dt = (T * np.cos(alpha_next) - D) / m - g * np.sin(gamma_prev)
		speed_next = speed_prev + dspeed_dt * self.dt

		x_next = x_prev + speed_prev * np.cos(gamma_prev) * self.dt
		z_next = z_prev + speed_prev * np.sin(gamma_prev) * self.dt

		return np.array([alpha_next, speed_next, q_next, gamma_next, x_next, z_next])

	@staticmethod
	def evaluate_cost(state, target_state, I_effort=0.0, weight_effort=1.0, poca_tol=2.0):
		rel_pos = np.array([state[4] - target_state[0], state[5] - target_state[1]])
		rel_vel = np.array([state[1] * np.cos(state[3]) - target_state[2], state[1] * np.sin(state[3]) - target_state[3]])

		miss_range = np.linalg.norm(rel_pos)
		closing_speed = np.dot(rel_pos, rel_vel) / miss_range

		# dimensionless miss term: (POCA / (poca_tol + POCA))^2, plus dimensionless load-factor effort
		miss_term = (miss_range / (poca_tol + miss_range)) ** 2
		cost = miss_term + weight_effort * I_effort

		return cost, closing_speed

	def run_simulation(self, weight_effort=1.0):
		prev_closing_speed = None
		launch_clear_steps = int(round(self.launcher_clear_time / self.dt))
		maneuver_end_steps = min(launch_clear_steps + int(round(self.maneuver_time / self.dt)), self.num_steps - 1)

		for i in range(self.num_steps - 1):
			t = i * self.dt
			self.state = self.missile_rk4(self.state, t, self.dt)

			(self.V[i + 1], self.alpha[i + 1], self.q[i + 1],
			 self.theta[i + 1], self.x[i + 1], self.z[i + 1]) = self.state
			self.gamma[i + 1] = self.theta[i + 1] - self.alpha[i + 1]

		self.steps_completed = self.num_steps

	def plot_states(self):
		n = self.steps_completed
		time = np.arange(n) * self.dt

		fig, axes = plt.subplots(4, 2, figsize=(14, 13))
		fig.suptitle('Missile State Propagation', fontsize=16, fontweight='bold')

		axes[0, 0].plot(time, self.V[:n], 'r-', linewidth=2)
		axes[0, 0].set_ylabel('Speed V (m/s)', fontweight='bold')
		axes[0, 0].grid(True, alpha=0.3)

		axes[0, 1].plot(time, np.rad2deg(self.alpha[:n]), 'b-', linewidth=2)
		axes[0, 1].set_ylabel('Angle of Attack (deg)', fontweight='bold')
		axes[0, 1].grid(True, alpha=0.3)

		axes[1, 0].plot(time, np.rad2deg(self.q[:n]), 'g-', linewidth=2)
		axes[1, 0].set_ylabel('Pitch Rate (deg/s)', fontweight='bold')
		axes[1, 0].grid(True, alpha=0.3)

		axes[1, 1].plot(time, np.rad2deg(self.theta[:n]), 'm-', linewidth=2)
		axes[1, 1].set_ylabel('Pitch Angle theta (deg)', fontweight='bold')
		axes[1, 1].grid(True, alpha=0.3)

		axes[2, 0].plot(time, np.rad2deg(self.gamma[:n]), 'c-', linewidth=2)
		axes[2, 0].set_ylabel('Flight Path Angle gamma (deg)', fontweight='bold')
		axes[2, 0].grid(True, alpha=0.3)

		axes[2, 1].plot(time, self.x[:n], 'orange', linewidth=2)
		axes[2, 1].set_ylabel('Downrange x (m)', fontweight='bold')
		axes[2, 1].grid(True, alpha=0.3)

		axes[3, 0].plot(time, self.z[:n], 'darkblue', linewidth=2)
		axes[3, 0].set_xlabel('Time (s)', fontweight='bold')
		axes[3, 0].set_ylabel('Altitude z (m)', fontweight='bold')
		axes[3, 0].grid(True, alpha=0.3)

		axes[3, 1].axis('off')

		plt.tight_layout()
		plt.show()

	def plot_trajectory(self):
		n = self.steps_completed
		fig, ax = plt.subplots(figsize=(8, 6))
		ax.plot(self.x[:n], self.z[:n], 'b-', linewidth=2)
		ax.set_xlabel('Downrange (m)', fontweight='bold')
		ax.set_ylabel('Altitude (m)', fontweight='bold')
		ax.set_title('Missile Trajectory', fontsize=14, fontweight='bold')
		ax.grid(True, alpha=0.3)
		plt.tight_layout()
		plt.show()

	def animate_chase_view(self, window=500, trail_length=200, time_scale=1.0, fps=30, repeat=True):
		"""
		Chase-camera animation: view stays centered on and zoomed in on the missile in real time.
		time_scale=1.0 ensures 1 second of real time corresponds to 1 second of simulation time.
		"""
		time_steps = self.steps_completed

		# Frame stride and interval to achieve exact 1:1 real-time playback
		frame_stride = max(1, int(round((1.0 / fps) / self.dt)))
		frame_indices = list(range(0, time_steps, frame_stride))
		if len(frame_indices) == 0 or frame_indices[-1] != time_steps - 1:
			frame_indices.append(time_steps - 1)

		interval_ms = (frame_stride * self.dt * 1000.0) / time_scale

		fig, ax = plt.subplots(figsize=(8, 8))
		fig.suptitle(f'Missile Chase Camera (Real Time {time_scale:.1f}x)', fontsize=16, fontweight='bold')
		ax.set_xlabel('Downrange (m)', fontweight='bold')
		ax.set_ylabel('Altitude (m)', fontweight='bold')
		ax.grid(True, alpha=0.3)

		missile_trail, = ax.plot([], [], 'b-', linewidth=2, label='Missile trail')
		missile_point, = ax.plot([], [], 'b^', markersize=14, label='Missile')
		ax.legend(loc='upper right', fontsize=9)

		time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
		                     verticalalignment='top', fontsize=11, fontweight='bold',
		                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
		state_text = ax.text(0.02, 0.90, '', transform=ax.transAxes,
		                      verticalalignment='top', fontsize=11, fontweight='bold',
		                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

		def animate_frame(frame):
			trail_start = max(0, frame - trail_length)
			missile_trail.set_data(self.x[trail_start:frame + 1], self.z[trail_start:frame + 1])
			missile_point.set_data([self.x[frame]], [self.z[frame]])

			# camera follows the missile, keeping it centered in a fixed-size zoom window
			cx, cz = self.x[frame], self.z[frame]
			ax.set_xlim(cx - window, cx + window)
			ax.set_ylim(cz - window, cz + window)

			current_time = frame * self.dt
			time_text.set_text(f'Time: {current_time:.2f} s')
			state_text.set_text(f'V: {self.V[frame]:.1f} m/s\n'
			                     f'theta: {np.rad2deg(self.theta[frame]):.1f} deg\n'
			                     f'alpha: {np.rad2deg(self.alpha[frame]):.1f} deg')

			return missile_trail, missile_point, time_text, state_text

		# blit=False required since the axis limits change every frame (camera follows the missile)
		anim = animation.FuncAnimation(fig, animate_frame, frames=frame_indices,
		                               interval=interval_ms, blit=False, repeat=repeat)
		plt.tight_layout()
		plt.show()

		return anim


def sweep_tvc_duration(t_max=2.0, multipliers=None):
	"""Propagate to t_max for each TVC burst duration and return (durations_s, gamma_deg_at_t_max)."""
	multipliers = multipliers if multipliers is not None else np.arange(1.0, 5.01, 0.2)
	durations = 0.1 * multipliers  # sweeps burst time from 0.1s to 0.5s

	gamma_deg = np.zeros(len(durations))
	for i, tvc_duration in enumerate(durations):
		sim = Guidance(t_max=t_max, tvc_duration=tvc_duration)
		sim.run_simulation()
		gamma_deg[i] = np.rad2deg(sim.gamma[-1])

	return durations, gamma_deg


def plot_tvc_duration_sweep(durations, gamma_deg, t_max=2.0):
	fig, ax = plt.subplots(figsize=(8, 6))
	ax.plot(durations, gamma_deg, 'o-', color='c', linewidth=2)
	ax.set_xlabel('TVC Burst Duration (s)', fontweight='bold')
	ax.set_ylabel(f'Flight Path Angle gamma at t={t_max:.0f}s (deg)', fontweight='bold')
	ax.set_title('Flight Path Angle vs TVC Burst Time', fontsize=14, fontweight='bold')
	ax.grid(True, alpha=0.3)
	plt.tight_layout()
	plt.show()


if __name__ == "__main__":
	durations, gamma_deg = sweep_tvc_duration(t_max=2.0)
	for d, gam in zip(durations, gamma_deg):
		print(f"tvc_duration={d:.2f}s -> gamma(2s)={gam:.3f} deg")
	plot_tvc_duration_sweep(durations, gamma_deg, t_max=2.0)
