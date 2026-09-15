import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Arrow
import warnings
warnings.filterwarnings('ignore')
warnings.filterwarnings('always', category=RuntimeWarning)

class Guidance:
	def __init__(self, dt=0.01, num_steps=30000,
				 alpha0=0.0, gamma0_deg=38.0, speed0=30.0, q0=0.0, x0=0.0, z0=0.0,
				 x_t0=20000.0, z_t0=10000.0, vx_t0=-250.0, vz_t0=0.0,
				 gamma_cmd_deg=None, launcher_clear_time=0.3, maneuver_time=1.0, maneuver_gain=1.0,
				 constants=None):
		self.dt = dt
		self.num_steps = num_steps

		# flight phase parameters:
		# 1. 0 to launcher_clear_time (0.5s): free normal dynamics (thrust ramp & launcher exit)
		# 2. launcher_clear_time to launcher_clear_time + maneuver_time: autopilot forces gamma -> gamma_cmd
		# 3. afterwards: coast / free dynamics
		self.launcher_clear_time = launcher_clear_time
		self.gamma_cmd = np.deg2rad(gamma_cmd_deg if gamma_cmd_deg is not None else gamma0_deg)
		self.maneuver_time = maneuver_time
		self.maneuver_gain = maneuver_gain

		# state arrays
		self.alpha = np.zeros(num_steps)    # angle of attack
		self.speed = np.zeros(num_steps)    # velocity magnitude
		self.q = np.zeros(num_steps)        # pitch rate
		self.gamma = np.zeros(num_steps)    # flight path angle
		self.x = np.zeros(num_steps)        # downrange position
		self.z = np.zeros(num_steps)        # altitude position
		self.theta = np.zeros(num_steps)    # pitch angle

		# acceleration and control effort tracking
		self.an = np.zeros(num_steps)       # normal acceleration v * gammadot (m/s^2)
		self.I = np.zeros(num_steps)        # integral of an^2 dt

		# initial conditions
		self.alpha[0] = alpha0
		self.gamma[0] = np.deg2rad(gamma0_deg)
		self.speed[0] = speed0
		self.q[0] = q0
		self.x[0] = x0
		self.z[0] = z0

		self.state = np.array([self.alpha[0], self.speed[0], self.q[0], self.gamma[0], self.x[0], self.z[0]])

		self.param = constants if constants is not None else self.get_constants()

		# target position and velocity arrays
		self.x_t = np.zeros(num_steps)
		self.z_t = np.zeros(num_steps)
		self.vx_t = np.zeros(num_steps)
		self.vz_t = np.zeros(num_steps)

		self.x_t[0] = x_t0
		self.z_t[0] = z_t0
		self.vx_t[0] = vx_t0
		self.vz_t[0] = vz_t0
		self.steps_completed = num_steps

	@staticmethod
	def get_constants():
		constants = {}

		# environment parameters
		constants["rho0"] = 1.225        # sea-level air density, kg/m^3
		constants["scale_height"] = 8500.0  # atmospheric scale height, m
		constants["g"] = 9.81        # gravitational acceleration, m/s^2

		# aerodynamic coefficients
		constants["CD0"] = 0.3       # zero-alpha drag coefficient
		constants["CDalpha"] = 2.5   # drag coefficient slope wrt alpha
		constants["CLalpha"] = 2.0   # lift coefficient slope wrt alpha, per rad
		constants["Cmq"] = -50      # pitch damping moment coefficient
		constants["Cmalpha"] = -1.2  # pitching moment coefficient slope wrt alpha

		# geometry / mass properties
		constants["I"] = 20.0        # pitch moment of inertia, kg*m^2
		constants["m"] = 100.0       # mass, kg
		constants["S"] = 0.05        # reference area, m^2
		constants["b"] = 0.3         # reference length, m
		constants["Tmax"] = 15000.0  # thrust, N
		constants["thrust_ramp"] = 0.2  # thrust ramp time constant, s
		constants["thrust_hold"] = 5.0  # thrust hold time, s

		return constants

	@staticmethod
	def thrust_profile(Tmax, t, thrust_ramp, thrust_hold):
		if t < thrust_ramp:
			return Tmax * (t / thrust_ramp)
		elif t < (thrust_ramp + thrust_hold):
			return Tmax
		else:
			t_max = thrust_ramp + thrust_hold
			if t < (t_max + thrust_ramp):
				return Tmax * (1 - (t - t_max) / thrust_ramp)
			else:
				return 0.0

	@staticmethod
	def target_position(t, x0, z0, vx0, vz0):
		x_t = x0 + vx0 * t
		z_t = z0 + vz0 * t
		return x_t, z_t

	def air_density(self, z):
		"""Exponential atmosphere approximation, kg/m^3"""
		rho0 = self.param["rho0"]
		scale_height = self.param["scale_height"]
		return rho0 * np.exp(-z / scale_height)

	def missile_odes(self, state, t):
		alpha, speed, q, gamma, x, z = state

		# unpack parameters
		rho = self.air_density(z)
		g = self.param["g"]
		CD0 = self.param["CD0"]
		CDalpha = self.param["CDalpha"]
		CLalpha = self.param["CLalpha"]
		Cmq = self.param["Cmq"]
		Cmalpha = self.param["Cmalpha"]
		I = self.param["I"]
		m = self.param["m"]
		S = self.param["S"]
		b = self.param["b"]
		Tmax = self.param["Tmax"]
		thrust_ramp = self.param["thrust_ramp"]
		thrust_hold = self.param["thrust_hold"]

		# compute thrust
		T = self.thrust_profile(Tmax, t, thrust_ramp, thrust_hold)

		# compute aerodynamic forces and moments
		qinf = 0.5 * rho * speed**2
		D = qinf * S * (CD0 + CDalpha * alpha**2)
		L = qinf * S * CLalpha * alpha
		Mq = qinf * S * b * Cmq * (b / (2 * speed)) * q
		Malpha = qinf * S * b * Cmalpha * alpha

		# equations of motion
		dalpha_dt = q - (T * np.sin(alpha) + L) / (m * speed) + g * np.sin(gamma) / speed
		dspeed_dt = (T*np.cos(alpha) - D) / m - g * np.sin(gamma)
		dq_dt = (Mq + Malpha) / I
		dgamma_dt = q - dalpha_dt
		dx_dt = speed * np.cos(gamma)
		dz_dt = speed * np.sin(gamma)

		STATE_ODE = np.array([dalpha_dt, dspeed_dt, dq_dt, dgamma_dt, dx_dt, dz_dt])

		return STATE_ODE

	def missile_rk4(self, state, t, dt):
		k1 = self.missile_odes(state, t)
		k2 = self.missile_odes(state + 0.5 * dt * k1, t + 0.5 * dt)
		k3 = self.missile_odes(state + 0.5 * dt * k2, t + 0.5 * dt)
		k4 = self.missile_odes(state + dt * k3, t + dt)

		state_next = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

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
		miss_term = (miss_range ) ** 2
		cost = miss_term 

		return cost, closing_speed

	def run_simulation(self, weight_effort=1.0):
		prev_closing_speed = None
		launch_clear_steps = int(round(self.launcher_clear_time / self.dt))
		maneuver_end_steps = min(launch_clear_steps + int(round(self.maneuver_time / self.dt)), self.num_steps - 1)

		for i in range(self.num_steps - 1):
			t = i * self.dt

			# Phase 1 (0 to 0.5s): free normal dynamics to clear launcher & ramp thrust
			# Phase 2 (0.5s to 1.5s): autopilot actively steers gamma to gamma_cmd
			# Phase 3 (1.5s+): coast / free dynamics after achieving desired angle
			if i < launch_clear_steps:
				state_next = self.missile_rk4(self.state, t, self.dt)
			elif i < maneuver_end_steps:
				state_next = self.maneuver_step(i, t)
			else:
				state_next = self.missile_rk4(self.state, t, self.dt)
			self.state[:] = state_next

			self.alpha[i + 1], self.speed[i + 1], self.q[i + 1], self.gamma[i + 1], self.x[i + 1], self.z[i + 1] = state_next

			# Normal acceleration: an = v * gammadot
			gammadot = (self.gamma[i + 1] - self.gamma[i]) / self.dt
			self.an[i + 1] = self.speed[i + 1] * gammadot

			# dimensionless load factor: an / (an + g); Idot = n_load^2, I = I + Idot * dt
			g = self.param["g"]
			n_load = self.an[i + 1] / (self.an[i + 1] + g)
			Idot = n_load**2
			self.I[i + 1] = self.I[i] + Idot * self.dt

			self.x_t[i + 1], self.z_t[i + 1] = self.target_position(t + self.dt, self.x_t[0], self.z_t[0], self.vx_t[0], self.vz_t[0])

			compute_cost, closing_speed = self.evaluate_cost(
				state_next, 
				[self.x_t[i + 1], self.z_t[i + 1], self.vx_t[0], self.vz_t[0]],
				I_effort=self.I[i + 1],
				weight_effort=weight_effort
			)

			# POCA: closing speed transitions from negative (closing) to positive (opening)
			if prev_closing_speed is not None and prev_closing_speed < 0 and closing_speed >= 0:
				self.steps_completed = i + 2
				self.theta = self.gamma + self.alpha
				return compute_cost, t

			prev_closing_speed = closing_speed

		# fell through without a closing-speed sign flip: sim window ended before intercept
		warnings.warn(f"POCA never detected within num_steps={self.num_steps} (dt={self.dt}); "
		              "reported miss is just the range at truncation, not the true closest approach. "
		              "Increase num_steps.", category=RuntimeWarning)
		self.steps_completed = self.num_steps
		self.theta = self.gamma + self.alpha
		return compute_cost, t

	def plot_states(self):
		"""Plot missile state variables and control effort over time"""
		n = getattr(self, 'steps_completed', len(self.alpha))
		time = np.arange(n) * self.dt
		
		fig, axes = plt.subplots(4, 2, figsize=(14, 12))
		fig.suptitle('Missile State and Acceleration Variables', fontsize=16, fontweight='bold')
		
		# Angle of Attack
		axes[0, 0].plot(time, np.rad2deg(self.alpha[:n]), 'b-', linewidth=2)
		axes[0, 0].set_ylabel('Angle of Attack (deg)', fontweight='bold')
		axes[0, 0].grid(True, alpha=0.3)
		
		# Speed
		axes[0, 1].plot(time, self.speed[:n], 'r-', linewidth=2)
		axes[0, 1].set_ylabel('Speed (m/s)', fontweight='bold')
		axes[0, 1].grid(True, alpha=0.3)
		
		# Pitch Rate
		axes[1, 0].plot(time, np.rad2deg(self.q[:n]), 'g-', linewidth=2)
		axes[1, 0].set_ylabel('Pitch Rate (deg/s)', fontweight='bold')
		axes[1, 0].grid(True, alpha=0.3)
		
		# Flight Path Angle
		axes[1, 1].plot(time, np.rad2deg(self.gamma[:n]), 'm-', linewidth=2)
		axes[1, 1].set_ylabel('Flight Path Angle (deg)', fontweight='bold')
		axes[1, 1].grid(True, alpha=0.3)
		
		# Downrange Position
		axes[2, 0].plot(time, self.x[:n], 'c-', linewidth=2)
		axes[2, 0].set_ylabel('Downrange (m)', fontweight='bold')
		axes[2, 0].grid(True, alpha=0.3)
		
		# Altitude Position
		axes[2, 1].plot(time, self.z[:n], 'orange', linewidth=2)
		axes[2, 1].set_ylabel('Altitude (m)', fontweight='bold')
		axes[2, 1].grid(True, alpha=0.3)

		# Normal Acceleration (an = v * gammadot)
		axes[3, 0].plot(time, self.an[:n], 'darkblue', linewidth=2)
		axes[3, 0].set_xlabel('Time (s)', fontweight='bold')
		axes[3, 0].set_ylabel(r'$a_n = v\dot{\gamma}$ (m/s$^2$)', fontweight='bold')
		axes[3, 0].grid(True, alpha=0.3)

		# Integral of an^2 dt (Control / Acceleration Effort)
		axes[3, 1].plot(time, self.I[:n], 'purple', linewidth=2)
		axes[3, 1].set_xlabel('Time (s)', fontweight='bold')
		axes[3, 1].set_ylabel(r'$I = \int a_n^2 \, dt$', fontweight='bold')
		axes[3, 1].grid(True, alpha=0.3)
		
		plt.tight_layout()
		plt.show()

	def plot_trajectories(self):
		"""Plot the full missile/target trajectories and miss distance for the whole engagement"""
		time_steps = getattr(self, 'steps_completed', len(self.z))

		time_vals = np.arange(time_steps) * self.dt
		miss_distance = np.sqrt((self.x[:time_steps] - self.x_t[:time_steps])**2 +
		                          (self.z[:time_steps] - self.z_t[:time_steps])**2)

		fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
		fig.suptitle('Missile and Target Engagement', fontsize=16, fontweight='bold')

		ax1.plot(self.x[:time_steps], self.z[:time_steps], 'b-', linewidth=2, label='Missile')
		ax1.plot(self.x_t[:time_steps], self.z_t[:time_steps], 'r-', linewidth=2, label='Target')
		ax1.plot(self.x[time_steps - 1], self.z[time_steps - 1], 'bo', markersize=10, label='Missile (final)')
		ax1.plot(self.x_t[time_steps - 1], self.z_t[time_steps - 1], 'rs', markersize=10, label='Target (final)')
		ax1.set_xlabel('Downrange (m)', fontweight='bold')
		ax1.set_ylabel('Altitude (m)', fontweight='bold')
		ax1.set_title('Trajectories', fontweight='bold')
		ax1.legend(loc='upper left', fontsize=10)
		ax1.grid(True, alpha=0.3)

		ax2.plot(time_vals, miss_distance, 'g-', linewidth=2)
		ax2.set_xlabel('Time (s)', fontweight='bold')
		ax2.set_ylabel('Separation Distance (m)', fontweight='bold')
		ax2.set_title('Separation Distance', fontweight='bold')
		ax2.grid(True, alpha=0.3)

		plt.tight_layout()
		plt.show()

	def animate_trajectories(self, time_scale=1.0, fps=30, repeat=True):
		"""
		Animate missile and target trajectories in real time.
		time_scale=1.0 ensures 1 second of real time corresponds to 1 second of simulation time.
		"""
		time_steps = getattr(self, 'steps_completed', len(self.z))

		# Frame stride and interval to achieve exact 1:1 real-time playback
		frame_stride = max(1, int(round((1.0 / fps) / self.dt)))
		frame_indices = list(range(0, time_steps, frame_stride))
		if len(frame_indices) == 0 or frame_indices[-1] != time_steps - 1:
			frame_indices.append(time_steps - 1)

		interval_ms = (frame_stride * self.dt * 1000.0) / time_scale

		fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
		fig.suptitle(f'Missile and Target Animation (Real Time {time_scale:.1f}x)', fontsize=16, fontweight='bold')
		
		# Left plot: Trajectory animation
		ax1.set_xlim(min(self.x[:time_steps].min(), self.x_t[:time_steps].min()) - 500, 
		              max(self.x[:time_steps].max(), self.x_t[:time_steps].max()) + 500)
		ax1.set_ylim(0, max(self.z[:time_steps].max(), self.z_t[:time_steps].max()) + 500)
		ax1.set_xlabel('Downrange (m)', fontweight='bold', fontsize=11)
		ax1.set_ylabel('Altitude (m)', fontweight='bold', fontsize=11)
		ax1.set_title('Trajectories', fontweight='bold')
		ax1.grid(True, alpha=0.3)
		
		# Right plot: Relative distance over time
		ax2.set_xlim(0, time_steps * self.dt)
		
		# Compute miss distance
		miss_distance = np.sqrt((self.x[:time_steps] - self.x_t[:time_steps])**2 + 
		                          (self.z[:time_steps] - self.z_t[:time_steps])**2)
		ax2.set_ylim(0, miss_distance.max() + 500)
		ax2.set_xlabel('Time (s)', fontweight='bold', fontsize=11)
		ax2.set_ylabel('Miss Distance (m)', fontweight='bold', fontsize=11)
		ax2.set_title('Separation Distance', fontweight='bold')
		ax2.grid(True, alpha=0.3)
		
		# Line objects
		missile_line, = ax1.plot([], [], 'b-', linewidth=2, label='Missile')
		target_line, = ax1.plot([], [], 'r-', linewidth=2, label='Target')
		missile_point, = ax1.plot([], [], 'bo', markersize=10, label='Missile (current)')
		target_point, = ax1.plot([], [], 'rs', markersize=10, label='Target (current)')
		distance_line, = ax2.plot([], [], 'g-', linewidth=2)
		
		ax1.legend(loc='upper left', fontsize=10)
		
		# Text annotations
		time_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
		                      verticalalignment='top', fontsize=11, fontweight='bold',
		                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
		distance_text = ax1.text(0.02, 0.90, '', transform=ax1.transAxes, 
		                          verticalalignment='top', fontsize=11, fontweight='bold',
		                          bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
		
		def init():
			missile_line.set_data([], [])
			target_line.set_data([], [])
			missile_point.set_data([], [])
			target_point.set_data([], [])
			distance_line.set_data([], [])
			time_text.set_text('')
			distance_text.set_text('')
			return missile_line, target_line, missile_point, target_point, distance_line, time_text, distance_text
		
		def animate_frame(frame):
			# Update trajectory lines
			missile_line.set_data(self.x[:frame + 1], self.z[:frame + 1])
			target_line.set_data(self.x_t[:frame + 1], self.z_t[:frame + 1])
			
			# Update current positions
			missile_point.set_data([self.x[frame]], [self.z[frame]])
			target_point.set_data([self.x_t[frame]], [self.z_t[frame]])
			
			# Update distance line
			time_vals = np.arange(frame + 1) * self.dt
			distance_line.set_data(time_vals, miss_distance[:frame + 1])
			
			# Update text
			current_time = frame * self.dt
			current_distance = miss_distance[frame]
			time_text.set_text(f'Time: {current_time:.2f} s')
			distance_text.set_text(f'Miss: {current_distance:.1f} m')
			
			return missile_line, target_line, missile_point, target_point, distance_line, time_text, distance_text
		
		anim = animation.FuncAnimation(fig, animate_frame, init_func=init, 
		                               frames=frame_indices, interval=interval_ms, 
		                               blit=True, repeat=repeat)
		plt.tight_layout()
		plt.show()
		
		return anim

	def animate_chase_view(self, window=500, trail_length=200, time_scale=1.0, fps=30, repeat=True):
		"""
		Chase-camera animation: view stays centered on and zoomed in on the missile in real time.
		time_scale=1.0 ensures 1 second of real time corresponds to 1 second of simulation time.
		"""
		time_steps = getattr(self, 'steps_completed', len(self.z))

		# Frame stride and interval to achieve exact 1:1 real-time playback
		frame_stride = max(1, int(round((1.0 / fps) / self.dt)))
		frame_indices = list(range(0, time_steps, frame_stride))
		if len(frame_indices) == 0 or frame_indices[-1] != time_steps - 1:
			frame_indices.append(time_steps - 1)

		interval_ms = (frame_stride * self.dt * 1000.0) / time_scale

		miss_distance = np.sqrt((self.x[:time_steps] - self.x_t[:time_steps])**2 +
		                          (self.z[:time_steps] - self.z_t[:time_steps])**2)

		fig, ax = plt.subplots(figsize=(8, 8))
		fig.suptitle(f'Missile Chase Camera (Real Time {time_scale:.1f}x)', fontsize=16, fontweight='bold')
		ax.set_xlabel('Downrange (m)', fontweight='bold')
		ax.set_ylabel('Altitude (m)', fontweight='bold')
		ax.grid(True, alpha=0.3)

		missile_trail, = ax.plot([], [], 'b-', linewidth=2, label='Missile trail')
		target_trail, = ax.plot([], [], 'r-', linewidth=2, label='Target trail')
		missile_point, = ax.plot([], [], 'b^', markersize=14, label='Missile')
		target_point, = ax.plot([], [], 'rs', markersize=12, label='Target')
		ax.legend(loc='upper right', fontsize=9)

		time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
		                     verticalalignment='top', fontsize=11, fontweight='bold',
		                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
		distance_text = ax.text(0.02, 0.90, '', transform=ax.transAxes,
		                         verticalalignment='top', fontsize=11, fontweight='bold',
		                         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

		def animate_frame(frame):
			trail_start = max(0, frame - trail_length)
			missile_trail.set_data(self.x[trail_start:frame + 1], self.z[trail_start:frame + 1])
			target_trail.set_data(self.x_t[trail_start:frame + 1], self.z_t[trail_start:frame + 1])
			missile_point.set_data([self.x[frame]], [self.z[frame]])
			target_point.set_data([self.x_t[frame]], [self.z_t[frame]])

			# camera follows the missile, keeping it centered in a fixed-size zoom window
			cx, cz = self.x[frame], self.z[frame]
			ax.set_xlim(cx - window, cx + window)
			ax.set_ylim(cz - window, cz + window)

			current_time = frame * self.dt
			time_text.set_text(f'Time: {current_time:.2f} s')
			distance_text.set_text(f'Miss: {miss_distance[frame]:.1f} m')

			return missile_trail, target_trail, missile_point, target_point, time_text, distance_text

		# blit=False required since the axis limits change every frame (camera follows the missile)
		anim = animation.FuncAnimation(fig, animate_frame, frames=frame_indices,
		                               interval=interval_ms, blit=False, repeat=repeat)
		plt.tight_layout()
		plt.show()

		return anim

class TrajectoryOptimizer:
	def __init__(self, guidance_kwargs=None, h=1.0, tol=0.01, max_iter=100, weight_effort=1.0):
		self.guidance_kwargs = guidance_kwargs or {}
		self.h = h            # finite difference step, degrees
		self.tol = tol        # convergence tolerance, degrees
		self.max_iter = max_iter
		self.weight_effort = weight_effort

	def run_sim(self, gamma_cmd_deg):
		kwargs = dict(self.guidance_kwargs)
		kwargs["gamma_cmd_deg"] = gamma_cmd_deg
		sim = Guidance(**kwargs)
		cost, _ = sim.run_simulation(weight_effort=self.weight_effort)
		return cost

	def compute_derivatives(self, gamma_cmd_deg):
		h = self.h
		cost_minus = self.run_sim(gamma_cmd_deg - h)
		cost_0 = self.run_sim(gamma_cmd_deg)
		cost_plus = self.run_sim(gamma_cmd_deg + h)

		dJ = (cost_plus - cost_minus) / (2 * h)
		d2J = (cost_plus - 2 * cost_0 + cost_minus) / (h**2)

		return dJ, d2J, cost_0

	def newtons_step(self, gamma_cmd_deg_init):
		gamma_i = gamma_cmd_deg_init
		cost_i = self.run_sim(gamma_i)

		for run in range(1, self.max_iter + 1):
			dJ, d2J, cost_i = self.compute_derivatives(gamma_i)
			print(f"run {run}: gamma_cmd = {gamma_i:.4f} deg, cost = {cost_i:.4f}")
			gamma_next = gamma_i - dJ / d2J

			if abs(gamma_next - gamma_i) < self.tol:
				return gamma_next, cost_i

			gamma_i = gamma_next

		return gamma_i, cost_i

if __name__ == "__main__":
	gamma_cmd_deg = 45

	optimizer = TrajectoryOptimizer(h=1.0, tol=0.5)
	optimal_gamma_cmd, optimal_cost = optimizer.newtons_step(gamma_cmd_deg)
	print(f"initial gamma_cmd = {gamma_cmd_deg:.2f} deg -> optimal gamma_cmd = {optimal_gamma_cmd:.2f} deg, cost = {optimal_cost:.2f}")
	
	# Run simulation with optimal 1s-maneuver target gamma and plot results
	print("\nRunning simulation with optimal maneuver target...")
	sim = Guidance(gamma_cmd_deg=optimal_gamma_cmd)
	cost, intercept_time = sim.run_simulation()
	print(f"Final miss distance: {cost:.2f} m")
	print(f"Intercept time: {intercept_time:.3f} s")
	
	# Plot states
	print("\nGenerating state plots...")
	sim.plot_states()
	
	# Plot full engagement trajectory
	print("Generating trajectory plot...")
	sim.plot_trajectories()
	
	# Chase-camera animation following the missile
	print("Generating chase-camera animation...")
	sim.animate_chase_view(window=300)
