import numpy as np


class FireControl:
    def __init__(self, P_initial, Q, R):
        self.P_initial = np.asarray(P_initial, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.P = self.P_initial.copy()
        self.state_estimate_NED = None
        self.first_iteration = True

    @staticmethod
    def _rotation(nominal_radar_pitch_angle, beam_angle):
        phi = nominal_radar_pitch_angle + beam_angle
        return np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])

    def _ned_to_radar(self, state_NED, nominal_radar_pitch_angle, beam_angle):
        rotation = self._rotation(nominal_radar_pitch_angle, beam_angle)
        return np.concatenate((rotation @ state_NED[:2], rotation @ state_NED[2:4]))

    def _radar_to_ned(self, state_radar, nominal_radar_pitch_angle, beam_angle):
        rotation = self._rotation(nominal_radar_pitch_angle, beam_angle)
        return np.concatenate((rotation.T @ state_radar[:2], rotation.T @ state_radar[2:4]))

    @staticmethod
    def _measurement_model(state_radar):
        px, pz, vx, vz = state_radar
        range_value = max(np.hypot(px, pz), 1e-9)
        radial_velocity = (px * vx + pz * vz) / range_value
        predicted_measurement = np.array([-np.arctan2(pz, px), range_value, radial_velocity])

        denominator = max(px**2 + pz**2, 1e-12)
        H = np.zeros((3, 4))
        H[0, 0] = pz / denominator
        H[0, 1] = -px / denominator
        H[1, 0] = px / range_value
        H[1, 1] = pz / range_value
        H[2, 0] = (vx * range_value - radial_velocity * px) / denominator
        H[2, 1] = (vz * range_value - radial_velocity * pz) / denominator
        H[2, 2] = px / range_value
        H[2, 3] = pz / range_value
        return predicted_measurement, H

    def estimate_target_state(self, noisy_radar_measurement, nominal_radar_pitch_angle,
                              dt, beam_angle):
        if noisy_radar_measurement is None:
            return self.state_estimate_NED, self.P.copy()

        measurement = np.asarray(noisy_radar_measurement, dtype=float)
        if measurement.shape != (3,):
            raise ValueError("noisy_radar_measurement must have shape (3,)")
        if dt <= 0:
            raise ValueError("dt must be positive")

        if self.first_iteration or self.state_estimate_NED is None:
            los, range_value, range_rate = measurement
            state_radar = np.array([
                range_value * np.cos(los),
                -range_value * np.sin(los),
                range_rate * np.cos(los),
                -range_rate * np.sin(los),
            ])
            self.P = self.P_initial.copy()
            self.first_iteration = False
        else:
            state_radar = self._ned_to_radar(
                self.state_estimate_NED, nominal_radar_pitch_angle, beam_angle)

        transition = np.array([[1.0, 0.0, dt, 0.0],
                               [0.0, 1.0, 0.0, dt],
                               [0.0, 0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]])
        state_prior = transition @ state_radar
        covariance_prior = transition @ self.P @ transition.T + self.Q

        predicted_measurement, H = self._measurement_model(state_prior)
        innovation = measurement - predicted_measurement
        innovation[0] = (innovation[0] + np.pi) % (2.0 * np.pi) - np.pi
        innovation_covariance = H @ covariance_prior @ H.T + self.R
        kalman_gain = covariance_prior @ H.T @ np.linalg.inv(innovation_covariance)
        state_post = state_prior + kalman_gain @ innovation

        identity = np.eye(4)
        self.P = (identity - kalman_gain @ H) @ covariance_prior
        self.P = 0.5 * (self.P + self.P.T)
        self.state_estimate_NED = self._radar_to_ned(
            state_post, nominal_radar_pitch_angle, beam_angle)
        return self.state_estimate_NED.copy(), self.P.copy()
