from argusim.simulation_manager import SimLogger
import numpy as np

class CLSimLogger(SimLogger):

    def __init__(self, log_directory, num_RWs, num_photodiodes, num_MTBs, num_panels, J2000_start_time):
        super().__init__(log_directory, num_RWs, num_photodiodes, num_MTBs, num_panels, J2000_start_time)

        self.fsw_state_labels = ["r_x ECI [m]", 
                            "r_y ECI [m]", 
                            "r_z ECI [m]", 
                            "v_x ECI [m/s]", 
                            "v_y ECI [m/s]", 
                            "v_z ECI [m/s]",
                            "q_w", 
                            "q_x", 
                            "q_y", 
                            "q_z", 
                            "omega_x [rad/s]", 
                            "omega_y [rad/s]", 
                            "omega_z [rad/s]", 
                            "rSun_x ECI [m]",
                            "rSun_y ECI [m]",
                            "rSun_z ECI [m]",
                            "xMag ECI [T]",
                            "yMag ECI [T]",
                            "zMag ECI [T]"] + \
                            ["omega_RW_" + str(i) + " [rad/s]" for i in range(self.num_RWs)] + \
                            ["bias_x [rad/s]",
                            "bias_y [rad/s]",
                            "bias_z [rad/s]"]    + \
                            ["rSun_x body [-]",
                            "rSun_y body [-]",
                            "rSun_z body [-]"]
        P_labels = [f"P_{i}_{j}" for i in range(6) for j in range(6)]
        # Only keep upper triangular P_labels for covariance logging
        self.fsw_ad_covar_labels = [P_labels[i * 6 + j] for i in range(6) for j in range(i, 6)]
    
    def log_fsw(self, current_time, adcs):
        # log state equivalent
        fsw_state = np.zeros((len(self.fsw_state_labels),))
        fsw_state[:3] = adcs.AD.state[adcs.AD.position_idx]
        fsw_state[3:6] = adcs.AD.state[adcs.AD.velocity_idx]
        fsw_state[6:10] = adcs.AD.state[adcs.AD.attitude_idx]
        fsw_state[10:13] = adcs.AD.state[adcs.AD.omega_idx]
        fsw_state[13:16] = adcs.AD.state[adcs.AD.sun_pos_eci_idx]
        fsw_state[16:19] = adcs.AD.state[adcs.AD.mag_field_eci_idx]
        fsw_state[19:19 + self.num_RWs] = 0
        fsw_state[19 + self.num_RWs:22 + self.num_RWs] = adcs.AD.state[adcs.AD.bias_idx]
        fsw_state[22 + self.num_RWs:25 + self.num_RWs] = adcs.AD.state[adcs.AD.sun_pos_idx]

        self.log_v(
            "fsw_state.bin",
            [current_time - self.J2000_start_time] + fsw_state.tolist(),
            ["Time [s]"] + self.fsw_state_labels,
        )
        EKF_P_ij = adcs.AD.P[np.triu_indices_from(adcs.AD.P)]
        self.log_v(
            "state_covariance.bin",
            [current_time - self.J2000_start_time] + EKF_P_ij.tolist(),
            ["Time [s]"] + self.fsw_ad_covar_labels,
        )

        """
    def log_estimation(self, current_time, attitude_ekf_state, attitude_estimate_error, gyro_bias_error, EKF_sigmas):
        self.log_v(
            "EKF_state.bin",
            [current_time - self.J2000_start_time] + attitude_ekf_state.tolist(),
            ["Time [s]"] + self.EKF_state_labels,
        )
        self.log_v(
            "EKF_error.bin",
            [current_time - self.J2000_start_time] + attitude_estimate_error.tolist() + gyro_bias_error.tolist(),
            ["Time [s]"] + self.attitude_estimate_error_labels + self.gyro_bias_error_labels,
        )

        self.log_v(
            "state_covariance.bin",
            [current_time - self.J2000_start_time] + EKF_sigmas.tolist(),
            ["Time [s]"] + self.EKF_sigma_labels,
        )
        """