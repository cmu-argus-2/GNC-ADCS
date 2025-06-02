# Attitude Determination and Control (ADC) task
from .adcs.acs import mcm_coil_allocator, spin_stabilizing_controller, sun_pointing_controller # , zero_all_coils
from .adcs.ad import AttitudeDetermination
from .adcs.consts import Modes, StatusConst
import numpy as np
from .other_functions.telemetry_constants import ADCS_IDX, CDH_IDX, GPS_IDX
# from core import DataHandler as DH
from .other_functions.template_task import TemplateTask
# from core import state_manager as SM
# from core.states import STATES
# from core.time_processor import TimeProcessor as TPM

"""
    ASSUMPTIONS :
        - ADCS Task runs at 5 Hz (TBD if we can't handle this)
        - In detumbling, control loop executes every 200ms
        - In nominal/experiment, for 4 executions, we do nothing. On the fifth, we run full MEKF + control
"""

class Task(TemplateTask):
    """data_keys = [
        "TIME_ADCS",
        "MODE",
        "GYRO_X",
        "GYRO_Y",
        "GYRO_Z",
        "MAG_X",
        "MAG_Y",
        "MAG_Z",
        "SUN_STATUS",
        "SUN_VEC_X",
        "SUN_VEC_Y",
        "SUN_VEC_Z",
        "LIGHT_SENSOR_XP",
        "LIGHT_SENSOR_XM",
        "LIGHT_SENSOR_YP",
        "LIGHT_SENSOR_YM",
        "LIGHT_SENSOR_ZP1",
        "LIGHT_SENSOR_ZP2",
        "LIGHT_SENSOR_ZP3",
        "LIGHT_SENSOR_ZP4",
        "LIGHT_SENSOR_ZM",
        "XP_COIL_STATUS",
        "XM_COIL_STATUS",
        "YP_COIL_STATUS",
        "YM_COIL_STATUS",
        "ZP_COIL_STATUS",
        "ZM_COIL_STATUS",
        "COARSE_ATTITUDE_QW",
        "COARSE_ATTITUDE_QX",
        "COARSE_ATTITUDE_QY",
        "COARSE_ATTITUDE_QZ",
    ]"""

    # log_data = [0] * 31
    coil_status = [0] * 6

    # Attitude Determination
    AD = AttitudeDetermination()

    ## ADCS Modes and switching logic
    MODE = Modes.TUMBLING

    # Sub-task architecture
    # [TODO:] fix when adding back state_manager
    execution_counter = 4 # 0

    # Failure message storage
    failure_messages = []

    def __init__(self, id):
        super().__init__(id)
        self.name = "ADCS"  # Override the name

    def main_task(self, sim_delta_time, measurements, sim_Idx): #  -> None:
        
        gyro                      = measurements[sim_Idx["Y"]["GYRO"]]
        mag                       = measurements[sim_Idx["Y"]["MAG"]]
        light_sensor_lux_readings = measurements[sim_Idx["Y"]["SUN"]]
        gps_data                  = np.zeros((21,)) 
        gps_data[GPS_IDX.TIME_GPS]                        = sim_delta_time
        gps_data[GPS_IDX.GPS_ECEF_X : GPS_IDX.GPS_ECEF_Z + 1]  = measurements[sim_Idx["Y"]["GPS_POS"]]
        gps_data[GPS_IDX.GPS_ECEF_VX : GPS_IDX.GPS_ECEF_VZ + 1] = measurements[sim_Idx["Y"]["GPS_VEL"]]
        
        # if SM.current_state == STATES.STARTUP:
        #     pass

        # else:
            # if not DH.data_process_exists("adcs"):
            #     data_format = "LB" + 6 * "f" + "B" + 3 * "f" + 9 * "H" + 6 * "B" + 4 * "f"
            #     DH.register_data_process("adcs", data_format, True, data_limit=100000, write_interval=5)

        self.time = sim_delta_time
        # self.log_data[ADCS_IDX.TIME_ADCS] = self.time

        # ------------------------------------------------------------------------------------------------------------------------------------
        # DETUMBLING
        # ------------------------------------------------------------------------------------------------------------------------------------
        """
        if SM.current_state == STATES.DETUMBLING:
            # Query the Gyro
            self.AD.gyro_update(self.time, measurements, update_covariance=False)

            # Query Magnetometer
            self.AD.magnetometer_update(self.time, measurements, update_covariance=False)

            # Run Attitude Control
            self.attitude_control()

            # Check if detumbling has been completed
            if self.AD.current_mode(self.MODE) != Modes.TUMBLING:
                zero_all_coils()
                self.MODE = Modes.STABLE
        """
        # [TODO:] work back low power mode with the FSW state machine

        # ------------------------------------------------------------------------------------------------------------------------------------
        # NOMINAL
        # ------------------------------------------------------------------------------------------------------------------------------------
        # else:
        if self.AD.current_mode(self.MODE) == Modes.TUMBLING:
            """
            (
                SM.current_state == STATES.NOMINAL
                and not DH.get_latest_data("cdh")[CDH_IDX.DETUMBLING_ERROR_FLAG]
                and self.AD.current_mode(self.MODE) == Modes.TUMBLING
            ):"""
            # Do not allow a switch to Detumbling from Low power
            self.MODE = Modes.TUMBLING

        else:
            if self.execution_counter == 2:
                # Turn coils off before measurements to allow time for coils to settle
                # zero_all_coils()
                self.coil_throttle = [0] * 6

            if self.execution_counter < 4:
                # Update Gyro and attitude estimate via propagation
                self.AD.gyro_update(self.time, gyro, update_covariance=True)
                self.execution_counter += 1

            else:
                if not self.AD.initialized:
                    status_1, status_2 = self.AD.initialize_mekf(self.time, gyro, mag, light_sensor_lux_readings, gps_data)
                    if status_1 != StatusConst.OK or status_2 != StatusConst.OK:
                        self.failure_messages.append(
                            StatusConst.get_fail_message(status_1) + " : " + StatusConst.get_fail_message(status_2)
                        )
                else:
                    # Update Each sensor with covariances
                    status_1, status_2 = self.AD.position_update(self.time, gps_data)
                    if status_1 != StatusConst.OK:
                        self.failure_messages.append(
                            StatusConst.get_fail_message(status_1) + " : " + StatusConst.get_fail_message(status_2)
                        )
                    else:
                        status_1, status_2 = self.AD.sun_position_update(self.time, light_sensor_lux_readings, update_covariance=True)
                        if status_1 != StatusConst.OK:
                            self.failure_messages.append(
                                StatusConst.get_fail_message(status_1) + " : " + StatusConst.get_fail_message(status_2)
                            )

                        self.AD.gyro_update(self.time, gyro, update_covariance=True)

                        status_1, status_2 = self.AD.magnetometer_update(self.time, mag, update_covariance=True)
                        if status_1 != StatusConst.OK:
                            self.failure_messages.append(
                                StatusConst.get_fail_message(status_1) + " : " + StatusConst.get_fail_message(status_2)
                            )

                # identify Mode based on current sensor readings
                new_mode = self.AD.current_mode(self.MODE)
                if new_mode != self.MODE:
                    # zero_all_coils()
                    self.coil_throttle = [0] * 6
                    self.MODE = new_mode

                # Run attitude control if not in Low-power
                if self.MODE != Modes.ACS_OFF: # SM.current_state != STATES.LOW_POWER and self.MODE != Modes.ACS_OFF:
                    self.coil_throttle = self.attitude_control()

                # Reset Execution counter
                self.execution_counter = 0

            # Log data
            # NOTE: In detumbling, most of the log will be zeros since very few sensors are queried
            # self.log()

    # ------------------------------------------------------------------------------------------------------------------------------------
    """ Attitude Control Auxiliary Functions """

    # ------------------------------------------------------------------------------------------------------------------------------------
    def attitude_control(self):
        """
        Performs attitude control on the spacecraft
        """

        # Decide which controller to choose
        if self.MODE in [Modes.TUMBLING, Modes.STABLE]:  # B-cross controller
            # Get sensor measurements
            omega_unbiased = self.AD.state[self.AD.omega_idx]  # - self.AD.state[self.AD.bias_idx]
            mag_field_body = self.AD.state[self.AD.mag_field_idx]

            # Control MCMs and obtain coil statuses
            dipole_moment = spin_stabilizing_controller(omega_unbiased, mag_field_body)

        else:  # Sun-pointed controller
            # Get measurements
            sun_pos_body = self.AD.state[self.AD.sun_pos_idx]
            omega_unbiased = self.AD.state[self.AD.omega_idx]  # - self.AD.state[self.AD.omega_idx]
            mag_field_body = self.AD.state[self.AD.mag_field_idx]
            sun_status = self.AD.state[self.AD.sun_status_idx]

            # Perform ACS iff a sun vector measurement is valid
            # i.e., ignore eclipses, insufficient readings etc.
            if sun_status != StatusConst.OK:
                return

            # Control MCMs and obtain coil statuses
            dipole_moment = sun_pointing_controller(sun_pos_body, omega_unbiased, mag_field_body)
        # [TODO:] reinstate coil allocation
        self.coil_status, coil_throttle = mcm_coil_allocator(dipole_moment)
        return coil_throttle
        

    # ------------------------------------------------------------------------------------------------------------------------------------
    """ LOGGING """

    # ------------------------------------------------------------------------------------------------------------------------------------
    def log(self):
        """
        Logs data to Data Handler
        Takes light sensor readings as input since they are not stored in AD
        """
        self.log_data[ADCS_IDX.MODE] = int(self.MODE)
        self.log_data[ADCS_IDX.GYRO_X] = self.AD.state[10]
        self.log_data[ADCS_IDX.GYRO_Y] = self.AD.state[11]
        self.log_data[ADCS_IDX.GYRO_Z] = self.AD.state[12]
        self.log_data[ADCS_IDX.MAG_X] = self.AD.state[16]
        self.log_data[ADCS_IDX.MAG_Y] = self.AD.state[17]
        self.log_data[ADCS_IDX.MAG_Z] = self.AD.state[18]
        self.log_data[ADCS_IDX.SUN_STATUS] = int(self.AD.state[22])
        self.log_data[ADCS_IDX.SUN_VEC_X] = self.AD.state[19]
        self.log_data[ADCS_IDX.SUN_VEC_Y] = self.AD.state[20]
        self.log_data[ADCS_IDX.SUN_VEC_Z] = self.AD.state[21]
        self.log_data[ADCS_IDX.LIGHT_SENSOR_XM] = int(self.AD.state[23]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_XP] = int(self.AD.state[24]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_YM] = int(self.AD.state[25]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_YP] = int(self.AD.state[26]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_ZM] = int(self.AD.state[27]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_ZP1] = int(self.AD.state[28]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_ZP2] = int(self.AD.state[29]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_ZP3] = int(self.AD.state[30]) & 0xFFFF
        self.log_data[ADCS_IDX.LIGHT_SENSOR_ZP4] = int(self.AD.state[31]) & 0xFFFF
        self.log_data[ADCS_IDX.XP_COIL_STATUS] = int(self.coil_status[0])
        self.log_data[ADCS_IDX.XM_COIL_STATUS] = int(self.coil_status[1])
        self.log_data[ADCS_IDX.YP_COIL_STATUS] = int(self.coil_status[2])
        self.log_data[ADCS_IDX.YM_COIL_STATUS] = int(self.coil_status[3])
        self.log_data[ADCS_IDX.ZP_COIL_STATUS] = int(self.coil_status[4])
        self.log_data[ADCS_IDX.ZM_COIL_STATUS] = int(self.coil_status[5])
        self.log_data[ADCS_IDX.ATTITUDE_QW] = self.AD.state[6]
        self.log_data[ADCS_IDX.ATTITUDE_QX] = self.AD.state[7]
        self.log_data[ADCS_IDX.ATTITUDE_QY] = self.AD.state[8]
        self.log_data[ADCS_IDX.ATTITUDE_QZ] = self.AD.state[9]
        # DH.log_data("adcs", self.log_data)

        if self.execution_counter == 0:
            # Empty failure message buffers
            for msg in self.failure_messages:
                self.log_warning(msg)
            self.failure_messages = []

            # Log Gyro Angular Velocities
            self.log_info(f"ADCS Mode : {self.MODE}")
            self.log_info(f"Gyro Ang Vel : {self.log_data[ADCS_IDX.GYRO_X:ADCS_IDX.GYRO_Z + 1]}")
            self.log_info(f"Gyro Bias : {self.AD.state[self.AD.bias_idx]}")
            
            # from hal.configuration import SATELLITE
            # SATELLITE.set_fsw_state(np.concatenate((self.AD.state[0:22], self.AD.true_map)))
