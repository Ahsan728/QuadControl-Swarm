import logging
import time
from threading import Thread

import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import cflib
# import csv
# import math
# import numpy as np
# from casadi import *
import HT_verify_config as askconfig
from HT_get_ref_pred_horz import *

# from Patriarch_Prev import *
from Patriarch_Add_on import *

from HT_static_functions import *

logging.basicConfig(level=logging.ERROR)


def get_ref_landing(Te):
    # n_step = int(5 / Te)
    tt = numpy.arange(0, 5, Te)
    v_ref_land = np.stack([tt * 0, tt * 0, tt * 0]).transpose()
    x_ref_land = np.stack([0 * tt, 0 * tt, 0.125 + 0 * tt, 0 * tt, 0 * tt, 0 * tt]).transpose()
    Nland = np.size(x_ref_land, 0)
    return x_ref_land, v_ref_land, Nland


def Thrust_to_PWM(Thrust):
    pwm_signal = 65535 * (-140.5e-3 + sqrt(140.5e-3 ** 2 - 4 * 0.409e-3 * (-0.099 - Thrust))) / (2 * 0.409e-3 * 256)
    return pwm_signal


class Control_Crazyflie:
    def __init__(self, link_uri, solver, solver_variables, plant, controller,
                 simulator, ip_add="192.168.137.148", data_destination="data.npy"):
        self.v_end = None
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.u_end = None
        self.landing_target = None
        self.is_connected = None
        self._lg_stab = None
        self._cf = None
        self.is_connected = None
        self.solver = solver
        self.solver_variables = solver_variables
        self.plant = plant
        self.controller = controller
        self.simulator = simulator

        self.flag_is_hovering = False
        self.time_hovering = 0
        self.current_yaw = plant['yaw']
        self.PS = 1  # 0 for Bitcraze, 1 for Qualisys
        self.sampling_time = plant['Te']
        self.coeffThrust = 5700 * 8.1
        self.x_ref_land, self.v_ref_land, self.Nland = get_ref_landing(self.sampling_time)
        # self.coeffThrust = 5700  # Depends on the battery => empiric search
        # https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/
        # self.output_file = open("HT_experiment_data.csv", "w")
        # self.HT_create_output_file_header()
        # self.output_file = open(data_destination, "a")
        # self.output_write = csv.writer(self.output_file)
        self.output_data = np.empty((0, 11))
        self.flag_save_output = True
        self.data_destination = data_destination
        self.current_state = np.array([0., 0., 0., 0., 0., 0.])  # To update this line
        self.timestamp = 0
        self.flag_data_received = False

        if self.PS == 1:
            pos_sys = "Qualisys"
            self.address_ip = ip_add
        else:
            pos_sys = "Bitcraze"
        askconfig.config_verification(ip_add, link_uri, pos_sys, self.sampling_time)
        self.connect_Crazyflie(link_uri)
        self.control_Crazyflie()

    def connect_Crazyflie(self, link_uri):
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        self.is_connected = True
        # Variable used to keep main loop occupied until disconnect
        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        print("Sending initial thrust of 0")
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.5)

        if self.PS == 0:  # Get the data from Bitcraze PS
            self.create_log_Bitcraze_PS()
        elif self.PS == 1:  # Get the data from Qualisys
            Thread(target=self.create_log_Qualisys_PS).start()  # do not touch without Martin

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._limit_time_connection).start()
        print('Counting down with 100s of fly time')

    def create_log_Bitcraze_yaw(self):
        self._lg_stab = LogConfig(name='stateEstimateData', period_in_ms=1000 * self.plant['Te'])
        self._lg_stab.add_variable('stateEstimate.yaw', 'float')
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_yaw)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add stateEstimate log config, bad configuration.')

    def create_log_Bitcraze_PS(self):
        # Position is measured in every Te seconds = period_in_ms=1000 * self.plant['Te'] (ms)
        self._lg_stab = LogConfig(name='stateEstimateData', period_in_ms=1000 * self.plant['Te'])
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stateEstimate.vx', 'float')
        self._lg_stab.add_variable('stateEstimate.vy', 'float')
        self._lg_stab.add_variable('stateEstimate.vz', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add stateEstimate log config, bad configuration.')

        time.sleep(3)
        print("Finished with create_log_Bitcraze_PS")

    def _stab_log_yaw(self, timestamp, data, logconf):
        self.current_yaw = data['stateEstimate.yaw']
        if not self.flag_data_received:
            self.flag_data_received = True

    def _stab_log_data(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        vx = data['stateEstimate.vx']
        vy = data['stateEstimate.vy']
        vz = data['stateEstimate.vz']

        self.current_state = np.array([x, y, z, vx, vy, vz])
        self.timestamp = timestamp

        if not self.flag_data_received:
            self.flag_data_received = True

    def create_log_Qualisys_PS(self):
        self.create_log_Bitcraze_yaw()
        connect_server(self.address_ip)
        while self.is_connected:
            data = obtain_data(Flag_Speed=1)
            tmp = self.current_state
            self.current_state[0] = data[0][0] / 1000
            self.current_state[1] = data[0][1] / 1000
            self.current_state[2] = data[0][2] / 1000
            self.current_state[3] = data[0][6]
            self.current_state[4] = data[0][7]
            self.current_state[5] = data[0][8]
            # self.timestamp = float(data[1] / 1000000)
            if np.linalg.norm(self.current_state) > 10000:
                print("QS msg: calibration needed")
                data = {'result': self.output_data}
                np.save(self.data_destination, data)
                self.current_state = tmp
                # self._cf.commander.send_position_setpoint(self.current_state[0], self.current_state[0], 0.1, 0)
                self._cf.commander.send_setpoint(0, 0,
                                                 0, 0)
                time.sleep(1)
                self._cf.close_link()
            if not self.flag_data_received:
                self.flag_data_received = True

            time.sleep(self.sampling_time / 4)  # 4
        disconnect_server()

    def control_Crazyflie(self):
        # Wait until the data is recived from the positioning system
        while not self.flag_data_received:
            time.sleep(self.plant['Te'])
            self._cf.commander.send_setpoint(0, 0, 0, 0)
        print("Started to receive positioning data => control started")

        k = 0
        x_ref_all = self.simulator['x_ref_all']
        v_ref_all = self.simulator['v_ref']
        v_controls = np.array([self.plant['g'], 0, 0])
        # controls_hover = v_controls
        k_stop = 999999
        N_sim = np.size(x_ref_all, 0)
        Kf = -np.array([[2.5, 0, 0, 1.5, 0, 0],
                            [0, 2.5, 0, 0, 1.5, 0],
                            [0, 0, 2.5, 0, 0, 1.5]])
        while k < N_sim:
            tic = time.time()
            x0 = self.current_state

            x_ref_khanh = x_ref_all[k,:]
            v_ref_khanh = v_ref_all[k,:]
            # print(x_ref_khanh)
            # print(v_ref_khanh)
            # print('curr',x0)
            

            v_khanh = v_ref_khanh + np.matmul(Kf, x0 - x_ref_khanh)
            controls = self.get_real_input(v_khanh)
            controls[0] = sat(controls[0],0, 15)
            controls[1] = sat(controls[1],-np.pi/18, np.pi/18)
            controls[2] = sat(controls[2],-np.pi/18, np.pi/18)
            print(controls)
            #Huu Thinh's controller=========================================================
            # x_ref = get_reference_for_pred_horz(x_ref_all,
            #                                     k,
            #                                     self.controller['Npred'])
            # v_ref = get_virtual_input_ref(v_ref_all, k, self.controller['Npred'])
            # x0 = self.current_state
            # v0 = v_controls
            # v_controls = self.generate_controls(x0, v0, x_ref, v_ref, self.plant['yaw'])
            # controls = self.get_real_input(v_controls)
            #===============================================================================
            if time.time() - tic > self.plant['Te']:
                print('***************************Taking to long to find the control*************')
                print('***************************** Prepare to land ****************************')
                print('**************************************************************************')
                k_stop = k
                k = 10000000000000
            else:
                Thrust_pwm = int(
                    self.plant["Thrust constant"] * Thrust_to_PWM(controls[0] / self.plant['g']))
                Roll = (controls[1] * 180) / np.pi
                Pitch = (controls[2] * 180) / np.pi
                Yaw = (self.plant['yaw'] * 180) / np.pi

                controls_cf = [Roll, Pitch, Yaw, Thrust_pwm]

                print(time.time()-tic)

                self.output_data = np.append(self.output_data,
                                             [np.hstack((x0, controls, self.current_yaw, time.time() - tic))],
                                             axis=0)
                self._cf.commander.send_setpoint(controls_cf[0], controls_cf[1],
                                                 controls_cf[2], controls_cf[3])
                print('Progress {xx} %'.format(xx=round(k*100/N_sim,2)))
                k = k + 1
                time.sleep(self.plant['Te'] - (time.time() - tic))
        if k_stop == 999999:
            k_stop = k
        print('---------------progress stop at {pct}%--------------'.format(pct= \
                                                                                round(k_stop * 100 / np.size(x_ref_all,
                                                                                                             0), 2)))
        print("Last reference point sent => Closing connection")
        self.v_end = v_controls
        print("Landing.............")
        data = {'result': self.output_data}
        np.save(self.data_destination, data)
        self.landing()
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self.flag_save_output = False
        self.is_connected = False
        self._cf.close_link()
        time.sleep(0.5)

    def get_real_input(self, v_controls):
        yaw = self.current_yaw * np.pi / 180
        # yaw = self.plant['yaw']
        T = round(sqrt(v_controls[0] ** 2 + v_controls[1] ** 2 + (v_controls[2] + self.plant['g']) ** 2), 5)
        phi = round(arcsin((v_controls[0] * sin(yaw) - v_controls[1] * cos(yaw)) / T), 5)
        theta = round(arctan((v_controls[0] * cos(yaw) + v_controls[1] * sin(yaw)) / (
                v_controls[2] + self.plant['g'])), 5)
        return [T, phi, theta]

    def get_virtual_input(self, u_controls):
        T = u_controls[0]
        phi = u_controls[1]
        tta = u_controls[2]
        psi = self.plant["yaw"]
        v1 = T * (cos(phi) * sin(tta) * cos(psi) + sin(phi) * sin(psi))
        v2 = T * (cos(phi) * sin(tta) * sin(psi) - sin(phi) * cos(psi))
        v3 = -self.plant['g'] + T * cos(phi) * cos(tta)

        return np.array([v1, v2, v3])

    def generate_controls(self, x0, v0, x_ref, v_ref_k, yaw):
        X = self.solver_variables['X']
        X_init = self.solver_variables['X_init']
        v = self.solver_variables['v']
        v_init = self.solver_variables['v_init']
        # U = self.solver_variables['U']
        # U_init = self.solver_variables['U_init']
        X_ref = self.solver_variables['X_ref']
        v_ref = self.solver_variables['v_ref']
        # yaw_par = self.solver_variables['yaw']

        # v_initial = np.tile(self.plant['v_init'], self.controller['Npred'])
        # v_initial = v_initial.reshape(len(self.plant['v_init']),
        #                               self.controller['Npred'])
        # v_initial = np.tile(self.plant['v_equilibrium'], self.controller['Npred'])
        # v_initial = v_initial.reshape(len(self.plant['v_equilibrium']),
        #                               self.controller['Npred'])
        # x_initial = np.tile(x0, self.controller['Npred'] + 1)
        # x_initial = x_initial.reshape(self.plant['dx'], self.controller['Npred'] + 1)

        self.solver.set_value(X_init, x0)
        self.solver.set_value(X_ref, x_ref)
        self.solver.set_value(v_init, v0)
        self.solver.set_value(v_ref, v_ref_k)
        # self.solver.set_value(yaw_par, yaw)

        # initial guess for the OP - QP
        self.solver.set_initial(v, v_ref_k)
        self.solver.set_initial(X, x_ref)

        sol = self.solver.solve()

        # Check if the solution is found. If not, return hovering control value
        # if np.size(sol.value):
        #     controls = self.get_real_input(sol.value(v[:, 0]))
        # else:
        #     print("************ Solution not found! ************")
        #     controls = self.plant['v_equilibrium']
        #     print("************* Hovering instead **************")
        return sol.value(v[:, 0])

    def landing(self):
        print("Coming home ------------------------------")
        v0 = self.v_end
        for i in range(self.Nland):
            x_ref = get_reference_for_pred_horz(self.x_ref_land,
                                                i,
                                                self.controller['Npred'])
            v_ref = get_virtual_input_ref(self.v_ref_land, i, self.controller['Npred'])
            x0 = self.current_state
            v_controls = self.generate_controls(x0, v0, x_ref, v_ref, self.plant['yaw'])
            controls = self.get_real_input(v_controls)
            Thrust_pwm = int(self.plant["Thrust constant"] * Thrust_to_PWM(controls[0] / self.plant['g']))
            Roll = (controls[1] * 180) / np.pi
            Pitch = (controls[2] * 180) / np.pi
            controls_cf = [Roll, Pitch, self.plant['yaw'], Thrust_pwm]
            self._cf.commander.send_setpoint(controls_cf[0], controls_cf[1],
                                             controls_cf[2], controls_cf[3])
            v0 = v_controls
            time.sleep(self.sampling_time)
        print("Done landing . . . ")

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_error1(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _limit_time_connection(self):
        it = 0
        max_time = 100
        while it < max_time:
            time.sleep(1)
            it += 1

        print("Maximum flight time ({maxtime}) passed => Close connection".format(maxtime=max_time))
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        time.sleep(1)
        self._cf.close_link()


    