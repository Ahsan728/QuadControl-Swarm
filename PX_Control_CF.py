import logging
import time
from threading import Thread

import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import cflib
import HT_verify_config as askconfig
from Patriarch_Add_on import *

logging.basicConfig(level=logging.ERROR)


class Control_Crazyflie:
    def __init__(self, link_uri, plant,
                 ip_add="192.168.137.148", data_destination="PX_data.npy",
                 capture_flag=1):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.is_connected = None
        self._lg_stab = None
        self._cf = None
        self.is_connected = None
        self.plant = plant
        self.flag_is_hovering = False
        self.time_hovering = 0
        self.current_yaw = plant['yaw']
        self.PS = capture_flag  # 0 for Bitcraze, 1 for Qualisys
        self.sampling_time = plant['Te']
        self.data_to_save = np.empty((0, 10))

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
        self.test_connection_CF()

    def test_connection_CF(self):
        while not self.flag_data_received:
            time.sleep(self.plant['Te'])
        print("Connection is ready")
        for i in range(self.plant['Nsim']):
            x0 = self.current_state
            yaw = self.current_yaw
            input_controls = np.array([0, 0, 0])
            print("current state x = ", x0)
            self.data_to_save = np.append(self.data_to_save
                                          [np.hstack((x0, yaw, input_controls))],
                                          axis=0
                                          )

            time.sleep(self.sampling_time)
        data = {'result': self.data_to_save}
        np.save(self.data_destination, data)
        print('Done testing, data saved at ', self.data_destination)

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
            Thread(target=self.create_log_Qualisys_PS).start()

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
            self.current_state[0] = data[0][0] / 1000
            self.current_state[1] = data[0][1] / 1000
            self.current_state[2] = data[0][2] / 1000
            self.current_state[3] = data[0][6]
            self.current_state[4] = data[0][7]
            self.current_state[5] = data[0][8]
            # self.timestamp = float(data[1] / 1000000)

            if not self.flag_data_received:
                self.flag_data_received = True

            time.sleep(self.sampling_time / 5)
        disconnect_server()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_error1(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
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
