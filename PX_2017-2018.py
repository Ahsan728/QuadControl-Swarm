import logging
import time
import math
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import numpy as np
from filterpy.kalman import KalmanFilter
import scipy.io

logging.basicConfig(level=logging.ERROR)


class AltHoldExample:
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        self._f1 = open("data_position.csv", "w")
        self._f2 = open("data_stabilizer.csv", "w")
        self._f3 = open("data_parameters.csv", "w")

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

        self._lg_stab = LogConfig(name='Position', period_in_ms=10)
        self._lg_stab.add_variable('kalman.stateX', 'float')
        self._lg_stab.add_variable('kalman.stateY', 'float')
        self._lg_stab.add_variable('kalman.stateZ', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        self._lg_stab1 = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab1.add_variable('stabilizer.pitch', 'float')
        self._lg_stab1.add_variable('stabilizer.roll', 'float')
        self._lg_stab1.add_variable('stabilizer.thrust', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_stab1)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab1.data_received_cb.add_callback(self._stab_log_data1)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # self._lg_stab1.error_cb.add_callback(self._stab_log_error1)
            # Start the logging
            self._lg_stab.start()
            self._lg_stab1.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._hover_test).start()

    def updateX(self, value, targetValue, dt):
        kp = 30.0
        kd = 0.0
        ki = 3
        minOutput = -3.0
        maxOutput = 3.0
        integratorMin = -0.3
        integratorMax = 0.3
        
        global boolSaveParametersX
        if (boolSaveParametersX == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersX = False

        global integral_X, previousError_X
        error = targetValue - value
        integral_X = integral_X + error * dt
        integral_X = max(min(integral_X, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_X) / dt
        i = ki * integral_X

        output = p + d + i
        previousError_X = error

        return max(min(output, maxOutput), minOutput)

    def updateY(self, value, targetValue, dt):
        kp = -30.0
        kd = 0.0
        ki = -3.0
        minOutput = -3.0
        maxOutput = 3.0
        integratorMin = -0.3
        integratorMax = 0.3
        
        global boolSaveParametersY
        if (boolSaveParametersY == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersY = False

        global integral_Y, previousError_Y
        error = targetValue - value
        integral_Y = integral_Y + error * dt
        integral_Y = max(min(integral_Y, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Y) / dt
        i = ki * integral_Y

        output = p + d + i
        previousError_Y = error

        return max(min(output, maxOutput), minOutput)

    def updateZ(self, value, targetValue, dt):
        # KiN = Ki
        # KpN = Kd
        # KpN = Kp + Ki
        # kp = 9000.0
        # kd = 11000.0
        # ki = 3500.0
        kp = 12000.0
        kd = 0.0
        ki = 3500.0
        minOutput = -20000.0
        maxOutput = 20000.0
        integratorMin = -1000.0
        integratorMax = 1000.0
        
        global boolSaveParametersZ
        if (boolSaveParametersZ == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(minOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersZ=False

        global integral_Z, previousError_Z
        error = targetValue - value
        integral_Z = integral_Z + error * dt
        integral_Z = max(min(integral_Z, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Z) / dt
        i = ki * integral_Z

        output = p + d + i
        previousError_Z = error

        return max(min(output, maxOutput), minOutput)

    def updateYaw(self, value, targetValue, dt):
        kp = -3.0
        kd = 0.0
        ki = 0.0
        minOutput = -200.0
        maxOutput = 200.0
        integratorMin = 0.0
        integratorMax = 0.0
        
        global boolSaveParametersYaw
        if (boolSaveParametersYaw == True):
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersYaw = False

        global integral_Yaw, previousError_Yaw
        error = targetValue - value
        integral_Yaw = integral_Yaw + error * dt
        integral_Yaw = max(min(integral_Yaw, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Yaw) / dt
        i = ki * integral_Yaw

        output = p + d + i
        previousError_Yaw = error

        return max(min(output, maxOutput), minOutput)

    def _stab_log_data(self, timestamp, data, logconf):
        x_des = 1.54
        y_des = 1.64
        z_des = 1.0
        yaw_des = 0

        offset_R = 0.0
        offset_P = 0.0
        offset_T = 40000
        dt = 0.01

        xr = data['kalman.stateX']
        yr = data['kalman.stateY']
        zr = data['kalman.stateZ']
        psi = data['stabilizer.yaw'] / 57.296

        global f
        f.predict()
        f.update([[xr],
                  [yr],
                  [zr]])
        A = f.x
        x = xr
        y = yr
        z = zr
        vx = A[3, 0]
        vy = A[4, 0]
        vz = A[5, 0]

        global x_pos, y_pos, z_pos, position, Nmax
        global xref_pos, yref_pos, zref_pos

        # Prefit
        w0 = 1.4;
        B = math.exp(-w0 * dt);

        global k
        k = k + 1;

        if ((k >= 1000)&(k < 4000)):
            t = k - 1000;
            if(t < Nmax):
                xref = x_des + position[1, t]
                yref = y_des + position[2, t]
                zref = z_des + position[3, t]
            else:
                xref = x_des + position[1, Nmax-1]
                yref = y_des + position[2, Nmax-1]
                zref = z_des + position[3, Nmax-1]
        elif ((k >= 4000)):
            xref = x_des + position[1, Nmax - 1]
            yref = y_des + position[2, Nmax - 1]

            z_des = 0.6
            if ((xref_pos == 0) & (yref_pos == 0) & (zref_pos == 0)):
                zref = B * z + (1 - B) * z_des;
            else:
                zref = B * zref_pos + (1 - B) * z_des;
        else:
            if ((xref_pos == 0) & (yref_pos == 0) & (zref_pos == 0)):
                xref = B * x + (1 - B) * x_des;
                yref = B * y + (1 - B) * y_des;
                zref = B * z + (1 - B) * z_des;
            else:
                xref = B * xref_pos + (1 - B) * x_des;
                yref = B * yref_pos + (1 - B) * y_des;
                zref = B * zref_pos + (1 - B) * z_des;
        
        global boolSaveParameters
        if (boolSaveParameters == True):
            self._f3.write("{},".format(dt))
            boolSaveParameters = False

        xref_pos = xref
        yref_pos = yref
        zref_pos = zref

        d_x = (math.cos(psi) * vx + math.sin(psi) * vy)
        d_y = (-math.sin(psi) * vx + math.cos(psi) * vy)
        d_z = vz
        x_pos = x
        y_pos = y
        z_pos = z

        xy_error_0 = math.cos(psi) * (xref - x) + math.sin(psi) * (yref - y)
        xy_error_1 = -math.sin(psi) * (xref - x) + math.cos(psi) * (yref - y)
        xy_error_2 = zref - z

        pitch = offset_P + self.updateX(d_x, xy_error_0, dt)
        roll = offset_R + self.updateY(d_y, xy_error_1, dt)
        thrust = offset_T + (int)(self.updateZ(d_z, xy_error_2, dt))

        yaw = self.updateYaw(psi * 57.296, yaw_des, dt)

        self._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        print('x = %f y = %f z = %f' % (x, y, z))
        #print('roll = %f pitch = %f thrust = %f' % (roll, pitch, thrust))
        #print('vx = %f vy = %f vz = %f' % (vx, vy, vz))

        self._f1.write("\n {},".format(timestamp))
        self._f1.write("{},".format(xr))
        self._f1.write("{},".format(yr))
        self._f1.write("{},".format(zr))
        self._f1.write("{},".format(roll))
        self._f1.write("{},".format(pitch))
        self._f1.write("{},".format(thrust))
        self._f1.write("{},".format(x_des))
        self._f1.write("{},".format(y_des))
        self._f1.write("{},".format(z_des))
        self._f1.write("{},".format(xref))
        self._f1.write("{},".format(yref))
        self._f1.write("{},".format(zref))
        self._f1.write("{},".format(yaw_des))
        self._f1.write("{}".format(psi*57.296))

    def _stab_log_data1(self, timestamp, data, logconf):
        self._f2.write("\n {},".format(timestamp))
        self._f2.write("{},".format(data["stabilizer.pitch"]))
        self._f2.write("{},".format(data["stabilizer.roll"]))
        self._f2.write("{}".format(data["stabilizer.thrust"]))

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

    def _hover_test(self):
        print("sending initial thrust of 0")

        self._cf.commander.send_setpoint(0, 0, 0, 0);
        # send_hover_setpoint
        time.sleep(0.5);

        it = 0

        while it < 470:
            time.sleep(0.1)
            it += 1

        print("Close connection")
        self._f1.close()
        self._f2.close()
        self._f3.close()
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(1);
        self._cf.close_link()


if __name__ == '__main__':
    x_pos = 0;
    y_pos = 0;
    z_pos = 0;
    k = 0;

    xref_pos = 0;
    yref_pos = 0;
    zref_pos = 0;

    # PID X
    integral_X = 0;
    previousError_X = 0;

    # PID Y
    integral_Y = 0;
    previousError_Y = 0;

    # PID Z
    integral_Z = 0;
    previousError_Z = 0;

    # PID Yaw
    integral_Yaw = 0;
    previousError_Yaw = 0;

    # Filter Kalman
    dt = 0.01
    f = KalmanFilter(dim_x=6, dim_z=3)
    f.x = np.array([[2.],
                    [2.],
                    [2.],
                    [0.],
                    [0.],
                    [0.]])  # initial state (location and velocity)

    f.F = np.array([[1., 0., 0., dt, 0., 0.],
                    [0., 1., 0., 0., dt, 0.],
                    [0., 0., 1., 0., 0., dt],
                    [0., 0., 0., 1., 0., 0.],
                    [0., 0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 0., 1.]])  # state transition matrix

    f.H = np.array([[1., 0., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0., 0.],
                    [0., 0., 1., 0., 0., 0.]])  # Measurement function

    G = np.array([[dt / 2, 0., 0.],
                  [0., dt / 2, 0.],
                  [0., 0., dt / 2],
                  [1., 0., 0.],
                  [0., 1., 0.],
                  [0., 0., 1.]])

    a11 = 3 * (10 ** -5)
    a12 = 8 * (10 ** -8)
    Q = np.array([[a11, 0., 0.],
                  [0., a11, 0.],
                  [0., 0., a12]])
    A = G.dot(Q)
    f.Q = A.dot(G.T)

    a21 = 5 * (10 ** -5)
    a22 = 5 * (10 ** -9)
    f.R = np.array([[a21, 0., 0.],
                    [0., a21, 0.],
                    [0., 0., a22]])  # state uncertainty

    PosTracking = scipy.io.loadmat('Position.mat')
    position = PosTracking['Positionref']
    Nmax = position.shape[1]

    boolSaveParameters = True
    boolSaveParametersX= True
    boolSaveParametersY = True
    boolSaveParametersZ = True
    boolSaveParametersYaw = True
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    le = AltHoldExample("radio://0/110/2M")
