import time
from Patriarch_Add_on import *
import numpy as np


def QS_check(id_qs='192.168.137.160'):
    current_state = np.zeros((6, 1))
    connect_server(id_qs)
    time_tmp = 0
    for i in range(50):
        data = obtain_data(Flag_Speed=1)
        current_state[0] = data[0][0] / 1000
        current_state[1] = data[0][1] / 1000
        current_state[2] = data[0][2] / 1000
        current_state[3] = data[0][6]
        current_state[4] = data[0][7]
        current_state[5] = data[0][8]
        print(np.transpose(current_state))
        print("Sampling time", (data[1] - time_tmp) / 1000000)

        if np.linalg.norm(current_state) >= 100:
            raise ValueError("False feedback position , calibration needed")
        time_tmp = data[1]
        if (data[1] - time_tmp) / 1000000 >= 0.125:
            raise ValueError("Sampling time exceeded, please check the connection")
    disconnect_server()


QS_check()
