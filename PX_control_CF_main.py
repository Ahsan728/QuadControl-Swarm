import PX_load_constants_Multicopter as load_const
from PX_Control_CF import *

if __name__ == '__main__':
    link_uri = "radio://0/80/2M/E7E7E7E7E7"
    # link_uri = "radio://0/80/2M/E7E7E7E7E8"
    Tsim = 30  # seconds
    sampling_time = 0.15
    Nsim = int(Tsim/sampling_time)
    desired_yaw = 0
    T_coeff = 22.8
    plant = load_const.load_constants_Multicopter(desired_yaw, Nsim, sampling_time, T_coeff)
    sim_para = {'plant': plant}
    simid = 0
    crazyflie = Control_Crazyflie(link_uri, plant, ip_add='192.168.137.160',
                                  data_destination="PX_data_drone_{id_file}.npy".format(id_file=simid))
    np.save('PX_experimental_parameters_{id_file}.npy'.format(id_file=simid), sim_para)

