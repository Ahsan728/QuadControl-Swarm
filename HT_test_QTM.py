import HT_QTM_feedback as QTMget

LOB = ['DroneE8', 'Drone']
for i in range(20):
    print(QTMget.get_QTM_datas(LOB, velocity_flag = True,timestamp_flag=True))
