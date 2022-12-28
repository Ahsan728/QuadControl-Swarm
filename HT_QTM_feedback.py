"""
    Streaming 6Dof from QTM
    updated by Huu-Thinh DO
"""
import numpy as np
import time
import asyncio
import xml.etree.ElementTree as ET
import pkg_resources

import qtm

number_of_bodies = 999
data_from_QTM = {}

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def setup(list_of_bodies):
    # global number_of_bodies
    number_of_bodies = 0
    # Connect to qtm
    connection = await qtm.connect("192.168.1.145")
    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return
    # Take control of qtm, context manager will automatically release control after scope end
    # async with qtm.TakeControl(connection, "password"):
    #     await connection.new()

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)
    def on_packet(packet):
        global data_from_QTM
        data_from_QTM = {}
        info, bodies = packet.get_6d_euler()
        for i in range(len(list_of_bodies)):
            wanted_body = list_of_bodies[i]
            if wanted_body is not None and wanted_body in body_index:
                wanted_index = body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                data_from_QTM[wanted_body] = [position.x/1000,position.y/1000,position.z/1000,rotation.a3] # x y z and psi angle
            else:
                print('ERROR: Name of rigid bodies not found')
                data_from_QTM = 9999
        data_from_QTM['info'] = [packet.framenumber,list_of_bodies]

    await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

async def get_QTM_raw(list_of_bodies):
    await setup(list_of_bodies)

def get_QTM_datas(list_of_bodies, velocity_flag = False, timestamp_flag = False):
    asyncio.run(get_QTM_raw(list_of_bodies))
    data_pre = data_from_QTM
    if velocity_flag:
        feedback_QTM = {}
        asyncio.run(get_QTM_raw(list_of_bodies))
        data_current = data_from_QTM
        for i in range(len(list_of_bodies)):
            curr = data_current[list_of_bodies[i]]
            pree = data_pre[list_of_bodies[i]]
            vel = 100 * (np.array( curr[0:3])  - np.array( pree[0:3] ))/(data_current['info'][0]-data_pre['info'][0])

            if timestamp_flag:
                feedback_QTM[list_of_bodies[i]] = curr[0:3] + vel.tolist() + [curr[3]]+[(data_current['info'][0]-data_pre['info'][0])/100]
            else:
                feedback_QTM[list_of_bodies[i]] = curr[0:3] + vel.tolist() + [curr[3]]
        return feedback_QTM
    else:
        return data_pre

# if __name__ == "__main__":
#     LOB = ['DroneE8', 'Drone']
#     for i in range(20):
#         print(get_QTM_datas(LOB, velocity_flag = True))
