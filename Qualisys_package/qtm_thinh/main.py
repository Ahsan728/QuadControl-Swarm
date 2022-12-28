from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import asyncio
import qtm


def on_packet(packet):
    """ Callback function that is called everytime a data packet arrives from QTM """
    print("Framenumber: {}".format(packet.framenumber))
    header, markers = packet.get_3d_markers()
    print("Component info: {}".format(header))
    for marker in markers:
        print("\t", marker)



async def main():
    """ Main function """
    connection = await qtm.connect("192.168.1.145")
    if connection is None:
        return

    async with qtm.TakeControl(connection, "password"):
            await connection.new()
    await connection.stream_frames(components=["3d"], on_packet=on_packet)


if __name__ == "__main__":
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()

