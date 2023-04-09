#!/usr/bin/env python3

# Warning: Only try this in simulation!
#          The direct attitude interface is a low level interface to be used
#          with caution. On real vehicles the thrust values are likely not
#          adjusted properly and you need to close the loop using altitude.

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (
    Attitude, OffboardError, ActuatorControl, ActuatorControlGroup)

async def run():
    """ Does Offboard control using attitude commands. """

    drone = System()
    # # rx_connection = "serial:///dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00-port0:57600"
    # rx_connection = 'serial:///dev/ttyUSB0:460800'
    # await drone.connect(system_address="serial:///dev/ttyACM0:57600", )
    # await drone.connect(system_address=rx_connection)
    await drone.connect(system_address="udp://:14540")


    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break


    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    group1 = ActuatorControlGroup([
            1.0,
            0.9,
            1.0,
            0.9,
            1.0,
            0.9,
            1.0,
            0.9

        ])


    group2 = ActuatorControlGroup([
            0.6,
            0.4,
            0.8,
            0.8,
            0.5,
            0.7,
            0.9,
            0.4

        ])
    
    testing = ActuatorControl ([
            group1, group2
        ])

    await drone.offboard.set_actuator_control(testing)


    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    while True:
        print('In Loop... sending command')

        await asyncio.sleep(2)

        await drone.offboard.set_actuator_control(ActuatorControl([
                group1, group2
            ]))

    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")



if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())