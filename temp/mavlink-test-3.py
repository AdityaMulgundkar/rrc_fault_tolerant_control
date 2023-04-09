import time
# from pymavlink import mavutil

import os
import sys
cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/../../../Documents/PX4-Autopilot/src/modules/mavlink")

from mavlink.pymavlink import mavutil
import struct
import numpy as np
import re
from array import array

mavutil.set_dialect("common")

master = mavutil.mavlink_connection('udpin:0.0.0.0:14554')
master.wait_heartbeat()

while True:
    try:
        msg = master.recv_match(blocking=False, timeout=1)
        if not msg:
            continue
        type = msg.get_type()
        # print(f"type: {type}")
        if type == 'UNKNOWN_12921' or type == 'DESIRED_VELOCITY_RATES':
            print(f"{msg}")
            hex_values = msg.data
            # print(f"A: {hex_values}")
            integer_list = [int(x) for x in re.findall(r'\d+', str(hex_values))]
            npa = np.asarray(integer_list, dtype=np.int)
            npb = np.split(npa, 7)
            # print(f"B: {npb}")

            for i in range(0,6):
                byte_data = [49, 10, 105, 122]
                float_value = struct.unpack('>f', byte_data)
                print(float_value)
    except:
        pass

    time.sleep(0.01)
    # print("main loop")
