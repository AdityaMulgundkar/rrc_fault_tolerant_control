#!/usr/bin/env python

'''
test mavlink messages
'''
from __future__ import print_function

from pymavlink import mavutil

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(blocking=False)
    print(f"msg: {msg}")
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))

# create a mavlink serial instance
# master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)
master = mavutil.mavlink_connection('udpout:localhost:14558', source_system=1)
print(f"master: {master}")

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

print("Sending all message types")