from pymavlink import mavutil

# Create the connection to the top-side computer as companion computer/autopilot
master = mavutil.mavlink_connection('udpout:localhost:14558', source_system=1)

# Send a message for QGC to read out loud
#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
# master.mav.statustext_send(mavutil.mavlink.VEHICLE_CMD_DO_LAND_START,
#                            "QGC will read this".encode())
# master.mav.statustext_send(mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
#                            "1, 1".encode())

while True:
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)