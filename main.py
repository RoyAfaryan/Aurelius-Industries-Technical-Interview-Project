from pymavlink import mavutil
from c2station import config


# Connect the drone
drone = mavutil.mavlink_connection(config.CONNECTION_STRING)
drone.wait_heartbeat()
print("Connected to drone with system ID:", drone.target_system)

drone.arducopter_arm()
drone.motors_armed_wait()
print("Armed")

drone.set_mode("AUTO")
drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
print("Mission started")

# 4. Ask the drone to send position 3 times per second
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    3,     # rate in Hz
    1)     # 1 = start sending, 0 = stop

# 5. Print each position message as it arrives
while True:
    msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    print("lat:", msg.lat / 1e7, " lon:", msg.lon / 1e7, " alt:", msg.relative_alt / 1000, "m")