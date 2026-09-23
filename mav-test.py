from pymavlink import mavutil
import time



def get_vtol_state(heartbeat, autopilot_type):
    # PX4: VTOL state is encoded in custom_mode (see PX4 source for mapping)
    if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        # PX4 VTOL states (from px4 source):
        # 0: MC, 1: FW, 2: TRANSITION_TO_FW, 3: TRANSITION_TO_MC, 4: UNDEFINED
        vtol_states = {
            0: "Multicopter",
            1: "Fixed Wing",
            2: "Transition to Fixed Wing",
            3: "Transition to Multicopter",
            4: "Undefined"
        }
        # PX4 encodes main mode in custom_mode >> 16, sub_mode in custom_mode >> 24
        # VTOL state is not directly in heartbeat, but you can get flight mode
        # For more accurate VTOL state, you need to listen to uORB or MAVLink extensions
        # Here, we just print the custom_mode for reference
        # return f"PX4 custom_mode: {heartbeat.custom_mode}"
    
        main_mode = (heartbeat.custom_mode >> 16) & 0xFF
        sub_mode = (heartbeat.custom_mode >> 24) & 0xFF
        # Map main_mode to VTOL state (see PX4 source for exact mapping)
        if main_mode == 2:
            return "Multicopter"
        elif main_mode == 1:
            return "Fixed Wing"
        elif main_mode in (3, 4):
            return "Transition"
        else:
            return f"Unknown (main_mode={main_mode}, sub_mode={sub_mode})"
        
    elif autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        # ArduPilot: VTOL state is not directly available in heartbeat
        # You can infer from mode or use MAVLink extensions if available
        return f"ArduPilot mode: {heartbeat.custom_mode}"
    else:
        return "Unknown VTOL state"
    

def px4_mode_string(custom_mode):
    # PX4 encodes main_mode in bits 16-23, sub_mode in bits 24-31

    # ACTUAL RESULTS (DeltaQuad SIM):
    ## Takeoff:             AUTO (TAKEOFF)
    ## Hovering:            AUTO (LOITER)
    ## Loitering:           AUTO (LOITER)
    ## Mission:             AUTO (MISSION)
    ## Mission Ended:       AUTO (MISSION) <-- issue here, should be LOITER
    ## RTH:                 AUTO (RTL)
    ## After RTH & Disarm:  AUTO (RTL) <-- ?
    ## Land:                AUTO (LAND)
    ## After Land & Disarm: AUTO (LAND) <-- ?
    
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF

    px4_main_modes = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        4: "AUTO",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
    }
    px4_sub_modes_auto = {
        1: "READY",
        2: "TAKEOFF",
        3: "LOITER",
        4: "MISSION",
        5: "RTL",
        6: "LAND",
        7: "RTGS",
        8: "FOLLOW_TARGET",
        9: "PRECLAND",
        10: "HOLD",  # PX4 1.14+ HOLD submode
    }

    if main_mode == 4:  # AUTO
        sub_mode_str = px4_sub_modes_auto.get(sub_mode, f"UNKNOWN_AUTO({sub_mode})")
        return f"AUTO ({sub_mode_str})"
    else:
        return px4_main_modes.get(main_mode, f"UNKNOWN({main_mode})")


# master = mavutil.mavlink_connection('tcpin:0.0.0.0:14552')
# master = mavutil.mavlink_connection('udp:169.254.1.130:14552')
# master = mavutil.mavlink_connection('udp:0.0.0.0:8000')
# master = mavutil.mavlink_connection('udp:169.254.1.250:14552')
# master = mavutil.mavlink_connection('tcp:169.254.1.250:14552')
# addr = 'udp:0.0.0.0:14550'

# run and log the output to a file:
# python3 mav-test.py | tee mav-test-output-$(date +%s).txt

addr = 'tcpin:169.254.1.250:14552'
print(f"Connecting to vehicle at {addr}...", flush=True)

# Connect to the vehicle
master = mavutil.mavlink_connection(addr)
master.wait_heartbeat()

autopilot_type = 0

# Get the autopilot type (ArduPilot, PX4, etc.)
while autopilot_type == 0:
    # Wait for a heartbeat message to ensure connection is established
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
    if heartbeat:
        autopilot_type = heartbeat.autopilot
        vehicle_type = heartbeat.type

print(f"\U00002705  CONNECTED!", flush=True)

print(f"Autopilot type: {autopilot_type}", flush=True)
print(f"Vehicle type: {vehicle_type}", flush=True) 

if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    print("This is an ArduPilot vehicle.", flush=True)
elif autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
    print("This is a PX4 vehicle.", flush=True)
else:
    print("Unknown autopilot type.", flush=True)

# print("Checking VTOL state...", flush=True)
# vtol_state = get_vtol_state(heartbeat, autopilot_type)
# print(f"VTOL state: {vtol_state}", flush=True)

mode_mapping = master.mode_mapping()
print("Available modes:", mode_mapping)


# Wait for the first GLOBAL_POSITION_INT message to get initial position and altitude
lat = 0
long = 0
current_alt_amsl = None

while True:
    try:
        out = master.recv_match().to_dict()
        print(out, flush=True)
        # if out.get('mavpackettype')=='GLOBAL_POSITION_INT':
        #     lat = out["lat"]/10000000
        #     long = out["lon"]/10000000
        #     current_alt_amsl = out["alt"] / 1000.0  # Convert mm to meters
    except Exception as e:
        continue

    time.sleep(0.1)

# print(f"Current position: Latitude {lat}, Longitude {long}", flush=True)
# print(f"Current altitude (AMSL): {current_alt_amsl} m", flush=True)





# mission_seq = None
# while True:
#     msg = master.recv_match(type='MISSION_CURRENT', blocking=False)
#     print(msg.seq)
#     mode = master.flightmode
#     px4_mode = px4_mode_string(heartbeat.custom_mode) if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4 else mode
#     print(f"Current flight mode: {mode} (PX4: {px4_mode})", flush=True)
#     print(f"Raw custom_mode: {heartbeat.custom_mode}", flush=True)
#     # print(f"Mode mapping: {mode_mapping}", flush=True)
#     # if msg:
#     #     if mission_seq is None:
#     #         mission_seq = msg.seq
#     #     elif msg.seq != mission_seq:
#     #         print(f"Mission in progress. Current waypoint: {msg.seq}")
#     #         mission_seq = msg.seq
#     time.sleep(1)




# ##########################
# #### TAKEOFF COMMANDS ####
# ##########################

# # arm
# print("Arming vehicle...", flush=True)
# master.mav.command_long_send(master.target_system, master.target_component,
#                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# print("Waiting for vehicle to arm...", flush=True)
# while True:
#     hb = master.recv_match(type='HEARTBEAT', blocking=True)
#     if hb:
#         if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
#             print("Vehicle is armed!", flush=True)
#             break


# # Send takeoff command (altitude in meters)
# takeoff_altitude = 11  # desired takeoff altitude
# takeoff_amsl_altitude = int(current_alt_amsl) + takeoff_altitude  # desired takeoff altitude

# print(f"Sending takeoff command to altitude: {takeoff_altitude} m", flush=True)
# mode_id = master.mode_mapping()["GUIDED"]
# print(f"Setting mode to ({mode_id})", flush=True)
# master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#     0,
#     0, 0, 0, 0, lat, long, int(takeoff_altitude)
# )

# threshold=2.0
# print(f"Waiting to reach target altitude: {takeoff_amsl_altitude} m...", flush=True)
# while True:
#     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
#     if msg:
#         current_alt_amsl = msg.alt / 1000.0  # mm to meters
#         print(f"Current altitude: {current_alt_amsl:.1f} m", flush=True)
#         if abs(current_alt_amsl - takeoff_amsl_altitude) < threshold:
#             print("Target altitude reached.", flush=True)
#             break
#     time.sleep(0.5)



# ##########################
# #### LANDING COMMANDS ####
# ##########################

# print("Sending LAND command...", flush=True)
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_LAND,
#     0,
#     0, 0, 0, 0, 0, 0, 0  # Land at current location
# )

# land_threshold = 2.0  # meters AMSL, adjust as needed for your field elevation
# print("Waiting for vehicle to land...", flush=True)
# while True:
#     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
#     if msg:
#         current_alt_amsl = msg.alt / 1000.0  # mm to meters
#         print(f"Current altitude: {current_alt_amsl:.1f} m", flush=True)
#         if current_alt_amsl < land_threshold:
#             print("Landing complete.", flush=True)
#             break
#     time.sleep(0.5)

    

# ############################
# #### VTOL TRANSITION TEST ####
# ############################

# print(f" transitioning to Fixed Wing...", flush=True)
# try:
#     if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
#         # For Ardupilot, use 1 for VTOL_TRANSITION_TO_FW
#         print("ArduPilot detected, transitioning to Fixed Wing mode...", flush=True)
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
#             0,
#             1,  # 1 = VTOL_TRANSITION_TO_FW for ArduPilot
#             0, 0, 0, 0, 0, 0
#         )
#     elif autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
#         # For PX4, use 4 for VTOL_TRANSITION_TO_FW
#         print("PX4 detected, transitioning to Fixed Wing mode...", flush=True)
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
#             0,
#             4,  # 4 = VTOL_TRANSITION_TO_FW for PX4
#             0, 0, 0, 0, 0, 0
#         )
#     else:
#         print("Unknown autopilot type for VTOL transition.", flush=True)
# except Exception as e:
#     print("ERROR: Transitioning to Fixed Wing failed!", flush=True)
#     print(e, flush=True)


# time.sleep(20)


# print(f" transitioning to Multicopter...", flush=True)
# try:
#     if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
#         # ArduPilot: 2 = VTOL_TRANSITION_TO_MC
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
#             0,
#             2,  # 2 = VTOL_TRANSITION_TO_MC for ArduPilot
#             0, 0, 0, 0, 0, 0
#         )
#     elif autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
#         # PX4: 3 = VTOL_TRANSITION_TO_MC
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
#             0,
#             3,  # 3 = VTOL_TRANSITION_TO_MC for PX4
#             0, 0, 0, 0, 0, 0
#         )
#     else:
#         print("Unknown autopilot type for VTOL transition.", flush=True)
# except Exception as e:
#     print("ERROR: Transitioning to Multicopter failed!", flush=True)
#     print(e, flush=True)