
import time
import asyncio
import math
import threading
from pymavlink import mavutil

import database.queries
import httpRequests
import utils
import kafka_broadcaster

class Uav2:

    def __init__(self, _name, _ip, _port, _model, _operationId, _protocol):
        self.name = _name
        self.ip = _ip
        self.port = _port
        self.model = _model
        self.operationId = _operationId
        self.protocol = _protocol
        # self.system = System()
        self.master = None

        self.id = 0
        self.connectedAt = ""
        self.telemetryUpdatedAt = ""
        self.connected = False
        self.inMission = False
        self.missionPaused = False
        self.numberOfMissionWaypoints = 0
        self.lastMissionWaypointLatitude = 0
        self.lastMissionWaypointLongitude = 0   

        self.telemetryObj = {
            "latitude": 0,
            "longitude": 0,
            "altitude": 0,
            "heading": 0, 
            "velocity": 0,
            "gpsSignal": 0, 
            "satelliteNumber": 0, 
            "homeLatitude": 0,
            "homeLongitude": 0, 
            "droneState": "Landed", # Flying, Landed, In_Mission, Paused_Mission
            "gimbalAngle": -90,
            "batteryPercentage": 0,
            "vtolState": "MC", # MC, FW, TRANSITION_TO_FW, TRANSITION_TO_MC
        }

        self.altitude_amsl = 0
        self.base_mode = 0
        self.custom_mode = 0
        self.autopilot_type = 0
        self.vehicle_type = 0


    def connect(self):
        try:
            addr = self.protocol + ":" + self.ip + ":" + self.port
            print("pymavlink @ " + addr, flush=True)
            self.master = mavutil.mavlink_connection(addr)
            self.master.wait_heartbeat()
            
            # Get the autopilot type (ArduPilot, PX4, etc.)
            while self.autopilot_type == 0:
                heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
                if heartbeat:
                    self.autopilot_type = heartbeat.autopilot
                    self.vehicle_type = heartbeat.type

            print(f"\U00002705 '{self.name}' CONNECTED!", flush=True)

            print(f"Autopilot type: {self.autopilot_type}", flush=True)
            print(f"Vehicle type: {self.vehicle_type}", flush=True) 

            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                print("This is an ArduPilot vehicle.", flush=True)
            elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                print("This is a PX4 vehicle.", flush=True)

            else:
                print("Unknown autopilot type.", flush=True)

            # Print available modes
            mode_mapping = self.master.mode_mapping()
            print("Available modes:", mode_mapping)                

            drone = database.queries.getDroneByNameAndModel(self.name, self.model)  # retrieve drone from database
            if drone is not None:
                droneId = drone[0]
                database.queries.updateDroneConnectionStatus(droneId, 1)  # flag drone as connected in the database
            else:
                print(f"ERROR: Drone '{self.name}' does not exist in the database", flush=True)
                raise Exception("ERROR: Drone does not exist in the database")

            self.id = droneId
            self.connected = True
            self.connectedAt = time.time()
            self.telemetryUpdatedAt = time.time()
            database.queries.saveMavlinkLog(self.id, self.operationId, "CONNECTED")
            database.queries.saveDroneTelemetry(self.id, 0, self.telemetryObj, None, self.operationId, []) # init telemetry data in the database
            database.queries.createDroneTelemetryLatest(self.id)
            httpRequests.startDroneLiveStreamCapture(droneId, self.name)    # start live stream capture

            if not utils.threadStarted(self.name):
                thread = threading.Thread(target=self.receiveTelemetry, args=())
                thread.name = self.name
                thread.start()

        except Exception as e:
            print(f"\U0000274C '{self.name}' FAILED TO CONNECT!", flush=True)
            self.connected = False
            raise Exception("Connection to the drone timed out")            

    
    def disconnect(self):
        print(f"\U0000274C Disconnecting '{self.name}'...")
        self.master.close()
        self.connected = False
        database.queries.saveMavlinkLog(self.id, self.operationId, "DISCONNECTED")
        database.queries.updateDroneConnectionStatus(self.id, 0)    # flag drone as disconnected
        httpRequests.stopDroneDetector(self.id, self.name)          # stop detection

    
    #chek vehice type
    async def check_vehicle_type(self):
        try:
            # Wait for the HEARTBEAT message
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
            print(heartbeat)
            vehicle_type = heartbeat.type
            return vehicle_type
        except Exception as e:
            print("ERROR: Unable to determine vehicle type!", flush=True)
            print(e, flush=True)
            return "Error"


    #########################
    ####### TELEMETRY #######
    #########################


    def receiveTelemetry(self):
        counter = 0
        while self.connected:
            try:
                out = self.master.recv_match().to_dict()

                if out.get('mavpackettype','')=='GPS_RAW_INT':
                    self.telemetryObj["velocity"] = out["vel"]/100
                    self.telemetryObj["satelliteNumber"] = out["satellites_visible"]
                elif out.get('mavpackettype')=='GLOBAL_POSITION_INT':
                    self.telemetryObj["latitude"] = out["lat"]/10000000
                    self.telemetryObj["longitude"] = out["lon"]/10000000
                    self.telemetryObj["altitude"] = out["relative_alt"]/1000
                    self.altitude_amsl = out["alt"] / 1000.0  # Convert mm to meters
                    if(self.telemetryObj["altitude"] > 0.5):
                        self.telemetryObj["droneState"] = "Flying"
                        if(self.inMission):
                            self.telemetryObj["droneState"] = "In_Mission" if self.missionPaused == False else "Paused_Mission"                        
                    else:
                        self.telemetryObj["droneState"] = "Landed"
                    self.telemetryObj["heading"] = out["hdg"]/100
                elif out.get('mavpackettype')=='HEARTBEAT':
                    self.base_mode = out["base_mode"]
                    self.custom_mode = out["custom_mode"]
                elif out.get('mavpackettype')=='MISSION_CURRENT':
                    # print(f"Current mission sequence: {out['seq']}/{self.numberOfMissionWaypoints}", flush=True)
                    # print(out, flush=True)
                    if self.inMission and out["seq"] >= self.numberOfMissionWaypoints:
                        # check distance to the last waypoint
                        distanceToLastWaypoint = utils.calculateDistanceInMeters(
                            self.telemetryObj["latitude"], 
                            self.telemetryObj["longitude"], 
                            self.lastMissionWaypointLatitude,
                            self.lastMissionWaypointLongitude
                        )
                        print(f"Distance to last waypoint: {distanceToLastWaypoint}m.", flush=True)
                        if(distanceToLastWaypoint <= 100):
                            print(f"\U0001F4A5 '{self.name}' mission completed!", flush=True)
                            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION COMPLETED")
                            self.inMission = False
                            self.missionPaused = False
                            self.numberOfMissionWaypoints = 0
                            self.lastMissionWaypointLatitude = 0
                            self.lastMissionWaypointLongitude = 0
                

                elif out.get('mavpackettype')=='BATTERY_STATUS':
                    self.telemetryObj["batteryPercentage"] = out["battery_remaining"]

                # TODO: get more telemetry data
                # - state (flying, landed, in mission, paused mission)
                # - VTOL state (MC, FW, transition to FW, transition to MC)
                # - gimbal angles (pitch, yaw)
                # - camera zoom level
                # - identify and separate battery levels (main battery level, fuel tank level, etc.)

                counter += 1
                self.telemetryUpdatedAt = time.time()
                if(counter == 10):
                    # print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)
                    self.storeTelemetryObject()
                    counter = 0
            except:
                if(time.time() - self.telemetryUpdatedAt > 15):
                    print(f"\U0001F4A9 '{self.name}' has timed out!", flush=True)
                    database.queries.saveMavlinkLog(self.id, self.operationId, "TIMED OUT")
                    self.disconnect()
                pass

            time.sleep(0.02)
            

    def storeTelemetryObject(self):

        connectionDuration = round((time.time() - self.connectedAt), 2)
        missionLogId = None
        # TODO:
        # if self.inMission:
        #     mission = database.queries.getDroneMissionLogId(self.id)
        #     missionLogId = mission[0]
        # print("SAVING TELEMETRY", flush=True)

        fov_polygon = []

        # print(self.telemetryObj["latitude"], flush=True)

        if(self.telemetryObj["latitude"] != 0 and self.telemetryObj["longitude"] != 0 and self.telemetryObj["altitude"] != 0):
            from camera_footprint_calculator import CameraFootprintCalculator
            c = CameraFootprintCalculator()
            fov_polygon = c.getBoundingPolygon(
                self.telemetryObj["latitude"], 
                self.telemetryObj["longitude"],   
                math.radians(68),
                math.radians(40),
                self.telemetryObj["altitude"], 
                math.radians(0),
                math.radians(45),
                math.radians(self.telemetryObj["heading"]+180 % 360))
        
        # print(self.telemetryObj, flush=True)
        kafka_broadcaster.broadcastTelemetry(self.name, self.id, self.telemetryObj)
        database.queries.saveDroneTelemetry(self.id, connectionDuration, self.telemetryObj, missionLogId, self.operationId, fov_polygon) # save telemetry data to the database
        database.queries.updateDroneTelemetryLatest(self.id, connectionDuration, self.telemetryObj, missionLogId, self.operationId, fov_polygon)




    #######################
    ####### ACTIONS #######
    #######################


    async def arm(self):
        print(f"Arming '{self.name}'...", flush=True)
        database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING...")
        print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)
        # database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING")
        try:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)

            # TODO: check if the vehicle is armed
            print("Waiting for vehicle to arm...", flush=True)
            while self.connected:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
                if hb:
                    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("Vehicle is armed!", flush=True)
                        break

            database.queries.saveMavlinkLog(self.id, self.operationId, "ARMED")
        except Exception as e:
            print("ERROR: Arming failed!", flush=True)
            print(e, flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING FAILED")


    async def disarm(self):
        print(f"Disarming '{self.name}'...", flush=True)
        database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMING...")
        try:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            while self.connected:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
                if hb:
                    if not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        print("Vehicle is disarmed!", flush=True)
                        break       

            database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMED")
        except Exception as e:
            print("ERROR: Disarming failed!", flush=True)
            print(e, flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMING FAILED")

    
    async def takeoff(self, _altitude):
        try:
            print(f"'{self.name}' taking off ({float(_altitude)}m.)...", flush=True)
            print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)

            # mode = "AUTO"

            # #fixed wing vehicles type :1
            # if self.vehicle_type == 1:
            #     # set auto mode for fixed wing
            #      mode = "AUTO"
                 
            # #other vehicles than fixed wing     
            # if self.vehicle_type in [2, 3, 4, 5, 6, 7]:
            #      # set guided mode for copter
            #      mode = "GUIDED"

            # mode_id = self.master.mode_mapping()[mode]
            # print(f"Setting mode to {mode} ({mode_id})", flush=True)
            # self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

            # out = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            # print(out)


            takeoff_altitude = float(_altitude) + int(self.altitude_amsl)

            # check autopilot type and send the appropriate takeoff command
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # takeoff command for ardupilot
                print("Sending takeoff command for ArduPilot...", flush=True)
                self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, float(0), float(0), float(_altitude))
            elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                # takeoff command for px4
                print("Sending takeoff command for PX4...", flush=True)
                print(f"Takeoff altitude: {takeoff_altitude}m.", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0,
                    self.telemetryObj["latitude"], self.telemetryObj["longitude"], int(takeoff_altitude)
                )
            else:
                print("ERROR: Unknown autopilot type!", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "ERROR: Unknown autopilot type!")
                return


            # takeoff command for using deltaquad simulator
            # self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, int(self.telemetryObj["latitude"]*10000000), int(self.telemetryObj["longitude"]*10000000), float(_altitude))
        
            #takeoff command for ardupilot simulator
            # self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, float(0), float(0), float(_altitude) 
            # )

            print("Takeoff command sent!", flush=True)

            out = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            print(out)
            database.queries.saveMavlinkLog(self.id, self.operationId, f"TAKING OFF ({float(_altitude)}m.)")

            print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)

            # Wait for the vehicle to reach the target altitude
            threshold=2.0
            print(f"Waiting to reach target altitude: {takeoff_altitude} m...", flush=True)
            while self.connected:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    current_alt_amsl = msg.alt / 1000.0  # mm to meters
                    # print(f"Current altitude: {current_alt_amsl:.1f} m", flush=True)
                    if abs(current_alt_amsl - takeoff_altitude) < threshold:
                        print("Target altitude reached.", flush=True)
                        database.queries.saveMavlinkLog(self.id, self.operationId, "TARGET ALTITUDE REACHED")
                        break
                await asyncio.sleep(1)

        except Exception as e:
            print("ERROR: Takeoff failed!", flush=True)
            print(e, flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TAKEOFF FAILED")


    async def land(self):
        print(f"'{self.name}' landing...", flush=True)
        try:
            # mode = "LAND"
            # mode_id = self.master.mode_mapping()[mode]
            # print(f"Setting mode to {mode} ({mode_id})", flush=True)
            # self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
            #                      mode_id)

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0  # Land at current location
            )

            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING...") 
            
        except Exception as e:
            print("ERROR: Landing command failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING FAILED")
            print(e, flush=True)


    async def wait_until_landed(self):
        print("Waiting for landing to complete...", flush=True)
        from pymavlink.dialects.v20.common import MAV_LANDED_STATE_ON_GROUND
        while self.connected:
            msg = self.master.recv_match(type='EXTENDED_SYS_STATE', blocking=False)
            if msg and hasattr(msg, "landed_state"):
                if msg.landed_state == MAV_LANDED_STATE_ON_GROUND:
                    print("Landed confirmed.", flush=True)
                    database.queries.saveMavlinkLog(self.id, self.operationId, "LANDED")
                    break
            await asyncio.sleep(1)



    async def sendMission(self, _missionPoints, _speed):

        # TODO: check if it's in mission and update telemetry accordingly
        try:
            print(f"Sending mission to '{self.name}'...", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "SENDING MISSION...")
            print(_missionPoints)
         
            # Ensure we're in the right mode before mission upload
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # Set to GUIDED mode first to ensure clean state
                mode = "GUIDED"
                mode_id = self.master.mode_mapping()[mode]
                print(f"Setting mode to {mode} ({mode_id}) before mission upload", flush=True)
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
                
                # Wait for mode change acknowledgment
                await asyncio.sleep(1)
            
            # Clear existing mission points with retry mechanism
            max_retries = 3
            for retry in range(max_retries):
                try:
                    print(f"Clearing mission (attempt {retry + 1})...", flush=True)
                    self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
                    
                    # Wait for acknowledgment with timeout
                    start_time = time.time()
                    while time.time() - start_time < 5:  # 5 second timeout
                        out = self.master.recv_match(type='MISSION_ACK', blocking=False)
                        if out:
                            print("Previous mission cleared:", out)
                            if out.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                break
                        await asyncio.sleep(0.1)
                    else:
                        if retry < max_retries - 1:
                            print("Mission clear timeout, retrying...", flush=True)
                            continue
                        else:
                            print("Mission clear failed after retries", flush=True)
                    break
                except Exception as e:
                    print(f"Mission clear attempt {retry + 1} failed: {e}", flush=True)
                    if retry == max_retries - 1:
                        raise

            # Wait a bit to ensure the system is ready
            await asyncio.sleep(1)

            #mission count
            print(f"Sending mission count: {len(_missionPoints)}", flush=True)
            self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(_missionPoints), 0)
            
            # Wait for mission request with timeout
            start_time = time.time()
            out = None
            while time.time() - start_time < 10:  # 10 second timeout
                out = self.master.recv_match(type='MISSION_REQUEST', blocking=False)
                if out:
                    break
                await asyncio.sleep(0.1)
            
            if not out:
                raise Exception("Timeout waiting for MISSION_REQUEST")
            
            print("Received mission request:", out)

            #upload mission

            self.numberOfMissionWaypoints = len(_missionPoints)
            self.lastMissionWaypointLatitude = _missionPoints[-1][1]
            self.lastMissionWaypointLongitude =  _missionPoints[-1][0]            
            
            # Upload waypoints sequentially based on mission requests
            waypoints_uploaded = 0
            total_waypoints = len(_missionPoints)  # Just the mission points, no extra takeoff point
            
            # Handle mission requests sequentially
            while waypoints_uploaded < total_waypoints:
                # Use the initial request or wait for subsequent ones
                if waypoints_uploaded == 0:
                    mission_request = out  # Use the initial MISSION_REQUEST we already received
                else:
                    # Wait for next mission request
                    start_time = time.time()
                    mission_request = None
                    print(f"Waiting for MISSION_REQUEST for sequence {waypoints_uploaded}...", flush=True)
                    while time.time() - start_time < 15:  # Increased timeout to 15 seconds
                        mission_request = self.master.recv_match(type='MISSION_REQUEST', blocking=False)
                        if mission_request:
                            print(f"Received MISSION_REQUEST: {mission_request}", flush=True)
                            break
                        # Also check for any other mission-related messages
                        other_msg = self.master.recv_match(type=['MISSION_ACK', 'STATUSTEXT'], blocking=False)
                        if other_msg:
                            print(f"Received other message: {other_msg}", flush=True)
                        await asyncio.sleep(0.1)
                    
                    if not mission_request:
                        # Check if we received a final mission ack instead of a request
                        print(f"No MISSION_REQUEST received, checking for MISSION_ACK...", flush=True)
                        final_check_start = time.time()
                        while time.time() - final_check_start < 3:  # Increased timeout
                            final_ack = self.master.recv_match(type='MISSION_ACK', blocking=False)
                            if final_ack:
                                print(f"Received MISSION_ACK instead of request: {final_ack}", flush=True)
                                if final_ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                    print("Mission upload completed successfully (early termination)", flush=True)
                                    waypoints_uploaded = total_waypoints  # Exit the loop
                                    break
                                else:
                                    raise Exception(f"Mission upload failed with result: {final_ack.type}")
                            await asyncio.sleep(0.1)
                        
                        if waypoints_uploaded < total_waypoints:
                            # Let's try a different approach - maybe the system expects us to continue
                            print(f"Still no response. Attempting to continue with sequence {waypoints_uploaded}...", flush=True)
                            # Create a fake mission request for the next sequence
                            class FakeMissionRequest:
                                def __init__(self, seq):
                                    self.seq = seq
                            mission_request = FakeMissionRequest(waypoints_uploaded)
                        else:
                            break  # Exit the while loop
                
                # Only process the mission request if we didn't break out of the loop
                if waypoints_uploaded >= total_waypoints:
                    break
                
                seq = mission_request.seq
                print(f"Received request for sequence {seq}", flush=True)
                
                if seq < len(_missionPoints):
                    # Mission waypoints - use the mission points directly
                    longitude, latitude, altitude = _missionPoints[seq]
                    latitude_int = int(latitude * 1E7)
                    longitude_int = int(longitude * 1E7)
                    
                    # For PX4, ensure proper mission item parameters
                    # First waypoint should be marked as current (1, 1), others as (0, 1)
                    current = 1 if seq == 0 else 0
                    autocontinue = 1
                    
                    # For PX4 VTOL in Fixed Wing mode, use appropriate parameters
                    param1 = 0  # Hold time (seconds)
                    param2 = 2  # Acceptance radius (meters) - smaller for better precision
                    param3 = 0  # Pass radius - 0 means fly through waypoint
                    param4 = float('nan')  # Desired yaw angle (NaN = don't change yaw)
                    
                    print(f"Uploading waypoint {seq}: Lat={latitude_int}, Lon={longitude_int}, Alt={altitude}", flush=True)
                    print(f"  Parameters: current={current}, autocontinue={autocontinue}, hold={param1}s, radius={param2}m", flush=True)
                    
                    # Use MISSION_ITEM instead of MISSION_ITEM_INT for better compatibility
                    self.master.mav.mission_item_send(
                        self.master.target_system, 
                        self.master.target_component,
                        seq,  # sequence number
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        current,  # current waypoint
                        autocontinue,  # autocontinue
                        param1,  # hold time
                        param2,  # acceptance radius
                        param3,  # pass radius
                        param4,  # yaw
                        latitude,  # latitude (float)
                        longitude,  # longitude (float)
                        altitude  # altitude (float)
                    )
                    
                    # Wait a brief moment for the message to be processed
                    await asyncio.sleep(0.3)
                    
                    # After uploading, check if we get an immediate acknowledgment
                    immediate_ack_start = time.time()
                    while time.time() - immediate_ack_start < 1:  # Check for 1 second
                        immediate_ack = self.master.recv_match(type='MISSION_ACK', blocking=False)
                        if immediate_ack:
                            print(f"Received immediate MISSION_ACK after waypoint {seq}: {immediate_ack}", flush=True)
                            if immediate_ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                print(f"Mission accepted after waypoint {seq}. Upload complete.", flush=True)
                                waypoints_uploaded = total_waypoints  # Mark as complete
                                break
                            elif immediate_ack.type == mavutil.mavlink.MAV_MISSION_ERROR:
                                raise Exception(f"Mission upload failed at waypoint {seq}: MISSION_ERROR")
                            elif immediate_ack.type == mavutil.mavlink.MAV_MISSION_INVALID:
                                raise Exception(f"Mission upload failed at waypoint {seq}: MISSION_INVALID")
                            elif immediate_ack.type == mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE:
                                raise Exception(f"Mission upload failed at waypoint {seq}: MISSION_INVALID_SEQUENCE")
                            else:
                                raise Exception(f"Mission upload failed at waypoint {seq} with type: {immediate_ack.type}")
                        await asyncio.sleep(0.05)
                    
                else:
                    raise Exception(f"Unexpected sequence number: {seq}")
                
                waypoints_uploaded += 1
                
                # Check if we already completed the mission upload
                if waypoints_uploaded >= total_waypoints:
                    print(f"All waypoints uploaded ({waypoints_uploaded}/{total_waypoints})", flush=True)
                    break

            # Check if we need to wait for a final mission acknowledgment
            if waypoints_uploaded == total_waypoints:
                # Wait for final mission acknowledgment
                print("Waiting for final mission acknowledgment...", flush=True)
                start_time = time.time()
                final_ack = None
                while time.time() - start_time < 10:  # Increased timeout to 10 seconds
                    final_ack = self.master.recv_match(type='MISSION_ACK', blocking=False)
                    if final_ack:
                        print(f"Final mission acknowledgment: {final_ack}", flush=True)
                        if final_ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            break
                        elif final_ack.type == mavutil.mavlink.MAV_MISSION_ERROR:
                            raise Exception("Mission upload failed: MISSION_ERROR")
                        elif final_ack.type == mavutil.mavlink.MAV_MISSION_INVALID:
                            raise Exception("Mission upload failed: MISSION_INVALID")
                        elif final_ack.type == mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE:
                            raise Exception("Mission upload failed: MISSION_INVALID_SEQUENCE")
                        else:
                            raise Exception(f"Mission upload failed with result: {final_ack.type}")
                    await asyncio.sleep(0.1)
                
                if not final_ack:
                    print("Warning: No final acknowledgment received", flush=True)
                elif final_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise Exception(f"Mission upload failed with result: {final_ack.type}")
                else:
                    print("Mission upload confirmed successful", flush=True)
            
            print(f"Successfully uploaded {waypoints_uploaded} waypoints", flush=True)
    
            # Set to AUTO mode before starting mission
            print("Setting vehicle to AUTO mode for mission execution...", flush=True)
            mode = "AUTO"
            mode_id = self.master.mode_mapping()[mode]
            self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            
            # Wait for mode change acknowledgment
            mode_change_start = time.time()
            while time.time() - mode_change_start < 5:
                mode_ack = self.master.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=False)
                if mode_ack:
                    if hasattr(mode_ack, 'custom_mode'):
                        print(f"Current mode: {mode_ack.custom_mode}", flush=True)
                    break
                await asyncio.sleep(0.1)
            
            # Wait a bit for mode change to complete
            await asyncio.sleep(2)
    
            # start mission
            print("Starting mission...", flush=True)
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0, 0)

            # Wait for mission start acknowledgment
            start_ack_timeout = time.time()
            mission_started = False
            while time.time() - start_ack_timeout < 5:
                start_ack = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                if start_ack and start_ack.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                    print(f"Mission start acknowledgment: {start_ack}", flush=True)
                    if start_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        mission_started = True
                        break
                    else:
                        raise Exception(f"Mission start denied: {start_ack.result}")
                await asyncio.sleep(0.1)
            
            if mission_started:
                print("Mission uploaded and started successfully", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION STARTED")
                self.inMission = True
            else:
                print("Warning: Mission start acknowledgment not received", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION START UNCERTAIN")
                self.inMission = True  # Assume it started
            
        except Exception as e:
            print("ERROR: Mission command failed!", flush=True)
            print(e, flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION ERROR")
            print(e, flush=True)


    async def returnHome(self):
        print(f"'{self.name}' returning home...", flush=True)
        try:
            # Send the return home command
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0, 0, 0, 0, 0, 0, 0, 0)
            database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME")
        except Exception as e:
            print("ERROR: Return home command failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME FAILED")
            print(e, flush=True)


    # set speed
    async def setSpeed(self, _speed):
        print(f"Setting speed for '{self.name}' to {float(_speed)}...", flush=True)
        try:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, float(_speed), 0, 0, 0, 0, 0)
            database.queries.saveMavlinkLog(self.id, self.operationId, f"SET SPEED ({float(_speed)}m/s)")
        except Exception as e:
            print("ERROR: Set speed failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "SET SPEED FAILED")
            print(e, flush=True)

    # transitions
    async def transitionToFw(self):

        print(f"'{self.name}' transitioning to Fixed Wing...", flush=True)

        try:
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # For Ardupilot, use 1 for VTOL_TRANSITION_TO_FW
                print("ArduPilot detected, transitioning to Fixed Wing mode...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    0,
                    1,  # 1 = VTOL_TRANSITION_TO_FW for ArduPilot
                    0, 0, 0, 0, 0, 0
                )
            elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                # For PX4, use 4 for VTOL_TRANSITION_TO_FW
                print("PX4 detected, transitioning to Fixed Wing mode...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    0,
                    4,  # 4 = VTOL_TRANSITION_TO_FW for PX4
                    0, 0, 0, 0, 0, 0
                )
            else:
                print("Unknown autopilot type for VTOL transition.", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "ERROR: Unknown autopilot type!")
                return
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW")
        except Exception as e:
            print("ERROR: Transitioning to Fixed Wing failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW FAILED")
            print(e, flush=True)




    async def transitionToMc(self):
        print(f"'{self.name}' Transitioning to Multi Copter...", flush=True)

        try:
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # ArduPilot: 2 = VTOL_TRANSITION_TO_MC
                print("ArduPilot detected, transitioning to Multicopter mode...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    0,
                    2,  # 2 = VTOL_TRANSITION_TO_MC for ArduPilot
                    0, 0, 0, 0, 0, 0
                )
            elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                ## PX4: 3 = VTOL_TRANSITION_TO_MC
                print("PX4 detected, transitioning to Multicopter mode...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    0,
                    3,  # 3 = VTOL_TRANSITION_TO_MC for PX4
                    0, 0, 0, 0, 0, 0
                )
            else:
                print("Unknown autopilot type for VTOL transition.", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "ERROR: Unknown autopilot type!")
                return
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC")
        except Exception as e:
            print("ERROR: Transitioning to Multicopter failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC FAILED")
            print(e, flush=True)



    # ########################
    # ######## UTILS #########
    # ########################


    # async def monitorMissionProgress(self):
    #     async for mission_progress in self.system.mission.mission_progress():
    #         if(self.connected == False or self.inMission == False):
    #             break            
    #         if(mission_progress.current == mission_progress.total):
    #             self.inMission = False
    #             self.missionPaused = False
    #             print(f"'{self.name}' mission ended!", flush=True)
    #             database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION ENDED")


