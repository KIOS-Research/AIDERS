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
            "gimbalAngle": -60,
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
            self.master = mavutil.mavlink_connection(addr, timeout=10)  # Added socket timeout (seconds)
            try:
                self.master.wait_heartbeat(timeout=10)  # Added heartbeat wait timeout (seconds)
            except Exception as e:
                print(f"\u274C '{self.name}' heartbeat wait timed out!", flush=True)
                self.connected = False
                raise Exception("Connection to the drone timed out (heartbeat)")
            
            # Get the autopilot type (ArduPilot, PX4, etc.)
            autopilot_timeout = 10  # seconds
            start_time = time.time()
            while self.autopilot_type == 0:
                if time.time() - start_time > autopilot_timeout:
                    print(f"\u274C '{self.name}' autopilot type detection timed out!", flush=True)
                    self.connected = False
                    raise Exception("Timeout waiting for autopilot type (HEARTBEAT)")
                heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if heartbeat:
                    self.autopilot_type = heartbeat.autopilot
                    self.vehicle_type = heartbeat.type
                else:
                    time.sleep(0.1)

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
            # print(f"Drone from database: {drone}", flush=True)
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

            # legacyLSC
            # httpRequests.startDroneLiveStreamCapture(droneId, self.name)    # start live stream capture

            # TODO: call ffrb container to start rebroadcasting the RTMP stream
            droneStreamURL = drone[2]
            if droneStreamURL is not None and droneStreamURL != "":
                print(f"Starting RTMP rebroadcast for '{self.name}' at {droneStreamURL}", flush=True)
                # httpRequests.startStreamRebroadcast(droneStreamURL)

            if not utils.threadStarted(self.name):
                thread = threading.Thread(target=self.receiveTelemetry, args=())
                thread.name = self.name
                thread.start()

        except Exception as e:
            print(f"\U0000274C '{self.name}' FAILED TO CONNECT!", flush=True)
            print(e, flush=True)
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
                        if(distanceToLastWaypoint <= 101):
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
                if(counter == 6):
                    # print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)
                    self.storeTelemetryObject()
                    counter = 0
            except:
                if(time.time() - self.telemetryUpdatedAt > 15):
                    print(f"\U0001F4A9 '{self.name}' has timed out!", flush=True)
                    database.queries.saveMavlinkLog(self.id, self.operationId, "TIMED OUT")
                    self.disconnect()
                pass

            time.sleep(0.01)
            

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
                math.radians(self.telemetryObj["gimbalAngle"]+90),
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
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                mode_id = self.master.mode_mapping()["GUIDED"]
                print(f"Setting mode to ({mode_id})", flush=True)
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
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

    
    async def takeoff(self, _takeoff_altitude):
        try:
            print(f"'{self.name}' taking off ({float(_takeoff_altitude)}m.)...", flush=True)

            takeoff_amsl_altitude = float(_takeoff_altitude) + int(self.altitude_amsl)
            print(f"Takeoff AMSL altitude: {takeoff_amsl_altitude}m.", flush=True)

            # check autopilot type and send the appropriate takeoff command
            if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                # takeoff command for ardupilot
                mode_id = self.master.mode_mapping()["GUIDED"]
                print(f"Setting mode to ({mode_id})", flush=True)
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)                
                print("Sending takeoff command for ArduPilot...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0,
                    float(0), float(0), float(_takeoff_altitude))
            elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                # takeoff command for px4
                print("Sending takeoff command for PX4...", flush=True)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0,
                    self.telemetryObj["latitude"], self.telemetryObj["longitude"], int(takeoff_amsl_altitude)
                )
            else:
                print("ERROR: Unknown autopilot type!", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "ERROR: Unknown autopilot type!")
                return

            print("Takeoff command sent!", flush=True)

            out = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            print(out)
            database.queries.saveMavlinkLog(self.id, self.operationId, f"TAKING OFF ({float(_takeoff_altitude)}m.)")

            print(f"Base mode: {self.base_mode}, Custom mode: {self.custom_mode}", flush=True)

            await asyncio.sleep(1)

            # Wait for the vehicle to reach the target altitude
            threshold=2.0
            print(f"Waiting to reach target altitude: {takeoff_amsl_altitude} m...", flush=True)
            while self.connected:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    current_alt_amsl = msg.alt / 1000.0  # mm to meters
                    # print(f"Current altitude: {current_alt_amsl:.1f} m", flush=True)
                    if abs(current_alt_amsl - takeoff_amsl_altitude) < threshold:
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

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0  # Land at current location
            )

            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING...") 
            
            await self.wait_until_landed()
            

        except Exception as e:
            print("ERROR: Landing command failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING FAILED")
            print(e, flush=True)


    # TODO: fix this for ardupilot (works for px4)
    async def wait_until_landed(self):
        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            print("Waiting for landing to complete...", flush=True)
            land_threshold = 0.5  # meters
            while self.connected:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    if self.telemetryObj["altitude"] < land_threshold:
                        print("Landing complete.", flush=True)
                        database.queries.saveMavlinkLog(self.id, self.operationId, "LANDED")
                        break
                await asyncio.sleep(1)
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:        
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
        try:
            print(f"Sending mission to '{self.name}'...", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "SENDING MISSION...")
            print(_missionPoints)

            # Clear previous mission points            
            # self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
            # out = self.master.recv_match(type='MISSION_ACK', blocking=True)  # Wait for acknowledgment
            # print("Previous mission cleared:", out)

            
 
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
            print("---- Waiting for system to stabilize after mission clear...", flush=True)
            await asyncio.sleep(1)

            # TODO: DeltaQuad crashes here for any subsequent mission upload

            #mission count
            print(f"Sending mission count: {len(_missionPoints)}", flush=True)
            self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(_missionPoints)+2, 0)
            
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
            
            # Send the first waypoint to initialize the mission
            self.master.mav.send(mavutil.mavlink.MAVLink_mission_item_int_message(
                                     self.master.target_system, self.master.target_component,0, 
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
                                     1, 1, 0, 0, 0, 0, int(_missionPoints[0][1] * 1E7), int(_missionPoints[0][0] * 1E7), _missionPoints[0][2]))
      
            
            for i, point in enumerate(_missionPoints, start=0):

                longitude, latitude, altitude = point
                # Convert latitude and longitude to integer representation
                latitude_int = int(latitude * 1E7)
                longitude_int = int(longitude * 1E7)
        
                self.master.mav.send(mavutil.mavlink.MAVLink_mission_item_int_message(
                                    self.master.target_system, self.master.target_component,
                                    i+1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, latitude_int, longitude_int, altitude))

                print(f"Uploaded waypoint {i}: Latitude = {latitude_int}, Longitude = {longitude_int}, Altitude = {altitude}")

            # Re-add the last waypoint to the mission
            self.master.mav.send(mavutil.mavlink.MAVLink_mission_item_int_message(
                                     self.master.target_system, self.master.target_component,0, 
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
                                     1, 1, 0, 0, 0, 0, int(_missionPoints[-1][1] * 1E7), int(_missionPoints[-1][0] * 1E7), _missionPoints[-1][2]))

            out = self.master.recv_match(type='MISSION_ACK', blocking=False)
            print(out)
    
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
                self.inMission = True
            
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

            await self.wait_until_landed()
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


