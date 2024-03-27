
import time
import asyncio
import math
import threading
from pymavlink import mavutil

import database.queries
import httpRequests
import utils

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

    def connect(self):
        try:
            addr = self.protocol + ":" + self.ip + ":" + self.port
            print("pymavlink @ " + addr, flush=True)
            self.master = mavutil.mavlink_connection(addr)
            self.master.wait_heartbeat()
            
            print(f"\U00002705 '{self.name}' CONNECTED!", flush=True)

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
                if out.get('mavpackettype')=='GLOBAL_POSITION_INT':
                    self.telemetryObj["latitude"] = out["lat"]/10000000
                    self.telemetryObj["longitude"] = out["lon"]/10000000
                    self.telemetryObj["altitude"] = out["relative_alt"]/1000
                    self.telemetryObj["heading"] = out["hdg"]/100
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
        database.queries.saveDroneTelemetry(self.id, connectionDuration, self.telemetryObj, missionLogId, self.operationId, fov_polygon) # save telemetry data to the database




    #######################
    ####### ACTIONS #######
    #######################


    async def arm(self):
        print(f"Arming '{self.name}'...", flush=True)
        # database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING")
        try:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)
            print(f"'{self.name}' Armed.", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "ARMED")

        except:
            print("ERROR: Arming failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING FAILED")


    # async def disarm(self):
    #     print(f"Disarming '{self.name}'...", flush=True)
    #     try:
    #         await self.system.action.disarm()
    #         async for is_armed in self.system.telemetry.armed():
    #             if not is_armed:
    #                 print(f"'{self.name}' Disarmed.", flush=True)
    #                 database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMED")
    #                 break
    #     except:
    #         print("ERROR: Disarming failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMING FAILED")
            

    def takeoff(self, _altitude):
        print(f"'{self.name}' taking off...", flush=True)
        try:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, self.telemetryObj["latitude"], self.telemetryObj["longitude"], float(_altitude))
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)

            database.queries.saveMavlinkLog(self.id, self.operationId, f"TAKING OFF ({float(_altitude)}m.)")
        except Exception as e:
            print("ERROR: Takeoff failed!", flush=True)
            print(e, flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TAKEOFF FAILED")


    # async def land(self):
    #     print(f"'{self.name}' landing...", flush=True)
    #     try:
    #         await self.system.action.land()        
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING")
    #     except:
    #         print("ERROR: Landing command failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING FAILED")


    # async def returnHome(self):
    #     print(f"'{self.name}' returning home...", flush=True)
    #     try:
    #         await self.system.action.return_to_launch()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME")
    #     except:
    #         print("ERROR: Return home command failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME FAILED")


    # async def transitionToFw(self):
    #     print(f"'{self.name}' transitioning to Fixed Wing...", flush=True)
    #     try:
    #         await self.system.action.transition_to_fixedwing()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW")
    #     except:
    #         print("ERROR: Transitioning to Fixed Wing failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW FAILED")


    # async def transitionToMc(self):
    #     print(f"'{self.name}' Transitioning to Multi Copter...", flush=True)
    #     try:
    #         await self.system.action.transition_to_multicopter()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC")
    #     except:
    #         print("ERROR: Transitioning to Multi Copter failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC FAILED")



    # # set speed
    # async def setSpeed(self, _speed):
    #     print(f"Setting speed for '{self.name}' to {float(_speed)}...", flush=True)
    #     try:
    #         await self.system.action.set_current_speed(float(_speed))
    #         database.queries.saveMavlinkLog(self.id, self.operationId, f"SET SPEED ({float(_speed)}m/s)")
    #     except:
    #         print("ERROR: Set speed failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "SET SPEED FAILED")


    # async def sendMission(self, _missionPoints, _speed):
    #     print(f"Sending mission to '{self.name}'...", flush=True)
    #     database.queries.saveMavlinkLog(self.id, self.operationId, "SENDING MISSION")
    #     try:
    #         # async for terrainInfo in self.system.telemetry.home():
    #         #     absoluteAltitude = terrainInfo.absolute_altitude_m
    #         #     break


    #         # Create a mission plan
    #         mission_items = []

    #         for point in _missionPoints:
    #             item = MissionItem(latitude_deg = point[1],
    #                         longitude_deg = point[0],
    #                         relative_altitude_m = point[2],
    #                         speed_m_s = float(_speed),
    #                         is_fly_through = False,
    #                         gimbal_pitch_deg = -90,
    #                         gimbal_yaw_deg = 0,
    #                         camera_action = MissionItem.CameraAction.NONE,
    #                         loiter_time_s = 0.0,  # Loiter time (in seconds)
    #                         camera_photo_interval_s = 0.0,  # Camera photo interval to use after this mission item (in seconds)
    #                         acceptance_radius_m = 1.0,  # Radius for completing a mission item (in metres)
    #                         yaw_deg = 0.0,  # Absolute yaw angle (in degrees)
    #                         camera_photo_distance_m = 1.0  # Camera photo distance to use after this mission item (in meters)
    #                     )
    #             mission_items.append(item)
    #             print(item, flush=True)

    #         mission_plan = MissionPlan(mission_items)

    #         # Upload the mission to the drone
    #         # await self.system.mission.clear_mission()
    #         await self.system.mission.set_return_to_launch_after_mission(False)
    #         await self.system.mission.upload_mission(mission_plan)

    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION UPLOADED")

    #         print("Waiting for drone to have a global position estimate...", flush=True)
    #         async for health in self.system.telemetry.health():
    #             if health.is_global_position_ok and health.is_home_position_ok:
    #                 print("-- Global position estimate OK", flush=True)
    #                 break

    #         # print("-- Arming")
    #         # await self.system.action.arm()

    #         # Start the mission
    #         await self.system.mission.start_mission()
    #         self.inMission = True

    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION STARTED")

    #         asyncio.ensure_future(self.monitorMissionProgress()) # start monitoring mission progress

    #         print("Mission sent!", flush=True)


    #     except MissionError as e:
    #         print(f"ERROR: Mission command failed - {e}", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION ERROR")

    
    # # pause mission
    # async def pauseMission(self):
    #     print(f"'{self.name}' pausing mission...", flush=True)
    #     try:
    #         await self.system.mission.pause_mission()
    #         self.missionPaused = True
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION PAUSED")
    #     except:
    #         print("ERROR: mission pausing failed!", flush=True)            
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION PAUSE FAILED")


    # # resume mission
    # async def resumeMission(self):
    #     print(f"'{self.name}' resuming mission...", flush=True)
    #     try:
    #         await self.system.mission.start_mission()
    #         self.missionPaused = False
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION RESUMED")
    #     except:
    #         print("ERROR: mission resuming failed!", flush=True)            
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION RESUME FAILED")


    # # cancel mission
    # async def cancelMission(self):
    #     print(f"'{self.name}' cancelling mission...", flush=True)
    #     try:
    #         await self.system.mission.clear_mission()
    #         self.inMission = False
    #         self.missionPaused = False
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION CANCELLED")
    #     except:
    #         print("ERROR: mission cancelling failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION CANCEL FAILED")


    # async def kill(self):
    #     print(f"Killing '{self.name}'...", flush=True)
    #     try:
    #         await self.system.action.kill()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "KILL COMMAND")
    #     except:
    #         print("ERROR: Kill failed!", flush=True)            
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "KILL COMMAND FAILED")


    # async def reboot(self):
    #     print(f"Rebooting '{self.name}'...", flush=True)
    #     try:
    #         await self.system.action.reboot()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "REBOOT")
    #     except:
    #         print("ERROR: Reboot failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "REBOOT FAILED")


    # async def shutdown(self):
    #     print(f"Shutting down '{self.name}'...", flush=True)
    #     try:
    #         await self.system.action.shutdown()
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "SHUTDOWN")
    #     except:
    #         print("ERROR: Shutdown failed!", flush=True)
    #         database.queries.saveMavlinkLog(self.id, self.operationId, "SHUTDOWN FAILED")



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


