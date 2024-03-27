
import time
import asyncio
import math
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan, MissionError)
import mavsdk.action

import database.queries
import httpRequests

class Uav:

    def __init__(self, _name, _ip, _port, _model, _operationId, _protocol):
        self.name = _name
        self.ip = _ip
        self.port = _port
        self.model = _model
        self.operationId = _operationId
        self.protocol = _protocol
        self.system = System()

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
            "droneState": "", # Flying, Landed, In_Mission, Paused_Mission
            "gimbalAngle": -90,
            "batteryPercentage": 0,
            "vtolState": "MC", # MC, FW, TRANSITION_TO_FW, TRANSITION_TO_MC
        }

        self.rawLatitude = 0
        self.rawLongitude = 0

    async def connect(self):
        try:
            addr = self.protocol + "://" + self.ip + ":" + self.port
            print("mavsdk @ " + addr, flush=True)
            await asyncio.wait_for(self.system.connect(system_address=addr), timeout=20)
            
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

            # start telemetry loops
            asyncio.ensure_future(self.receivePositionTelemetry()) # start receiving position telemetry
            asyncio.ensure_future(self.receiveVelocityTelemetry()) # start receiving velocity telemetry
            asyncio.ensure_future(self.receiveHeadingTelemetry()) # start receiving heading telemetry
            asyncio.ensure_future(self.receiveSatellitesTelemetry()) # start receiving gps info telemetry
            asyncio.ensure_future(self.receiveBatteryTelemetry()) # start receiving battery telemetry
            asyncio.ensure_future(self.receiveInAirTelemetry()) # start receiving in_air status
            asyncio.ensure_future(self.receiveVtolState()) # start receiving vtol state
            asyncio.ensure_future(self.receiveRawGps())
            asyncio.ensure_future(self.storeTelemetryObject()) # start loop for saving telemetry object to the database
            # asyncio.ensure_future(self.receiveStatus())

            httpRequests.startDroneLiveStreamCapture(droneId, self.name)    # start live stream capture

        except asyncio.TimeoutError:
            print(f"\U0000274C '{self.name}' FAILED TO CONNECT!", flush=True)
            self.connected = False
            raise Exception("Connection to the drone timed out")            

    
    async def disconnect(self):
        print(f"\U0000274C Disconnecting '{self.name}'...")
        database.queries.saveMavlinkLog(self.id, self.operationId, "DISCONNECTED")
        self.connected = False
        database.queries.updateDroneConnectionStatus(self.id, 0)    # flag drone as disconnected
        httpRequests.stopDroneDetector(self.id, self.name)          # stop detection



    #########################
    ####### TELEMETRY #######
    #########################

    async def receivePositionTelemetry(self):
        async for position in self.system.telemetry.position():
            # print("Position:", flush=True)
            # print(position.latitude_deg, position.longitude_deg, position.relative_altitude_m, flush=True)
            if(self.connected == False):
                break
            self.telemetryObj["latitude"] = position.latitude_deg
            self.telemetryObj["longitude"] = position.longitude_deg
            self.telemetryObj["altitude"] = position.relative_altitude_m
            self.telemetryUpdatedAt = time.time()
            

    async def receiveRawGps(self):
        async for raw_gps in self.system.telemetry.raw_gps():
            self.rawLatitude = raw_gps.latitude_deg
            self.rawLongitude = raw_gps.longitude_deg
            self.telemetryUpdatedAt = time.time()
            # print("raw_gps:", raw_gps, flush=True)


    async def receiveVelocityTelemetry(self):
        async for velocity_ned in self.system.telemetry.velocity_ned():
            if(self.connected == False):
                break
            velocity = math.sqrt(velocity_ned.north_m_s**2 + velocity_ned.east_m_s**2 + velocity_ned.down_m_s**2)
            self.telemetryObj["velocity"] = velocity
            self.telemetryUpdatedAt = time.time()


    async def receiveHeadingTelemetry(self):
        async for heading in self.system.telemetry.heading():
            if(self.connected == False):
                break
            self.telemetryObj["heading"] = heading.heading_deg
            self.telemetryUpdatedAt = time.time()


    async def receiveSatellitesTelemetry(self):
        async for gpsInfo in self.system.telemetry.gps_info():
            if(self.connected == False):
                break
            self.telemetryObj["satelliteNumber"] = gpsInfo.num_satellites
            self.telemetryUpdatedAt = time.time()
            # print(gpsInfo.fix_type, flush=True)


    async def receiveBatteryTelemetry(self):
        async for battery in self.system.telemetry.battery():
            # print("battery:", flush=True)
            # print(battery.remaining_percent, flush=True)
            if(self.connected == False):
                break
            self.telemetryObj["batteryPercentage"] = battery.remaining_percent
            self.telemetryUpdatedAt = time.time()
            

    async def receiveInAirTelemetry(self):
        async for inAir in self.system.telemetry.in_air():
            # print(inAir, flush=True)
            if(self.connected == False):
                break
            if(self.inMission):
                self.telemetryObj["droneState"] = "In_Mission" if self.missionPaused == False else "Paused_Mission"
            else:
                if(self.telemetryObj["droneState"] == "Flying" and inAir == False):
                    database.queries.saveMavlinkLog(self.id, self.operationId, "LANDED")
                elif(self.telemetryObj["droneState"] == "Landed" and inAir == True):
                    database.queries.saveMavlinkLog(self.id, self.operationId, "TOOK OFF")
                self.telemetryObj["droneState"] = "Flying" if inAir else "Landed"
            self.telemetryUpdatedAt = time.time()


        
    async def receiveVtolState(self):
        async for vtol_state in self.system.telemetry.vtol_state():
            self.telemetryObj["vtolState"] = vtol_state
            self.telemetryUpdatedAt = time.time()


    async def storeTelemetryObject(self):
        while self.connected:
            if(time.time() - self.telemetryUpdatedAt > 15):
                print(f"\U0001F4A9 '{self.name}' has timed out!", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "TIMED OUT")
                await self.disconnect()
                break

            # print(f"SAVING TELEMETRY {time.time()}", flush=True)

            connectionDuration = round((time.time() - self.connectedAt), 2)
            missionLogId = None
            # TODO:
            # if self.inMission:
            #     mission = database.queries.getDroneMissionLogId(self.id)
            #     missionLogId = mission[0]
            # print("SAVING TELEMETRY", flush=True)

            fov_polygon = []

            # if normal gps data is not available, use raw gps data
            if(self.telemetryObj["latitude"] == 0 and self.telemetryObj["longitude"] == 0):
                self.telemetryObj["latitude"] = self.rawLatitude
                self.telemetryObj["longitude"] = self.rawLongitude

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
            
            database.queries.saveDroneTelemetry(self.id, connectionDuration, self.telemetryObj, missionLogId, self.operationId, fov_polygon) # save telemetry data to the database
            await asyncio.sleep(0.5)

        print(f"\U0001F480 TELEMETRY THREAD FOR '{self.name}' STOPPED!", flush=True)


    async def receiveStatus(self):
        async for status_text in self.system.telemetry.status_text():
            print("Statustext:", status_text, flush=True)



    #######################
    ####### ACTIONS #######
    #######################
            

    async def arm(self):
        print(f"Arming '{self.name}'...", flush=True)
        # database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING")
        try:
            await self.system.action.arm()
            async for is_armed in self.system.telemetry.armed():
                if is_armed:
                    print(f"'{self.name}' Armed.", flush=True)
                    database.queries.saveMavlinkLog(self.id, self.operationId, "ARMED")
                    break
        except:
            print("ERROR: Arming failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "ARMING FAILED")


    async def disarm(self):
        print(f"Disarming '{self.name}'...", flush=True)
        try:
            await self.system.action.disarm()
            async for is_armed in self.system.telemetry.armed():
                if not is_armed:
                    print(f"'{self.name}' Disarmed.", flush=True)
                    database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMED")
                    break
        except:
            print("ERROR: Disarming failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "DISARMING FAILED")
            

    async def takeoff(self, _altitude):
        print(f"'{self.name}' taking off...", flush=True)
        try:
            await self.system.action.set_takeoff_altitude(int(_altitude))
            await self.system.action.takeoff()
            database.queries.saveMavlinkLog(self.id, self.operationId, f"TAKING OFF ({_altitude}m.)")

            # # Wait for the drone to be in the air
            # async for is_in_air in self.system.telemetry.in_air():
            #     if is_in_air:
            #         print("Drone has taken off.", flush=True)
            #         break
        except:
            print("ERROR: Takeoff failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TAKEOFF FAILED")


    async def land(self):
        print(f"'{self.name}' landing...", flush=True)
        try:
            await self.system.action.land()        
            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING")
        except:
            print("ERROR: Landing command failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "LANDING FAILED")


    async def returnHome(self):
        print(f"'{self.name}' returning home...", flush=True)
        try:
            await self.system.action.return_to_launch()
            database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME")
        except:
            print("ERROR: Return home command failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "RETURN HOME FAILED")


    async def transitionToFw(self):
        print(f"'{self.name}' transitioning to Fixed Wing...", flush=True)
        try:
            await self.system.action.transition_to_fixedwing()
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW")
        except:
            print("ERROR: Transitioning to Fixed Wing failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO FW FAILED")


    async def transitionToMc(self):
        print(f"'{self.name}' Transitioning to Multi Copter...", flush=True)
        try:
            await self.system.action.transition_to_multicopter()
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC")
        except:
            print("ERROR: Transitioning to Multi Copter failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "TRANSITION TO MC FAILED")



    # set speed
    async def setSpeed(self, _speed):
        print(f"Setting speed for '{self.name}' to {float(_speed)}...", flush=True)
        try:
            await self.system.action.set_current_speed(float(_speed))
            database.queries.saveMavlinkLog(self.id, self.operationId, f"SET SPEED ({float(_speed)}m/s)")
        except:
            print("ERROR: Set speed failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "SET SPEED FAILED")


    async def sendMission(self, _missionPoints, _speed):
        print(f"Sending mission to '{self.name}'...", flush=True)
        database.queries.saveMavlinkLog(self.id, self.operationId, "SENDING MISSION")
        try:
            # async for terrainInfo in self.system.telemetry.home():
            #     absoluteAltitude = terrainInfo.absolute_altitude_m
            #     break


            # Create a mission plan
            mission_items = []

            for point in _missionPoints:
                item = MissionItem(latitude_deg = point[1],
                            longitude_deg = point[0],
                            relative_altitude_m = point[2],
                            speed_m_s = float(_speed),
                            is_fly_through = False,
                            gimbal_pitch_deg = -90,
                            gimbal_yaw_deg = 0,
                            camera_action = MissionItem.CameraAction.NONE,
                            vehicle_action = MissionItem.VehicleAction.NONE,
                            loiter_time_s = 0.0,  # Loiter time (in seconds)
                            camera_photo_interval_s = 0.0,  # Camera photo interval to use after this mission item (in seconds)
                            acceptance_radius_m = 1.0,  # Radius for completing a mission item (in metres)
                            yaw_deg = 0.0,  # Absolute yaw angle (in degrees)
                            camera_photo_distance_m = 1.0  # Camera photo distance to use after this mission item (in meters)
                        )
                mission_items.append(item)
                print(item, flush=True)

            mission_plan = MissionPlan(mission_items)

            # Upload the mission to the drone
            # await self.system.mission.clear_mission()
            await self.system.mission.set_return_to_launch_after_mission(True)
            await self.system.mission.upload_mission(mission_plan)

            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION UPLOADED")

            print("Waiting for drone to have a global position estimate...", flush=True)
            async for health in self.system.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    print("-- Global position estimate OK", flush=True)
                    break

            # print("-- Arming")
            # await self.system.action.arm()

            # Start the mission
            await self.system.mission.start_mission()
            self.inMission = True

            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION STARTED")

            asyncio.ensure_future(self.monitorMissionProgress()) # start monitoring mission progress

            print("Mission sent!", flush=True)


        except Exception as e:
            print(f"ERROR: Mission command failed - {e}", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION ERROR")

    
    # pause mission
    async def pauseMission(self):
        print(f"'{self.name}' pausing mission...", flush=True)
        try:
            await self.system.mission.pause_mission()
            self.missionPaused = True
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION PAUSED")
        except:
            print("ERROR: mission pausing failed!", flush=True)            
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION PAUSE FAILED")


    # resume mission
    async def resumeMission(self):
        print(f"'{self.name}' resuming mission...", flush=True)
        try:
            await self.system.mission.start_mission()
            self.missionPaused = False
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION RESUMED")
        except:
            print("ERROR: mission resuming failed!", flush=True)            
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION RESUME FAILED")


    # cancel mission
    async def cancelMission(self):
        print(f"'{self.name}' cancelling mission...", flush=True)
        try:
            await self.system.mission.clear_mission()
            self.inMission = False
            self.missionPaused = False
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION CANCELLED")
        except:
            print("ERROR: mission cancelling failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION CANCEL FAILED")


    async def kill(self):
        print(f"Killing '{self.name}'...", flush=True)
        try:
            await self.system.action.kill()
            database.queries.saveMavlinkLog(self.id, self.operationId, "KILL COMMAND")
        except:
            print("ERROR: Kill failed!", flush=True)            
            database.queries.saveMavlinkLog(self.id, self.operationId, "KILL COMMAND FAILED")


    async def reboot(self):
        print(f"Rebooting '{self.name}'...", flush=True)
        try:
            await self.system.action.reboot()
            database.queries.saveMavlinkLog(self.id, self.operationId, "REBOOT")
        except:
            print("ERROR: Reboot failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "REBOOT FAILED")


    async def shutdown(self):
        print(f"Shutting down '{self.name}'...", flush=True)
        try:
            await self.system.action.shutdown()
            database.queries.saveMavlinkLog(self.id, self.operationId, "SHUTDOWN")
        except:
            print("ERROR: Shutdown failed!", flush=True)
            database.queries.saveMavlinkLog(self.id, self.operationId, "SHUTDOWN FAILED")



    ########################
    ######## UTILS #########
    ########################


    async def monitorMissionProgress(self):
        async for mission_progress in self.system.mission.mission_progress():
            if(self.connected == False or self.inMission == False):
                break            
            if(mission_progress.current == mission_progress.total):
                self.inMission = False
                self.missionPaused = False
                print(f"'{self.name}' mission ended!", flush=True)
                database.queries.saveMavlinkLog(self.id, self.operationId, "MISSION ENDED")



    # async def checkConnection(self):
    #     async for connection_state in self.system.core.connection_state():
    #         TEST_self_connected = connection_state.is_connected
    #         if not TEST_self_connected:
    #             print(f"'{self.name}' is NOT connected", flush=True)
    #         else:
    #             print(f"'{self.name}' is connected", flush=True)
    #         await asyncio.sleep(0.5)                