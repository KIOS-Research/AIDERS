
import time
import asyncio
import math
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan, MissionError)
import mavsdk.action

import database.queries
import httpRequests

class Uav:

    # def __init__(self, _name, _ip, _port, _type, _model, _operationId):
    def __init__(self, _name, _ip, _port, _model, _operationId):
        self.name = _name
        self.ip = _ip
        self.port = _port
        self.model = _model
        self.operationId = _operationId
        self.system = System()

        self.id = 0
        self.connectedAt = ""
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
        }

    async def connect(self):
        try:
            addr = "udp://" + self.ip + ":" + self.port
            print(addr, flush=True)
            await asyncio.wait_for(self.system.connect(system_address=addr), timeout=20)
            
            print(f"\U00002705 '{self.name}' CONNECTED!", flush=True)

            drone = database.queries.getDroneByNameAndModel(self.name, self.model)  # retrieve drone from database
            if drone is not None:
                droneId = drone[0]
                database.queries.updateDroneConnectionStatus(droneId, 1)  # flag drone as connected in the database
            else:
                print(f"ERROR: Drone '{self.name}' does not exist in the database", flush=True)
                raise Exception("ERROR: Drone does not exist in the database")
            
                # droneObj = {
                #     "name": self.name,
                #     "operation_id": self.operationId,
                #     "ip": self.ip,
                #     "type": "MAVLINK",
                #     "cameraModel": "no_cam",
                #     "lidarAvailable": 0,
                #     "ballisticAvailable": 0,
                #     "waterSamplerAvailable": 0,
                #     "multispectralAvailable": 0,
                #     "weatherStationAvailable": 0,
                # }
                # droneId = database.queries.saveDrone(droneObj)      # create drone entry in the database
                # database.queries.createDroneDetectionEntry(droneId) # create entry in aiders_detection

            self.id = droneId
            self.connected = True
            self.connectedAt = time.time()
            database.queries.saveDroneTelemetry(self.id, 0, self.telemetryObj, None, self.operationId) # init telemetry data in the database

            # start telemetry loops
            asyncio.ensure_future(self.receivePositionTelemetry()) # start receiving position telemetry
            asyncio.ensure_future(self.receiveVelocityTelemetry()) # start receiving velocity telemetry
            asyncio.ensure_future(self.receiveHeadingTelemetry()) # start receiving heading telemetry
            asyncio.ensure_future(self.receiveSatellitesTelemetry()) # start receiving gps info telemetry
            asyncio.ensure_future(self.receiveBatteryTelemetry()) # start receiving battery telemetry
            asyncio.ensure_future(self.receiveInAirTelemetry()) # start receiving in_air status
            asyncio.ensure_future(self.storeTelemetryObject()) # start loop for saving telemetry object to the database

            httpRequests.startDroneLiveStreamCapture(droneId, self.name)    # start live stream capture

        except asyncio.TimeoutError:
            print(f"\U0000274C '{self.name}' FAILED TO CONNECT!", flush=True)
            self.connected = False
            raise Exception("Connection to the drone timed out")            

    
    async def disconnect(self):
        print(f"Disconnecting '{self.name}'...")
        self.connected = False
        httpRequests.stopDroneDetector(self.id, self.name)          # stop detection
        database.queries.updateDroneConnectionStatus(self.id, 0)    # flag drone as disconnected



    #############
    # TELEMETRY #
    #############
        

    async def receivePositionTelemetry(self):
        async for position in self.system.telemetry.position():
            if(self.connected == False):
                break
            self.telemetryObj["latitude"] = position.latitude_deg
            self.telemetryObj["longitude"] = position.longitude_deg
            self.telemetryObj["altitude"] = position.relative_altitude_m
            

    async def receiveVelocityTelemetry(self):
        async for velocity_ned in self.system.telemetry.velocity_ned():
            if(self.connected == False):
                break
            velocity = math.sqrt(velocity_ned.north_m_s**2 + velocity_ned.east_m_s**2 + velocity_ned.down_m_s**2)
            self.telemetryObj["velocity"] = velocity


    async def receiveHeadingTelemetry(self):
        async for heading in self.system.telemetry.heading():
            if(self.connected == False):
                break
            self.telemetryObj["heading"] = heading.heading_deg 


    async def receiveSatellitesTelemetry(self):
        async for gpsInfo in self.system.telemetry.gps_info():
            if(self.connected == False):
                break
            self.telemetryObj["satelliteNumber"] = gpsInfo.num_satellites    


    async def receiveBatteryTelemetry(self):
        async for battery in self.system.telemetry.battery():
            if(self.connected == False):
                break
            self.telemetryObj["batteryPercentage"] = battery.remaining_percent
            

    async def receiveInAirTelemetry(self):
        async for inAir in self.system.telemetry.in_air():
            # print(inAir, flush=True)
            if(self.connected == False):
                break
            if(self.inMission):
                self.telemetryObj["droneState"] = "In_Mission" if self.missionPaused == False else "Paused_Mission"
            else:
                self.telemetryObj["droneState"] = "Flying" if inAir else "Landed"


    async def storeTelemetryObject(self):
        while True:
            connectionDuration = round((time.time() - self.connectedAt), 2)
            missionLogId = None
            # TODO:
            # if self.inMission:
            #     mission = database.queries.getDroneMissionLogId(self.id)
            #     missionLogId = mission[0]
            # print("SAVING TELEMETRY", flush=True)
            database.queries.saveDroneTelemetry(self.id, connectionDuration, self.telemetryObj, missionLogId, self.operationId) # save telemetry data to the database
            await asyncio.sleep(0.5)



    ###########
    # ACTIONS #
    ###########
            

    async def arm(self):
        print("Arming...", flush=True)
        try:
            await self.system.action.arm()
            async for is_armed in self.system.telemetry.armed():
                if is_armed:
                    print("Armed.", flush=True)
                    break
        except:
            print("ERROR: Arming failed!", flush=True)


    async def disarm(self):
        print("Disarming...", flush=True)
        try:
            await self.system.action.disarm()
            async for is_armed in self.system.telemetry.armed():
                if not is_armed:
                    print("Disarmed.", flush=True)
                    break
        except:
            print("ERROR: Disarming failed!", flush=True)
            

    async def takeoff(self, _altitude):
        print("Taking off...", flush=True)
        try:
            await self.system.action.set_takeoff_altitude(int(_altitude))
            await self.system.action.takeoff()

            # # Wait for the drone to be in the air
            # async for is_in_air in self.system.telemetry.in_air():
            #     if is_in_air:
            #         print("Drone has taken off.", flush=True)
            #         break
        except:
            print("ERROR: Takeoff failed!", flush=True)


    async def land(self):
        print("Landing the drone...", flush=True)
        try:
            await self.system.action.land()        
        except:
            print("ERROR: Landing command failed!", flush=True)


    async def returnHome(self):
        print("Returning home...", flush=True)
        try:
            await self.system.action.return_to_launch()
            
        except:
            print("ERROR: Return home command failed!", flush=True)


    async def transitionToFw(self):
        print("Transitioning to Fixed Wing...", flush=True)
        try:
            await self.system.action.transition_to_fixedwing()
        except:
            print("ERROR: Transitioning to Fixed Wing failed!", flush=True)


    async def transitionToMc(self):
        print("Transitioning to Multi Copter...", flush=True)
        try:
            await self.system.action.transition_to_multicopter()
        except:
            print("ERROR: Transitioning to Multi Copter failed!", flush=True)


    # set speed
    async def setSpeed(self, _speed):
        print(f"Setting speed to {float(_speed)}...", flush=True)
        try:
            await self.system.action.set_current_speed(float(_speed))
        except:
            print("ERROR: Set speed failed!", flush=True)            


    async def sendMission(self, _missionPoints, _speed):
        print("Sending mission...", flush=True)
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
            await self.system.mission.set_return_to_launch_after_mission(False)
            await self.system.mission.upload_mission(mission_plan)


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

            asyncio.ensure_future(self.monitorMissionProgress()) # start monitoring mission progress

            print("Mission sent!", flush=True)


        except MissionError as e:
            print(f"ERROR: Mission command failed - {e}", flush=True)



    async def monitorMissionProgress(self):
        async for mission_progress in self.system.mission.mission_progress():
            if(self.connected == False or self.inMission == False):
                break            
            if(mission_progress.current == mission_progress.total):
                self.inMission = False
                self.missionPaused = False
        print(f"Mission ended!", flush=True)


    # pause mission
    async def pauseMission(self):
        print(f"Pausing mission...", flush=True)
        try:
            await self.system.mission.pause_mission()
            self.missionPaused = True
        except:
            print("ERROR: mission pausing failed!", flush=True)            


    # resume mission
    async def resumeMission(self):
        print(f"Resuming mission...", flush=True)
        try:
            await self.system.mission.start_mission()
            self.missionPaused = False
        except:
            print("ERROR: mission resuming failed!", flush=True)            


    # cancel mission
    async def cancelMission(self):
        print(f"Cancelling mission...", flush=True)
        try:
            await self.system.mission.clear_mission()
            self.inMission = False
            self.missionPaused = False
        except:
            print("ERROR: mission cancelling failed!", flush=True)            


    async def kill(self):
        print("Killing...", flush=True)
        try:
            await self.system.action.kill()
        except:
            print("ERROR: Kill failed!", flush=True)            


    async def reboot(self):
        print("Rebooting...", flush=True)
        try:
            await self.system.action.reboot()
        except:
            print("ERROR: Reboot failed!", flush=True)


    async def shutdown(self):
        print("Shutting down...", flush=True)
        try:
            await self.system.action.shutdown()
        except:
            print("ERROR: Shutdown failed!", flush=True)


    # async def print_status_text(self):
    #     try:
    #         async for status_text in self.system.telemetry.status_text():
    #             print(f"Status: {status_text.type}: {status_text.text}", flush=True)
    #     except asyncio.CancelledError:
    #         return
            
    # async def goTo(self, _lat, _lon, _alt):    
    #     try:
    #         async for terrainInfo in self.system.telemetry.home():
    #             absoluteAltitude = terrainInfo.absolute_altitude_m
    #             break
    #         flyingAlt = absoluteAltitude + _alt # fly drone X meters above the ground plane
    #         await self.system.action.goto_location(_lat, _lon, flyingAlt, 120) # goto_location() takes AMSL altitude
    #     except:
    #         print("ERROR: Go-To command failed!", flush=True)
