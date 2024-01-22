import rospy
import random
import threading
import os
import time

import utils
from drone_mission import DroneMission
from drone_build_map import DroneBuildMap
from drone_lidar import DroneLidar

# ROS messages
from std_msgs.msg import String
from kios.msg import Telemetry, MissionDji, MissionCommandDJI, GpsInput, BuildMap, trisonica_msg, TerminalHardware

activeDroneBuildMap = {}
activeDroneLidar = {}

class Drone:
    def __init__(self, name, model, ipAddress, frequency, netIp, liveStreamActive, index):
        self.name = name
        self.model = model
        self.ipAddress = ipAddress
        self.frequency = frequency
        self.netIp = netIp

        self.homeLatitude = random.uniform(35.14888839, 35.15622871)
        self.homeLongitude = random.uniform(33.38157353, 33.39072217)
        self.latitude = self.homeLatitude
        self.longitude = self.homeLongitude
        self.heading = random.randint(0,359)
        self.altitude = 30 + (index * 10)
        self.connected = False
        self.isInMission = False
        self.mission = None
        self.previousDirection = "n"
        self.liveStreamActive = liveStreamActive

        self.batteryPercentage = 100
        self.batteryTickInterval = random.randint(20,40) # how often to decrease the battery percentage (in seconds)
        self.lastBatteryTick = time.time()

        self.satelliteNumber = (20 * index) + 8
        if self.satelliteNumber > 28:
            self.satelliteNumber = 28

        self.index = index
        self.i = 0

        # publishers
        self.droneIdsPublisher = rospy.Publisher("/droneIds", String, queue_size=10)
        self.telemetryPublisher = rospy.Publisher(f"/{self.name}/Telemetry", Telemetry, queue_size=10)
        self.errorPublisher = rospy.Publisher(f"/{self.name}/Error", String, queue_size=10)
        self.monitoringDataPublisher = rospy.Publisher(f"/{self.name}/TerminalHardware", TerminalHardware, queue_size=10)
        self.weatherDataPublisher = rospy.Publisher(f"/{self.name}/WeatherStation", trisonica_msg, queue_size=10)

        # subscribers
        rospy.Subscriber(f"/{self.name}/handshake", String, self.handshakeReceived)
        rospy.Subscriber(f"/{self.name}/Mission", MissionDji, self.missionReceived)
        rospy.Subscriber(f"/{self.name}/BuildMapRequest", BuildMap, self.buildMapRequestReceived)
        rospy.Subscriber(f"/{self.name}/StartOrStopPointCloud", String , self.lidarRequestReceived)


    def start(self):
        utils.myPrint(f"Simulating drone: {self.name}")
        self.loop()


    def loop(self):
        rate = rospy.Rate(float(self.frequency))  # adjust the publishing rate
        while True:
            if not self.connected:
                self.connectToPlatform()
            else:
                self.decreaseBatteryLevel()
                self.move()
                self.publishTelemetry()
                if self.i % 10 == 0:
                    self.publishMonitoringData()
                # self.publishWeatherData()
                # self.publishError()
                # self.disconnectFromPlatform()
            self.i = self.i + 1
            rate.sleep()


    def connectToPlatform(self):
        msgConnect = f"{{\"DroneName\":\"{self.name}\",\"Model\":\"{self.model}\",\"DroneIp\":\"{self.ipAddress}\",\"Connected\":\"True\",\"cameraName\":\"no_cam\",\"Ballistic\":\"0\",\"Lidar\":\"0\",\"Multispectral\":\"0\",\"WaterSampler\":\"0\",\"WeatherStation\":\"0\"}}"
        self.droneIdsPublisher.publish(msgConnect)


    def disconnectFromPlatform(self):
        msgDisconnect = f"{{\"DroneName\":\"{self.name}\",\"Model\":\"{self.model}\",\"DroneIp\":\"{self.ipAddress}\",\"Connected\":\"False\",\"cameraName\":\"no_cam\",\"Ballistic\":\"0\",\"Lidar\":\"0\",\"Multispectral\":\"0\",\"WaterSampler\":\"0\",\"WeatherStation\":\"0\"}}"
        self.droneIdsPublisher.publish(msgDisconnect)
        self.connected = False
    

    def move(self):
        if self.isInMission:
            if not self.mission.is_over:
                new_lat, new_lon, new_altitude, new_heading = self.mission.update()
                self.latitude = new_lat
                self.longitude = new_lon
                self.altitude = new_altitude
                self.heading = new_heading
            else:
                self.isInMission = False


    def publishTelemetry(self):
        telemetryMessage = Telemetry()
        telemetryMessage.latitude = self.latitude
        telemetryMessage.longitude = self.longitude
        telemetryMessage.altitude = self.altitude
        telemetryMessage.heading = self.heading
        telemetryMessage.velocity = 45.0
        telemetryMessage.gpsSignal = 1
        telemetryMessage.satelliteNumber = self.satelliteNumber
        telemetryMessage.homeLatitude = self.homeLatitude
        telemetryMessage.homeLongitude = self.homeLongitude
        
        state = "Flying" if self.altitude > 0.5 else "Landed"
        if self.isInMission:
            state = "Paused_Mission" if self.mission.is_paused else "In_Mission"
        telemetryMessage.droneState = state
        
        telemetryMessage.batteryPercentage = self.batteryPercentage
        telemetryMessage.gimbalAngle = 90
        self.telemetryPublisher.publish(telemetryMessage)


    def publishMonitoringData(self):
        monitoringMessage = TerminalHardware()
        monitoringMessage.seq = self.i + 1
        monitoringMessage.uid = "d346"

        monitoringMessage.ram_use = random.randint(2000,7000)
        monitoringMessage.ram_max = 7765

        monitoringMessage.swap_use = random.randint(0,3000)
        monitoringMessage.swap_max = 3883
        monitoringMessage.swap_cache = 0

        monitoringMessage.emc_usage = 0

        monitoringMessage.cpu_core_count = 6
        monitoringMessage.cpu_core_usage = [100, 23, 19, 23, 20, 21]
        monitoringMessage.cpu_core_freq = [1420, 1420, 1420, 1420, 1420, 1420]
        monitoringMessage.cpuTemp = 36000
        monitoringMessage.cpuFanRPM = 3398

        monitoringMessage.gr3d_usage = random.randint(10,100)
        monitoringMessage.gr3d_freq = 204000000
        monitoringMessage.gpuTemp = 34000

        self.monitoringDataPublisher.publish(monitoringMessage)


    def decreaseBatteryLevel(self):
        if time.time() - self.lastBatteryTick >= self.batteryTickInterval:
            if self.batteryPercentage > 0:
                self.batteryPercentage -= 1  # decrease battery level
            self.lastBatteryTick = time.time()
            

    def publishError(self):
        # TODO: publishError
        pass


    def publishWeatherData(self):
        # TODO: publishWeatherData
        pass


    def startVideoStreamThread(self):
        input_file = f"/vids/{self.name}.mp4"
        if os.path.exists(input_file):
            rtmp_url = f"rtmp://{self.netIp}/live/{self.name}"
            streaming_thread = threading.Thread(target=utils.streamToRtmp, args=(input_file, rtmp_url))
            streaming_thread.start()
        else:
            print(f"The file '{input_file}' does not exist.")        


    # ROS CALLBACKS #


    def handshakeReceived(self, _msg):
        if _msg.data == "1":
            self.connected = True
            if self.liveStreamActive == 1:
                self.startVideoStreamThread()


    # handle received mission
    def missionReceived(self, _msg):
        cmd = _msg.missionCommand.missionCommand
        utils.myPrint(f"\nMission cmd ({cmd}) for {self.name}")
        
        if cmd == 0:    # start
            if not self.isInMission:
                initial_position = (self.latitude, self.longitude)
                target_coordinates = []
                for coords in _msg.gpsInput:
                    target_coordinates.append((coords.latitude, coords.longitude, coords.altitude))
                    speed = coords.speed # TODO: different speed per point ?
                self.mission = DroneMission(initial_position, self.altitude, target_coordinates, speed, self.frequency)
                self.isInMission = True

        elif cmd == 1:  # cancel
            if self.isInMission:
                self.mission.cancel()

        elif cmd == 2:  # pause
            if self.isInMission:
                self.mission.pause()

        elif cmd == 3:  # resume
            if self.isInMission:
                self.mission.resume()

        else:
            utils.myPrint(f"Error: Invalid mission command!")



    # handle build map request
    def buildMapRequestReceived(self, _msg):
        utils.myPrint(f"{self.name} buildMapRequestReceived")
        if _msg.buildmap == True:
            if not self.name in activeDroneBuildMap:
                droneBuildMap = DroneBuildMap(self, _msg.overlap, self.netIp)
                activeDroneBuildMap[self.name] = droneBuildMap
                activeDroneBuildMap[self.name].start()
                utils.myPrint('start')
        else:
            if self.name in activeDroneBuildMap:
                activeDroneBuildMap[self.name].stop()
                del activeDroneBuildMap[self.name]
                utils.myPrint('stop')
        pass

    def lidarRequestReceived(self, _msg):
        utils.myPrint(f"{self.name} lidarRequestReceived")
        if _msg.data == "START":
            if not self.name in activeDroneLidar:
                droneLidar = DroneLidar(self)
                activeDroneLidar[self.name] = droneLidar
                activeDroneLidar[self.name].start()
                utils.myPrint('start')
        else:
            if self.name in activeDroneLidar:
                activeDroneLidar[self.name].stop()
                del activeDroneLidar[self.name]
                utils.myPrint('stop')
        pass