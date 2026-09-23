import websocket
import random
import threading
import os
import time
import json

import utils
from drone_mission import DroneMission
from drone_build_map import DroneBuildMap
# from drone_lidar import DroneLidar


activeDroneBuildMap = {}
activeDroneLidar = {}

class DroneWS:
    def __init__(self, name, model, ipAddress, frequency, netIp, liveStreamActive, index):
        self.ws_url = "ws://" + netIp + ":" + "8888" + "/wsi/" + name  # WebSocket server URL
        # self.ws_url = "wss://" + netIp + "/wsi/" + name  # WebSocket server URL (ssl)
        self.name = name
        self.model = model
        self.ipAddress = ipAddress
        self.frequency = frequency
        self.netIp = netIp

        # Nicosia
        self.homeLatitude = 35.15698271920874
        self.homeLongitude = 33.37830189376086 + (index * 0.0005)

        # Athens
        # self.homeLatitude = 37.965739698074216
        # self.homeLongitude = 23.779161151930424 + (index * 0.0005)

        # Protaras (sea)
        # self.homeLatitude = 34.962891
        # self.homeLongitude = 34.086274 + (index * 0.0005)

        self.latitude = self.homeLatitude
        self.longitude = self.homeLongitude
        # self.heading = random.randint(0,359)
        self.heading = 1
        self.altitude = 20 + (index * 10)
        self.velocity = 0
        self.connected = False
        self.isInMission = False
        self.mission = None
        self.liveStreamActive = liveStreamActive

        self.batteryPercentage = 100
        self.batteryTickInterval = random.randint(20,40) # how often to decrease the battery percentage (in seconds)
        self.lastBatteryTick = time.time()

        self.satelliteNumber = (20 * index) + 8
        if self.satelliteNumber > 28:
            self.satelliteNumber = 28

        self.gimbalAngle = -90 + (index * 10)

        self.cpuUsage = random.randint(20,80)
        self.cpuTemperature = random.randint(40,80)

        self.index = index
        self.i = 0

        self.demoAltitude = False
        self.demoHeading = False
        self.demoGimbalAngle = False
        self.demoDirection = 1


   
    def start(self):
        utils.myPrint(f"Simulating drone: {self.name}")
        self.loop()


    def loop(self):
        rate = float(1/float(self.frequency))  # adjust the publishing rate
        while True:
            if not self.connected:
                self.connectToPlatform()
                time.sleep(3)
            else:
                self.decreaseBatteryLevel()
                self.move()
                self.publishTelemetry()
                # if self.i % 10 == 0:
                #     self.publishMonitoringData()

            self.i = self.i + 1
            time.sleep(rate)


    def connectToPlatform(self):
        # websocket.enableTrace(True)
        utils.myPrint(self.ws_url)
        self.ws = websocket.WebSocketApp(self.ws_url,
                                    on_open=self.on_open,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)

        # Run the WebSocket connection in a separate thread with SSL verification disabled
        import ssl
        ws_thread = threading.Thread(target=lambda: self.ws.run_forever(
            sslopt={"cert_reqs": ssl.CERT_NONE, "check_hostname": False}
        ))
        ws_thread.daemon = True  # Daemonize thread
        ws_thread.start()


    # def disconnectFromPlatform(self):
    #     self.connected = False
    



    def on_message(self, ws, message):
        print(f"Received message: {message}")
        jsonMsg = json.loads(message)
        if jsonMsg["type"] == "mission":
            self.missionReceived(jsonMsg)
        elif jsonMsg["type"] == "buildmap":
            self.buildMapRequestReceived(jsonMsg)            
        elif jsonMsg["type"] == "notification":
            self.notificationReceived(jsonMsg)

    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.connected = False
        # print("### closed ###")

    def on_open(self, ws):
        self.connected = True
        if self.liveStreamActive == 1: 
            self.startVideoStreamThread()        
        # def run(*args):
        #     for i in range(100):  # Send 100 messages, modify as needed
        #         time.sleep(3)  # Wait for 3 seconds between messages
        #         message = f"Message {i}"
        #         ws.send(message)
        #         print(f"Sent: {message}")
        #     time.sleep(1)
        #     ws.close()
        #     print("Thread terminating...")
        # threading.Thread(target=run).start()


    def move(self):
        if self.isInMission:
            if not self.mission.is_over:
                new_lat, new_lon, new_altitude, new_heading, new_speed = self.mission.update()
                self.latitude = new_lat
                self.longitude = new_lon
                self.altitude = new_altitude
                self.heading = new_heading
                self.velocity = new_speed
            else:
                self.isInMission = False
        else:
            self.velocity = 0
            if self.demoHeading:
                self.heading = (self.heading + 10) % 360 # spin around
            if self.demoGimbalAngle:
                self.gimbalAngle = self.gimbalAngle + (2 * self.demoDirection) # move gimbal up
                if self.gimbalAngle > -10 or self.gimbalAngle < -89:
                    self.demoDirection = -self.demoDirection
            if self.demoAltitude:
                self.altitude = self.altitude + (1 * self.demoDirection) # move up and down
                if self.altitude >= 30 or self.altitude <= 10:
                    self.demoDirection = -self.demoDirection


    def publishTelemetry(self):
        state = "Flying" if self.altitude > 0.5 else "Landed"
        if self.isInMission:
            state = "Paused_Mission" if self.mission.is_paused else "In_Mission"

        telemetryData = {
            "telemetry":{
                 "latitude": self.latitude,
                 "longitude": self.longitude,
                 "altitude": self.altitude,
                 "velocity": self.velocity,
                 "heading": self.heading,
                 "gimbalAngle": self.gimbalAngle,
                 "gpsSignal": 2,
                 "satelliteNumber": self.satelliteNumber,
                 "homeLatitude": self.homeLatitude,
                 "homeLongitude": self.homeLongitude,
                 "droneState": state,
                 "batteryPercentage": self.batteryPercentage
            }
           
        }

        # Convert the dictionary to a JSON-formatted string
        telemetryJson = json.dumps(telemetryData)

        # Send the JSON string through the WebSocket connection
        self.ws.send(telemetryJson)


    # def publishMonitoringData(self):
    #     monitoringMessage = TerminalHardware()
    #     monitoringMessage.seq = self.i + 1
    #     monitoringMessage.uid = "d346"

    #     monitoringMessage.ram_use = random.randint(2000,7000)
    #     monitoringMessage.ram_max = 7765

    #     monitoringMessage.swap_use = random.randint(0,3000)
    #     monitoringMessage.swap_max = 3883
    #     monitoringMessage.swap_cache = 0

    #     monitoringMessage.emc_usage = 0

    #     monitoringMessage.cpu_core_count = 6

    #     self.cpuUsage = self.cpuUsage + random.randint(-5, 5)
    #     self.cpuUsage = 90 if self.cpuUsage > 90 else self.cpuUsage
    #     self.cpuUsage = 20 if self.cpuUsage < 20 else self.cpuUsage

    #     monitoringMessage.cpu_core_usage = [
    #         self.cpuUsage + random.randint(-10, 10),
    #         self.cpuUsage + random.randint(-10, 10),
    #         self.cpuUsage + random.randint(-10, 10),
    #         self.cpuUsage + random.randint(-10, 10),
    #         self.cpuUsage + random.randint(-10, 10),
    #         self.cpuUsage + random.randint(-10, 10)
    #     ]
    #     monitoringMessage.cpu_core_freq = [1420, 1420, 1420, 1420, 1420, 1420]

    #     self.cpuTemperature = self.cpuTemperature + random.randint(-5, 5)
    #     self.cpuTemperature = 20 if self.cpuTemperature < 20 else self.cpuTemperature
    #     monitoringMessage.cpuTemp = self.cpuTemperature * 1000

    #     monitoringMessage.cpuFanRPM = 3398

    #     monitoringMessage.gr3d_usage = random.randint(10,100)
    #     monitoringMessage.gr3d_freq = 204000000
    #     monitoringMessage.gpuTemp = 34000

    #     self.monitoringDataPublisher.publish(monitoringMessage)


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


    # handle received mission
    def missionReceived(self, _msg):
        cmd = _msg['missionCommand']['missionCommandValue']
        utils.myPrint(f"\nMission cmd ({cmd}) for {self.name}")
        
        if cmd == 0:    # start
            if not self.isInMission:
                initial_position = (self.latitude, self.longitude)
                target_coordinates = []
                for coords in _msg["gpsInput"]:
                    target_coordinates.append((coords["latitude"], coords["longitude"], coords["altitude"]))
                    speed = coords["speed"] # TODO: different speed per point ?
                self.mission = DroneMission(initial_position, self.altitude, target_coordinates, speed, self.frequency)
                self.isInMission = True
                utils.myPrint(f"Mission started for {self.name}")
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
        utils.myPrint(f"{self.name} Build Map: {_msg['buildmapCommand']}, Interval: {_msg['interval']}")
        if _msg["buildmapCommand"] == 1:
            if not self.name in activeDroneBuildMap:
                droneBuildMap = DroneBuildMap(self, _msg["interval"], self.netIp)
                activeDroneBuildMap[self.name] = droneBuildMap
                activeDroneBuildMap[self.name].start()
                # utils.myPrint('start')
        else:
            if self.name in activeDroneBuildMap:
                activeDroneBuildMap[self.name].stop()
                del activeDroneBuildMap[self.name]
                # utils.myPrint('stop')
        pass


    # handle notification
    def notificationReceived(self, _msg):
        utils.myPrint(f"{self.name} notification received: {_msg['info']['message']}")
        print(f"Notification for {self.name}: {_msg['info']['message']} from {_msg['info']['sender']}")

