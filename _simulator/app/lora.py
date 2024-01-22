import rospy
import random
import time
import threading

import utils
from constants import loraClientNames

# ROS messages
from std_msgs.msg import String

class LoraMaster:
    def __init__(self, name, noOfClients, frequency, index):
        self.name = name
        self.noOfClients = noOfClients
        self.frequency = frequency
        self.connected = False
        self.index = index

        # publishers
        self.deviceIdsPublisher = rospy.Publisher("/loraIds", String, queue_size=10)
        
        # subscribers
        rospy.Subscriber(f"/{self.name}/handshake", String, self.handshakeReceived)

    def start(self):
        utils.myPrint(f"Simulating Lora master: {self.name} with {self.noOfClients} clients")

        message = f"{self.name},True"

        rate = rospy.Rate(float(self.frequency))  # adjust the publishing rate
        while not self.connected:
            self.deviceIdsPublisher.publish(message)
            rate.sleep()

        # spawn clients
        for i, _ in enumerate(range(int(self.noOfClients))):
            nameIndex = i+(int(self.index)*int(self.noOfClients))
            loraClient = LoraClient(loraClientNames[nameIndex], self.name, self.frequency)
            thread = threading.Thread(target=loraClient.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            time.sleep(0.25)        

    def handshakeReceived(self, _msg):
        if _msg.data == "1":
            self.connected = True


class LoraClient:
    def __init__(self, name, masterName, frequency):
        self.name = name
        self.masterName = masterName
        self.frequency = frequency
        self.latitude = random.uniform(35.14838605, 35.15367687)
        self.longitude = random.uniform(33.37446854, 33.38183904)
        self.previousDirection = "n"
        self.monitoringMessageInterval = 5

        # publishers
        self.telemetryPublisher = rospy.Publisher(f"/{self.masterName}/TelemetryLora", String, queue_size=10)
        self.monitorPublisher = rospy.Publisher(f"/{self.masterName}/MonitorLora", String, queue_size=10)

    def start(self):
        self.loop()

    def loop(self):
        rate = rospy.Rate(float(self.frequency))  # adjust the publishing rate
        nextMonitoringMessage = time.time() + self.monitoringMessageInterval
        while True:
            self.move()
            self.publishTelemetry()
            if time.time() >= nextMonitoringMessage:
                self.publishMonitoringData()
                nextMonitoringMessage += self.monitoringMessageInterval            
            rate.sleep()
    
    def move(self):
        stepSize = random.uniform(0.00001, 0.00003)
        deltaLatitude, deltaLongitude, direction = utils.randomDirection(stepSize, self.previousDirection)
        self.latitude += deltaLatitude
        self.longitude += deltaLongitude
        self.previousDirection = direction

    def publishTelemetry(self):
        # telemetryMessage = f"{self.name},{time.time()},{self.latitude},{self.longitude},1111,2222,1.11,2.22"
        telemetryMessageNew = f"{self.name},{time.time()},{self.latitude},{self.longitude},1111,2222,1.11,2.22,3.33,7777"
        self.telemetryPublisher.publish(telemetryMessageNew)
        
    def publishMonitoringData(self):
        monitoringMessage = f"{self.name},69,77,88,72"
        self.monitorPublisher.publish(monitoringMessage)

