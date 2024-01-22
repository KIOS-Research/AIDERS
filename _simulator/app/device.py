import rospy
import random
import time
import requests
import os
from datetime import datetime

import utils

# ROS messages
from std_msgs.msg import String
from kios.msg import TelemetryDevice

class Device:
    def __init__(self, name, model, ipAddress, frequency, netIp):
        self.name = name
        self.model = model
        self.ipAddress = ipAddress
        self.frequency = frequency
        self.netIp = netIp
        self.latitude = random.uniform(35.15519455, 35.16004460)
        self.longitude = random.uniform(33.37611508, 33.38052320)
        self.heading = random.randint(0,359)
        self.connected = False
        self.previousDirection = "n"
        self.postPhotoInterval = 5

        #publishers
        self.deviceIdsPublisher = rospy.Publisher("/deviceIds", String, queue_size=10)
        self.telemetryPublisher = rospy.Publisher(f"/{self.name}/TelemetryDevice", TelemetryDevice, queue_size=10)

        # subscribers
        rospy.Subscriber(f"/{self.name}/handshake", String, self.handshakeReceived)


    def start(self):
        utils.myPrint(f"Simulating device: {self.name}")
        self.loop()


    def loop(self):
        rate = rospy.Rate(float(self.frequency))  # adjust the publishing rate
        nextPostPhotoTime = time.time() + self.postPhotoInterval
        while True:
            if not self.connected:
                self.connectToPlatform()
            else:
                self.move()
                self.publishTelemetry()
                if time.time() >= nextPostPhotoTime:
                    self.postPhoto()
                    nextPostPhotoTime += self.postPhotoInterval
            rate.sleep()


    def connectToPlatform(self):
        message = f"{{\"Name\":\"{self.name}\",\"Model\":\"{self.model}\",\"Ip\":\"{self.ipAddress}\",\"Connected\":\"True\"}}"
        self.deviceIdsPublisher.publish(message)


    def move(self):
        stepSize = random.uniform(0.000005, 0.000019)
        deltaLatitude, deltaLongitude, direction = utils.randomDirection(stepSize, self.previousDirection)
        self.latitude += deltaLatitude
        self.longitude += deltaLongitude
        self.previousDirection = direction
        heading = utils.randomHeadingIncrement(self.heading, (5,20))
        self.heading = heading


    def publishTelemetry(self):
        telemetryMessage = TelemetryDevice()
        telemetryMessage.latitude = self.latitude
        telemetryMessage.longitude = self.longitude
        telemetryMessage.altitude = 100.0
        telemetryMessage.heading = self.heading
        telemetryMessage.orientation_x = 0.1
        telemetryMessage.orientation_y = 0.2
        telemetryMessage.orientation_z = 0.3
        telemetryMessage.accelerometer_x = 0.4
        telemetryMessage.accelerometer_y = 0.5
        telemetryMessage.accelerometer_z = 0.6
        telemetryMessage.gyroscope_x = 0.7
        telemetryMessage.gyroscope_y = 0.8
        telemetryMessage.gyroscope_z = 0.9
        telemetryMessage.geomagnetic_x = 1.0
        telemetryMessage.geomagnetic_y = 1.1
        telemetryMessage.geomagnetic_z = 1.2
        telemetryMessage.light = 100.0
        telemetryMessage.step = 500.0
        telemetryMessage.pressure = 1013.25
        telemetryMessage.proximity = 0.5
        telemetryMessage.battery_percentage = 75.0
        self.telemetryPublisher.publish(telemetryMessage)


    def postPhoto(self):
        url = f"http://{self.netIp}:8000/postDeviceImg/"

        payload = {
            "deviceName": self.name,
            "latitude": self.latitude,
            "longitude": self.longitude
        }

        image_filename = f"/pics/{random.randint(1, 5)}.jpg" # choose a random photo
        with open(image_filename, "rb") as image_file:
            image_data = image_file.read()

        # rename image file before posting it
        currentDateTime = datetime.now().time()
        formattedDateTime = currentDateTime.strftime('%H-%M-%S')
        new_filename = f"{self.name}_{formattedDateTime}.jpg"
        files = {'image_file': (new_filename, image_data)}

        response = requests.post(url, data=payload, files=files) # Make the POST request
        if response.status_code == 200:
            print(f"Request {url} successful!")
        else:
            print(f"Request {url} failed.")


    def handshakeReceived(self, _msg):
        if _msg.data == "1":
            self.connected = True

