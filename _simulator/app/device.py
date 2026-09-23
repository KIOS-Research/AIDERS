import websocket
import random
import threading
import time
import json
import ssl
import requests
import os
from datetime import datetime

import utils


class Device:
    def __init__(self, name, model, ipAddress, frequency, netIp):
        self.ws_url = f"ws://{netIp}:8888/wsi/deviceConnect/{name}"
        self.name = name
        self.model = model
        self.ipAddress = ipAddress
        self.frequency = frequency
        self.netIp = netIp
        self.latitude = random.uniform(35.15519455, 35.16004460)
        self.longitude = random.uniform(33.37611508, 33.38052320)
        self.altitude = 1.0
        self.heading = random.randint(0, 359)
        self.batteryPercentage = 100
        self.connected = False
        self.previousDirection = "n"
        self.postPhotoInterval = 5
        self.ws = None

    def start(self):
        utils.myPrint(f"Simulating device: {self.name}")
        self.loop()

    def loop(self):
        rate = float(1 / float(self.frequency))
        nextPostPhotoTime = time.time() + self.postPhotoInterval
        while True:
            if not self.connected:
                self.connectToPlatform()
                time.sleep(3)
            else:
                self.move()
                self.publishTelemetry()
                if time.time() >= nextPostPhotoTime:
                    self.postPhoto()
                    nextPostPhotoTime += self.postPhotoInterval
            time.sleep(rate)

    def connectToPlatform(self):
        utils.myPrint(f"Device {self.name} connecting to {self.ws_url}")
        self.ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        ws_thread = threading.Thread(
            target=lambda: self.ws.run_forever(
                sslopt={"cert_reqs": ssl.CERT_NONE, "check_hostname": False}
            )
        )
        ws_thread.daemon = True
        ws_thread.start()

    def on_open(self, ws):
        self.connected = True
        utils.myPrint(f"Device {self.name} connected")

    def on_message(self, ws, message):
        utils.myPrint(f"Device {self.name} received: {message}")

    def on_error(self, ws, error):
        utils.myPrint(f"Device {self.name} WS error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.connected = False
        utils.myPrint(f"Device {self.name} disconnected")

    def move(self):
        stepSize = random.uniform(0.000005, 0.000019)
        deltaLatitude, deltaLongitude, direction = utils.randomDirection(stepSize, self.previousDirection)
        self.latitude += deltaLatitude
        self.longitude += deltaLongitude
        self.previousDirection = direction
        self.heading = utils.randomHeadingIncrement(self.heading, (5, 20))

    def publishTelemetry(self):
        telemetryData = {
            "telemetry": {
                "latitude": self.latitude,
                "longitude": self.longitude,
                "altitude": self.altitude,
                "heading": self.heading,
                "batteryPercentage": self.batteryPercentage,
            }
        }
        self.ws.send(json.dumps(telemetryData))

    def postPhoto(self):
        url = f"http://{self.netIp}:8888/postDeviceImg/"

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

