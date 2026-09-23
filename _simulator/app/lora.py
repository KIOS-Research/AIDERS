import random
import time
import threading

import utils
from constants import loraClientNames


class LoraMaster:
    def __init__(self, name, noOfClients, frequency, index):
        self.name = name
        self.noOfClients = noOfClients
        self.frequency = frequency
        self.index = index

    def start(self):
        utils.myPrint(f"Simulating Lora master: {self.name} with {self.noOfClients} clients")
        # TODO: add /wsi/loraConnect/ handler to wsi and connect via WebSocket here

        # spawn clients
        for i, _ in enumerate(range(int(self.noOfClients))):
            nameIndex = i + (int(self.index) * int(self.noOfClients))
            loraClient = LoraClient(loraClientNames[nameIndex], self.name, self.frequency)
            thread = threading.Thread(target=loraClient.start)
            thread.daemon = False
            thread.start()
            time.sleep(0.25)


class LoraClient:
    def __init__(self, name, masterName, frequency):
        self.name = name
        self.masterName = masterName
        self.frequency = frequency
        self.latitude = random.uniform(35.14838605, 35.15367687)
        self.longitude = random.uniform(33.37446854, 33.38183904)
        self.previousDirection = "n"
        self.monitoringMessageInterval = 5

    def start(self):
        self.loop()

    def loop(self):
        rate = float(1 / float(self.frequency))
        nextMonitoringMessage = time.time() + self.monitoringMessageInterval
        while True:
            self.move()
            self.buildTelemetry()
            if time.time() >= nextMonitoringMessage:
                self.buildMonitoringData()
                nextMonitoringMessage += self.monitoringMessageInterval
            time.sleep(rate)

    def move(self):
        stepSize = random.uniform(0.00001, 0.00003)
        deltaLatitude, deltaLongitude, direction = utils.randomDirection(stepSize, self.previousDirection)
        self.latitude += deltaLatitude
        self.longitude += deltaLongitude
        self.previousDirection = direction

    def buildTelemetry(self):
        # TODO: send via WebSocket once a server-side handler exists
        pm = random.uniform(10, 500)
        pm25 = random.uniform(10, 500)
        rssi = random.uniform(-30, -10)
        _ = f"{self.name},{time.time()},{self.latitude},{self.longitude},{pm:.2f},{pm25:.2f},1.11,2.22,3.33,{rssi}"

    def buildMonitoringData(self):
        # TODO: send via WebSocket once a server-side handler exists
        _ = f"{self.name},69,77,88,72"

