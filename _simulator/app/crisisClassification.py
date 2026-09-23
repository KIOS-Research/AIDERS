import threading
import time
import json
import random
from datetime import datetime, timezone
from kafka import KafkaProducer  # Ensure kafka-python is in requirements.txt

import utils

class CrisisClassification:
    # Constant header values and incident type
    TOPIC_NAME = "Crisis_Classification"
    SENDER = "CERTH_Crisis_Model"
    # INCIDENT_TYPES = ["Human Trafficking", "Cyber Attack", "Natural Disaster", "Terrorist Attack"]
    INCIDENT_TYPES = ["SuspiciousActivity"]
    # ACTION_TYPES = ["Update", "Create", "Alert", "Resolve"]
    SEVERITY_LEVELS = ["High", "Medium", "Low"] 


    def __init__(self, bootsrapServerIp, kafkaTopic):
        self.kafkaTopic = kafkaTopic
        try:
            self.producer = KafkaProducer(
                bootstrap_servers=bootsrapServerIp,
                value_serializer=lambda v: json.dumps(v).encode('utf-8')
            )
            self.connected = True
        except:
            self.connected = False
            utils.myPrint(f"\nWARNING: Kafka broker not reachable at {bootsrapServerIp}")


    def generateRandomCrisisIncident(self):
        now = datetime.now(timezone.utc)
        self.timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        self.incidentID = "incident_" + now.strftime("%Y%m%d%H%M%S")
        self.severityLevel = random.choice(self.SEVERITY_LEVELS)
        self.incidentType = random.choice(self.INCIDENT_TYPES)
        # self.actionType = random.choice(self.ACTION_TYPES)
        self.latitude = round(random.uniform(37.99, 38), 8)  # Latitude range around Athens (37.9838 ± 0.5)
        self.longitude = round(random.uniform(23.99, 24), 8)  # Longitude range around Athens (23.7275 ± 0.5)
        self.publishIncident()


    def generateCrisisIncident(self, _id, _type, _severity):
        now = datetime.now(timezone.utc)
        self.timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        self.incidentID = "incident_" + _id
        self.severityLevel = _severity
        self.incidentType = _type
        # self.actionType = _action
        self.latitude = round(random.uniform(35.14, 35.18), 8)
        self.longitude = round(random.uniform(33.35, 33.39), 8)
        # self.latitude = round(random.uniform(37.99, 38), 8)  # Latitude range around Athens (37.9838 ± 0.5)
        # self.longitude = round(random.uniform(23.99, 24), 8)  # Longitude range around Athens (23.7275 ± 0.5)
        self.publishIncident()


    def publishIncident(self):
        if not self.connected:
            utils.myPrint("\nERROR: Kafka producer not connected. Cannot publish incident.")
            return

        incidentData = {
            "header": {
                "topicName": self.TOPIC_NAME,
                "sentUTC": self.timestamp,
                "sender": self.SENDER,
                # "actionType": self.actionType
            },
            "body": {
                "incidentID": self.incidentID,
                "incidentType": self.incidentType,
                "severityLevel": self.severityLevel,
                "geoLocation": {
                    "latitude": self.latitude,
                    "longitude": self.longitude
                },
                "timestamp": self.timestamp,
                "areaType": "land",
                "timeOfDay": "day",
                "detectionsSummary": {
                    "humans": random.randint(0, 5),
                    "cars": random.randint(0, 5),
                    "skiffs": random.randint(0, 5),
                    "sailboats": random.randint(0, 5),
                    "cruises": random.randint(0, 5)                                      
                }
            }
        }
        
        # Publish directly to Kafka
        self.producer.send(self.kafkaTopic, incidentData)
        self.producer.flush()  # Ensure the message is sent promptly
        utils.myPrint(f"\nCrisis incident published: #{self.incidentID}")