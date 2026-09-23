import threading
import time
import json
import random
from datetime import datetime, timezone
from kafka import KafkaProducer

class CrisisClassification:
    # Constants
    TOPIC_NAME = "Crisis_Classification"
    SENDER = "CERTH_Crisis_Model"
    INCIDENT_TYPES = ["Human_Trafficking", "Smuggling", "Natural_Disaster", "Terrorist_Attack"]
    ACTION_TYPES = ["Update", "Create", "Alert", "Resolve"]
    SEVERITY_LEVELS = ["High", "Medium", "Low"]

    sequential_id = 0
    seq_lock = threading.Lock()

    def __init__(self):
        self.bootstrap_server = "apps.edutel.uniwa.gr:9092"
        # self.bootstrap_server = "192.168.0.10:9092"  # <-- Hardcoded Kafka broker
        self.kafka_topic = "CrisisClassification"
        self.interval = 3  # seconds between messages
        self.sent_count = 0  # initialize the counter

        self.producer = KafkaProducer(
            bootstrap_servers=self.bootstrap_server,
            value_serializer=lambda v: json.dumps(v).encode('utf-8')
        )

    def start(self):
        print(f"Sending data to Kafka topic '{self.kafka_topic}' every {self.interval} seconds...")
        while True:
            self.publish_incident()
            time.sleep(self.interval)

    def generate_crisis_incident(self):
        now = datetime.now(timezone.utc)
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        incident_id = "incident_" + now.strftime("%Y%m%d%H%M%S")

        with CrisisClassification.seq_lock:
            CrisisClassification.sequential_id += 1
            seq_id = CrisisClassification.sequential_id

        return {
            "header": {
                "topicName": self.TOPIC_NAME,
                "sentUTC": timestamp,
                "sender": self.SENDER,
                "actionType": random.choice(self.ACTION_TYPES)
            },
            "body": {
                "incidentID": incident_id,
                "incidentType": random.choice(self.INCIDENT_TYPES),
                "severityLevel": random.choice(self.SEVERITY_LEVELS),
                "geoLocation": {
                    "latitude": round(random.uniform(37.99, 38), 8),  # Latitude range around Athens (37.9838 ± 0.5)
                    "longitude": round(random.uniform(23.99, 24), 8)  # Longitude range around Athens (23.7275 ± 0.5)
                },
                "timestamp": timestamp
            }
            
        }

    crisis_counter = 0
    def publish_incident(self):
        incident_data = self.generate_crisis_incident()
        incident_id = incident_data['body']['incidentID']
        self.producer.send(self.kafka_topic, incident_data)
        self.producer.flush()
        self.sent_count += 1
        # print(f"{json.dumps(incident_data)}")
        print(f"SENT CC COUNT: {self.sent_count}, INCIDENT ID: {incident_id}")


# --- Run it ---
if __name__ == "__main__":
    try:
        producer = CrisisClassification()
        producer.start()
    except KeyboardInterrupt:
        print("\nStopped by user.")
