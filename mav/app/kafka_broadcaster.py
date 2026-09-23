from confluent_kafka import Producer
import json
import os
from dotenv import load_dotenv
from datetime import datetime

load_dotenv()

# Environment variables for Kafka and the target HTTP endpoint
# if kafka is local, get KAFKA_LOCAL_IP instead of KAFKA_IP
if os.environ.get("KAFKA_LOCAL", "0") == "1":
    kafkaIp = os.environ.get("KAFKA_LOCAL_IP")
else:
    kafkaIp = os.environ.get("KAFKA_IP")

KAFKA_BROKER = kafkaIp
KAFKA_PORT = os.getenv("KAFKA_PORT")
KAFKA_SERVER = f"{KAFKA_BROKER}:{KAFKA_PORT}"
KAFKA_TELEMETRY_TOPIC = os.getenv("KAFKA_TELEMETRY_TOPIC")

if os.environ.get("KAFKA_ACTIVE") == "1":
    producer = Producer({'bootstrap.servers': KAFKA_SERVER})
else:
    print("🔕 Kafka is not active. Telemetry broadcasting is disabled.")
    producer = None

# def delivery_report(err, msg):
#     if err is not None:
#         print(f"Delivery failed: {err}")
#     else:
#         print(f"Telemetry sent to {msg.topic()} [{msg.partition()}]")

def broadcastTelemetry(name, id, telemetry):
    """
    Broadcasts the telemetry dictionary to the UAV_Telemetry Kafka topic.
    """
    if producer is None:
        return
    
    try:

        # {
        # 	"drone_name": "SIM_Bravo",
        # 	"drone_id": 4,
        # 	"timestamp": 1749194171,
        # 	"connection_duration": 572,
        # 	"telemetry": {
        # 		"latitude": 35.15698271920874,
        # 		"longitude": 33.378801893760865,
        # 		"altitude": 30,
        # 		"velocity": 0,
        # 		"heading": 1,
        # 		"gimbalAngle": -80,
        # 		"gpsSignal": 2,
        # 		"satelliteNumber": 28,
        # 		"homeLatitude": 35.15698271920874,
        # 		"homeLongitude": 33.378801893760865,
        # 		"droneState": "Flying",
        # 		"batteryPercentage": 81
        #       "vtolState": "MC"
        # 	}
        # }


        msg = {
            "drone_name": name,
            "drone_id": id,
            "timestamp": datetime.now().timestamp(),
            "telemetry": telemetry
        }
        # print(telemetry, flush=True)
        telemetry_json = json.dumps(msg)
        producer.produce(KAFKA_TELEMETRY_TOPIC, telemetry_json.encode('utf-8'))
        producer.poll(0)
    except Exception as e:
        print(f"Error broadcasting telemetry: {e}")

def flush():
    producer.flush()