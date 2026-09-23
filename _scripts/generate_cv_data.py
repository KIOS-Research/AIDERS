import threading
import time
import json
import random
from datetime import datetime, timezone
from kafka import KafkaProducer

class ObjectDetection:
    # Constants
    TOPIC_NAME     = "ObjectDetection"
    UAV_STATUSES   = ["Flying", "Landing", "Hovering"]
    DISTRICTS      = ["Athens", "Thessaloniki", "Patras", "Heraklion"]
    OBJECT_CLASSES = ["car", "person", "truck", "bicycle"]
    OBJECT_CLASSES = ["car", "human", "skiff", "cruise", "sailboat"]

    # Message ID counter
    sequential_id = 0
    seq_lock      = threading.Lock()

    # Frame‐ID cycling params
    START_TIME    = datetime.strptime("06:55:53", "%H:%M:%S").time()
    END_TIME      = datetime.strptime("08:13:21", "%H:%M:%S").time()
    _START_SEC    = START_TIME.hour*3600 + START_TIME.minute*60 + START_TIME.second
    _END_SEC      = END_TIME.hour*3600   + END_TIME.minute*60   + END_TIME.second
    FRAME_ID_MAX  = _END_SEC - _START_SEC
    frame_counter = 0
    frame_lock    = threading.Lock()

    def __init__(self):
        self.bootstrap_server = "apps.edutel.uniwa.gr:9092"
        # self.bootstrap_server = "192.168.0.10:9092"
        self.kafka_topic     = self.TOPIC_NAME
        self.interval        = 2  # seconds between messages

        # initialize the send counter
        self.sent_count = 0

        self.producer = KafkaProducer(
            bootstrap_servers=self.bootstrap_server,
            value_serializer=lambda v: json.dumps(v).encode("utf-8")
        )

    def start(self):
        print(f"Sending data to Kafka topic '{self.kafka_topic}' every {self.interval} seconds...")
        try:
            while True:
                self.publish_detection()
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("\nStopped by user.")

    def generate_detection(self):
        # ISO timestamp for the message envelope
        now       = datetime.now(timezone.utc)
        timestamp = now.strftime("%Y-%m-%dT%H:%M:%S.%fZ")

        # bump the message ID
        with ObjectDetection.seq_lock:
            ObjectDetection.sequential_id += 1
            msg_id = ObjectDetection.sequential_id

        self.drone_id = 1
        self.drone_name = "SIM_Alpha"
        # frame geo coords
        # frame_lat = round(random.uniform(35.0, 36.0), 8)
        # frame_lon = round(random.uniform(33.0, 34.0), 8)
        # frame_alt = round(random.uniform(10.0, 100.0), 4)
        # frame_lat = 35.156983
        # frame_lon = 33.378302
        frame_lat = 37.965739698074216
        frame_lon = 23.779161151930424
        frame_alt = 20

        status   = random.choice(self.UAV_STATUSES)
        district = random.choice(self.DISTRICTS)

        detection_list = []
        num_frames = 1

        for _ in range(num_frames):
            # build object detections
            dets = []
            for obj in range(1, random.randint(1, 5) + 1):
                x1 = round(random.uniform(0, 400), 4)
                y1 = round(random.uniform(0, 300), 4)
                w  = round(random.uniform(20, 200), 4)
                h  = round(random.uniform(20, 200), 4)
                class_label = random.choice(self.OBJECT_CLASSES)

                # random object geolocation
                obj_lat = frame_lat + random.uniform(-0.000055, 0.000055)
                obj_lon = frame_lon + random.uniform(-0.00011, 0.00011)

                dets.append({
                    "objectID":   random.randint(1, 100),
                    "class":      class_label,
                    "confidence": round(random.uniform(0.5, 1.0), 10),
                    "bbox":       [x1, y1, round(x1 + w, 4), round(y1 + h, 4)],
                    "obj_geolocation": [obj_lat, obj_lon]
                })
                # print("Generated detection:", dets[-1])



            # cycle the global frame counter
            with ObjectDetection.frame_lock:
                ObjectDetection.frame_counter += 1
                if ObjectDetection.frame_counter > ObjectDetection.FRAME_ID_MAX:
                    ObjectDetection.frame_counter = 0
                global_frame_id = ObjectDetection.frame_counter

            image_url = f"http://46.12.206.90:8000/images/frame_{global_frame_id:04d}.jpg"

            detection_list.append({
                "frameID":    global_frame_id,
                "imageURL":   image_url,
                "detections": dets,
                "GeoLocation": {
                    "latitude":   frame_lat,
                    "longitude":  frame_lon,
                    "altitude":   frame_alt
                }
            })

        header = {
            "topicName":     self.TOPIC_NAME,
            "msgIdentifier": str(msg_id),
            "uav_status":    status,
            "droneID":       self.drone_id,
            "drone_name":    self.drone_name,
            "sentUTC":       timestamp,
            "district":      district,
            "body": {
                "detection_list": detection_list
            }
        }

        return {"header": header}

    def publish_detection(self):
        data = self.generate_detection()

        # increment and print only the counter and msgIdentifier
        self.sent_count += 1
        msg_id = data["header"]["msgIdentifier"]
        payload = {"records": [{"value": data}]}

        # print(payload)

        self.producer.send(self.kafka_topic, payload)
        self.producer.flush()

        print(f"Sent payload #{self.sent_count} with MSG ID: {msg_id}")

        # end the session after every 10 messages
        if self.sent_count % 10 == 0:
            print(f"Sent {self.sent_count} messages, ending session.")
            payload ={
                    "records": [
                        {
                            "value": {
                                "header": {
                                    "topicName": "ObjectDetection",
                                    "droneID": self.drone_id,
                                    "drone_name": self.drone_name,
                                    "end_session": True
                                }
                            }
                        }
                    ]
                }
            self.producer.send(self.kafka_topic, payload)
            self.producer.flush()

        # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # print(f"[{current_timestamp}] Sent payload #{self.sent_count} with MSG ID: {msg_id}")

if __name__ == "__main__":
    detector = ObjectDetection()
    detector.start()
