#!/usr/bin/env python3
import base64
import json
import os
import random
import uuid
import database.queries
from kafka import KafkaConsumer, KafkaAdminClient
from dotenv import load_dotenv
import time
from datetime import datetime
import database

load_dotenv()

# Environment variables for Kafka and the target HTTP endpoint
# if kafka is local, get KAFKA_LOCAL_IP instead of KAFKA_IP
if os.environ.get("KAFKA_LOCAL", "0") == "1":
    kafkaIp = os.environ.get("KAFKA_LOCAL_IP")
else:
    kafkaIp = os.environ.get("KAFKA_IP")

KAFKA_BROKER = kafkaIp

#KAFKA_GROUP_ID = os.getenv("KAFKA_GROUP_ID")
KAFKA_TELEMETRY_GROUP_ID = os.getenv("KAFKA_TELEMETRY_GROUP_ID")
KAFKA_CRISIS_CLASSIFICATION_GROUP_ID = os.getenv("KAFKA_CRISIS_CLASSIFICATION_GROUP_ID")
KAFKA_OBJECT_DETECTION_GROUP_ID = os.getenv("KAFKA_OBJECT_DETECTION_GROUP_ID")
KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID = os.getenv("KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID")

KAFKA_PORT = os.getenv("KAFKA_PORT")
KAFKA_SERVER = f"{KAFKA_BROKER}:{KAFKA_PORT}"

# Parse topics 
KAFKA_TELEMETRY_TOPIC = os.getenv("KAFKA_TELEMETRY_TOPIC")
KAFKA_PATH_PLANNING_OUTPUT_TOPIC = os.getenv("KAFKA_PATH_PLANNING_OUTPUT_TOPIC")
KAFKA_OBJECT_DETECTION_TOPIC = os.getenv("KAFKA_OBJECT_DETECTION_TOPIC")
KAFKA_CRISIS_CLASSIFICATION_TOPIC = os.getenv("KAFKA_CRISIS_CLASSIFICATION_TOPIC")

print("Kafka Server:", KAFKA_SERVER)
# print("Kafka Group IDS:", KAFKA_TELEMETRY_GROUP_ID, KAFKA_CRISIS_CLASSIFICATION_GROUP_ID,
#        KAFKA_OBJECT_DETECTION_GROUP_ID, KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID)


# def createTelemetryConsumer():
#     return KafkaConsumer(
#         KAFKA_TELEMETRY_TOPIC,  # Topic for telemetry
#         group_id=KAFKA_TELEMETRY_GROUP_ID,
#         bootstrap_servers=KAFKA_SERVER,
#         auto_offset_reset='earliest',  # Start from the earliest message
#         enable_auto_commit=True,  # auto commit
#         consumer_timeout_ms=5000, 
#         value_deserializer=lambda x: json.loads(x.decode('utf-8')),
    # )

MEDIA_ROOT = "/media"

def save_base64_frame(image_base64, drone_name, session_start_time):
    """Decode a base64 image, write it to the shared media directory,
    and return the relative path (as stored by Django's ImageField)."""
    if "," in image_base64:
        image_base64 = image_base64.split(",", 1)[1]

    image_bytes = base64.b64decode(image_base64)

    folder_name = (
        "Detection_Images_"
        + drone_name
        + "_"
        + session_start_time.strftime("%Y-%m-%d_%H.%M.%S")
    )
    relative_dir = folder_name
    abs_dir = os.path.join(MEDIA_ROOT, relative_dir)
    os.makedirs(abs_dir, exist_ok=True)

    filename = uuid.uuid4().hex + ".jpg"
    relative_path = os.path.join(relative_dir, filename)
    abs_path = os.path.join(MEDIA_ROOT, relative_path)

    with open(abs_path, "wb") as f:
        f.write(image_bytes)

    return os.path.join("media", relative_path)


def createObjectDetectionConsumer():
    return KafkaConsumer(
        KAFKA_OBJECT_DETECTION_TOPIC,  # Topic for object detection
        group_id=KAFKA_OBJECT_DETECTION_GROUP_ID,
        bootstrap_servers=KAFKA_SERVER,
        auto_offset_reset='latest',  # Start from the latest message
        enable_auto_commit=True,  #auto commit
        # consumer_timeout_ms=500000, 
        value_deserializer=lambda x: json.loads(x.decode('utf-8')),
    )

def createCrisisClassificationConsumer():
    return KafkaConsumer(
        KAFKA_CRISIS_CLASSIFICATION_TOPIC,  # Topic for crisis classification
        group_id=KAFKA_CRISIS_CLASSIFICATION_GROUP_ID,
        bootstrap_servers=KAFKA_SERVER,
        auto_offset_reset='latest',  # Start from the latest message
        enable_auto_commit=True,   #auto commit
        # consumer_timeout_ms=500000,  
        value_deserializer=lambda x: json.loads(x.decode('utf-8')),
    )

def createPathPlanningOutputConsumer():
    return KafkaConsumer(
        KAFKA_PATH_PLANNING_OUTPUT_TOPIC,  # Topic for path planning input
        group_id=KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID,
        bootstrap_servers=KAFKA_SERVER,
        auto_offset_reset='latest',  # Start from the latest message
        enable_auto_commit=True,  #  #auto commit
        # consumer_timeout_ms=500000, 
        value_deserializer=lambda x: json.loads(x.decode('utf-8')), 
    )

# def consumeTelemetry(telemetryConsumer):
#     print("Starting to consume telemetry messages...")
    
#     for message in telemetryConsumer:
#         try:
#             print(message)
#             # Deserialize the message
#             telemetryData = json.dumps(message.value, indent=4, sort_keys=True) 
            
#             # Pretty-print the JSON with indentation
#             print(telemetryData)

#         except Exception as e:
#             print(f"Error processing message: {e}")
#             print(f"Raw message: {message.value}")

         # database.queries.saveDroneTelemetry(drone_id,connection_duration, telemetry, mission_log_id, operation_id, fov_polygon)
        # database.queries.updateDroneTelemetryLatest(drone_id, connection_duration, telemetry, mission_log_id, operation_id, fov_polygon)





def consumeObjectDetection(objectDetectionConsumer):
    print("Starting to consume object detection messages...")
    user_id = 2
    consumed_count = 0
    drones_with_active_detections = {}  # if a drone is in this dictionary, it means we are actively receiving object detection messages for it and we have an active detection session

    for message in objectDetectionConsumer:
        try:
            consumed_count += 1

            # process all records in this Kafka message
            for record in message.value['records']:
                value  = record["value"]
                header = value["header"]

                drone_id = header["droneID"]
                drone_name = header["drone_name"]

                # print(f"Processing ObjectDetection message for drone {drone_name} (ID: {drone_id})")

                # TODO: ask partners to send a message with drone_id with the same structure to signify the end of the session
                # {
                #     "records": [
                #         {
                #             "value": {
                #                 "header": {
                #                     "topicName": "ObjectDetection",
                #                     "droneID": 3,
                #                     "drone_name": "SIM_Alpha",
                #                     "end_session": true
                #                 }
                #             }
                #         }
                #     ]
                # }


                end_session = header.get("end_session", False)
                if end_session:
                    print(f"Received end session message for drone {drone_name} (ID: {drone_id})")
                    # if end_session is True, we should end the session for this drone
                    if drone_id in drones_with_active_detections:
                        detection_session_id = drones_with_active_detections[drone_id]["detection_session_id"]
                        del drones_with_active_detections[drone_id]  # remove from the tracking dictionary
                    else:
                        detection_session_id = database.queries.getActiveDroneDetectionSession(drone_id)
                        detection_session_id = detection_session_id[0] if detection_session_id else 0  # get the session ID, or 0 if not found
                    
                    if detection_session_id > 0:
                        database.queries.updateSessionEnd(detection_session_id)
                        print(f"Ended detection session for drone {drone_name}")                        
                    continue  # skip further processing for this record

                # check if drone_id is already in drones_with_active_detections, otherwise create a new detection session
                if not drone_id in drones_with_active_detections:
                    operation_id = database.queries.getDroneOperationIdByDroneName(drone_name)  # get drone operation ID from the database
                    session_start_time = datetime.now()
                    detection_session_id = database.queries.deactivateDetectionSessionsAndCreateNew(drone_id, operation_id, user_id) # prepare a new detection session                    
                    drones_with_active_detections[drone_id] = {
                        "detection_session_id": detection_session_id,
                        "operation_id": operation_id,
                        "session_start_time": session_start_time,
                        "drone_name": drone_name,
                    }
                    print(f"\n *** Created detection session for drone {drone_name} *** \n")
                else:
                    detection_session_id = drones_with_active_detections[drone_id]["detection_session_id"]
                    operation_id = drones_with_active_detections[drone_id]["operation_id"]
                    session_start_time = drones_with_active_detections[drone_id]["session_start_time"]

                
                # print(f"Drone ID: {drone_id}, Drone Name: {drone_name}, Detection Session ID: {detection_session_id}", flush=True)

                msg_identifier = header["msgIdentifier"]
                # print(f"OD MSG ID: {msg_identifier}")
                uav_status = header["uav_status"]
                district = header["district"]

                # parse timestamp
                iso_str = header["sentUTC"].replace("Z", "+00:00")
                sentUTC = datetime.fromisoformat(iso_str)

                body = header["body"]
                detection_list = body.get("detection_list", [])

                # save frame-level info
                database.queries.saveObjectDetectionInfo(
                    msg_identifier,
                    uav_status,
                    drone_id,
                    sentUTC,
                    district,
                    detection_session_id
                )

                # save each detected frame and objects
                for detection_frame in detection_list:
                    image_base64 = detection_frame.get("imageData", "")
                    # geo_loc = detection_frame.get("GeoLocation", {})
                    # latitude = float(geo_loc.get("latitude", 0.0))
                    # longitude = float(geo_loc.get("longitude", 0.0))
                    # altitude = float(geo_loc.get("altitude", 0.0))

                    frame_path = save_base64_frame(image_base64, drone_name, session_start_time)
                    frame_id = database.queries.saveDetectedFrame(
                        detection_session_id, frame_path
                    )

                    # print(frame_path)

                    # print("\n-------------------------------------------------")
                    detections = detection_frame.get("detections", [])
                    for obj in detections:
                        bbox_json = json.dumps(obj.get("bbox", []))
                        track_id = obj.get("objectID", None)
                        obj_geolocation = obj.get("obj_geolocation", [0.0, 0.0])
                        obj_latitude = float(obj_geolocation[0])
                        obj_longitude = float(obj_geolocation[1])  
                        obj_class = obj.get("class", "unknown")
                        if obj_class == "human":
                            obj_class = "person"
                       
                        # print(f"Detected Object: {obj_class}, Track ID: {track_id}")
                        database.queries.saveDetectedObject(
                            obj_latitude,
                            obj_longitude,
                            obj_class,
                            detection_session_id,
                            drone_id,
                            frame_id,
                            operation_id,
                            # altitude,
                            bbox_json,
                            obj.get("confidence"),
                            track_id
                        )



                        # print(f"Object Detected: {obj_class}, Track ID: {track_id}, GeoLocation: {obj_geolocation}")
                    # print("-------------------------------------------------")

            # commit only after full processing
            objectDetectionConsumer.commit()

            # print("OD Data Saved Successfully!")
            # print(f"RECEIVED OD COUNT: {consumed_count}, MSG ID: {msg_identifier}")

        except Exception as e:
            print(f"Error processing ObjectDetection message #{consumed_count}: {e}")

    print("Ending consume object detection messages...")


    

def consumeCrisisClassification(crisisClassificationConsumer):
    print("Starting to consume crisis classification messages...")
    consumed_count = 0

    for message in crisisClassificationConsumer:
        try:
            consumed_count += 1

            # Extract the incidentID from the message payload
            incident_id = message.value['body']['incidentID']

            # Retrieve the incident description from the database for the last event with the same incident ID
            incident_description = database.queries.getDescriptionOfCrisisClassificationWithIncidentId(incident_id)
            if incident_description:
                incident_description = incident_description[0]
            else:
                incident_description = "-"

            # save to database
            database.queries.saveCrisisClassificationData(message.value, incident_description)

            # Commit after processing
            crisisClassificationConsumer.commit()

            # Print the count and the incident ID
            print(f"RECEIVED CC COUNT: {consumed_count}, INCIDENT ID: {incident_id}")


        except Exception as e:
            print(f"Error processing CrisisClassification message #{consumed_count}: {e}")



       


def consumePathPlanningOutput(pathPlanningOutputConsumer):
    
    '''
    Consume path planning output messages from Kafka and process them.
        1. save the raw message to the database
        2. extract missions from the message and save each mission to algorithms table
        3. post each mission to Django URL /api/operations/{operation_name}/drones/{drone_name}/mission
    '''
    
    print("Starting to consume path planning output messages")
    
    for message in pathPlanningOutputConsumer:
        all_missions_valid = False
        mission_count = 0
        try:
            print("\n++++++++++++++++++++")
            # print(f"Path Planning Message: {message.value}")
            raw_path_id = database.queries.savePathPlanningDataRaw(message.value) # save the raw message

            for mission in message.value:
                print("**********")

                print(mission["name"])
                action = mission["action"]
                speed = mission["missionSpeed"]
                mission_path  = mission["missionPath"]
                drone_name = mission["name"]
                operation_id = database.queries.getDroneOperationIdByDroneName(drone_name)
                operation_name = database.queries.getOperationNameById(operation_id)
                # TODO: get user ID based on the token sent with the message (?)
                user_id = 2  # hardcoded for now, should be replaced with actual user ID from the message
                database.queries.savePathPlanningPathAsAlgorithm(mission_path, operation_id, drone_name, user_id)
                mission_count += 1

            # update path planning raw data entry with processed=True
            database.queries.updatePathPlanningRawData(raw_path_id, True)  # mark as processed
            all_missions_valid = True
        except Exception as e:
            # TODO: delete any paths that were added to algorithms table
            print(f"Error processing PathPlanning message: {e}")
        finally:
            print("-------------------")

        if all_missions_valid:
            print(f"All missions ({mission_count}) are valid. Sending them to the UAVs...")

            import requests
            for mission in message.value:
                print("**********")

                print(mission["name"])
                action = mission["action"]
                speed = mission["missionSpeed"]
                mission_path  = mission["missionPath"]
                drone_name = mission["name"]
                operation_id = database.queries.getDroneOperationIdByDroneName(drone_name)
                operation_name = database.queries.getOperationNameById(operation_id)
                mission_type = "SEARCH_AND_RESCUE_MISSION"

                url = f"http://{os.getenv('NET_IP')}:{os.getenv('NGINX_PORT')}/api/operations/{operation_name}/drones/{drone_name}/mission"
                # TODO: include bearer token in the request headers (received from kafka message?) or user_id (?)
                payload = {
                    "mission_type": mission_type,
                    "action": action,
                    "grid": False,
                    "captureAndStoreImages": "false",
                    "mission_points": mission_path,
                    "mission_speeds": speed,
                    "mission_gimbal": "N",
                    "mission_repeat": 1,
                }

                try:
                    response = requests.post(url, json=payload)
                    print(f"POST {url} status: {response.status_code}")
                except Exception as e:
                    print(f" ^^^^^^^^^^^^^^^^^^^^^^^ Failed to POST mission to UAV: {e}")
                # ------------------------------------------------





def listConsumersFromKafka():
    # Create an admin client
    adminClient = KafkaAdminClient(
        bootstrap_servers = KAFKA_SERVER
    )
    
    # List topics
    topics = adminClient.list_topics()
    # List consumer groups
    consumerGroups = adminClient.list_consumer_groups()

    for topic in topics:
        print(f"Kafka Topic: {topic}")
    
    for group in consumerGroups:
        groupId = group[0]  # Consumer group ID
        print(f"Consumer Group ID: {groupId}")
        
    
    
    adminClient.close()