import os
import sys
import time
import threading
import cv2
from datetime import datetime

# custom libs
from database.connection import MySQLConnector
import database.queries
import utils
import object_detection.aiders_tracker
import Disaster_Classification.Predict_Images
import crowd_loc.crowdloc_run

runningTrackers = {}
runningDisasterClassifications = {}
runningCrowdDetectors = {}



# start the tracker on a new thread
def startDroneStreamTracker(_data):
    threadName = f"trackerThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNew(_data['droneId'], _data['operationId'], _data['userId'])  # deactivate drone's sessions and create a new one

    # start a thread that runs CV tracker on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=trackerThread, args=(_data, sessionId))
        thread.name = threadName
        thread.start()


# instantiate and run a new aiders_tracker object
def trackerThread(_data, _sessionId):
    rtmp_url = f"rtmp://{os.environ['NET_IP']}/live/{_data['droneName']}"
    tracker = object_detection.aiders_tracker.Tracker(
        rtmp_url,
        drone = _data['droneId'],
        operation = _data['operationId'],
        detection_type_str = _data['detectionType'],
        user = _data['userId'],
        session = _sessionId,
        drone_name = _data['droneName']
    )
    database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_CONNECTED", _data["detectionType"], tracker.get_detector_model(_data["detectionType"]))
    runningTrackers[_data['droneName']] = tracker
    tracker.start_tracker()     




# start the disaster classification
def startDroneDisasterClassification(_data):
    threadName = f"disasterThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNew(_data['droneId'], _data['operationId'], _data['userId'])  # deactivate drone's sessions and create a new one

    # start a thread that runs CV disaster classification on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=disasterClassificationThread, args=(_data, sessionId))
        thread.name = threadName
        thread.start()


# instantiate and run a new DisasterClassification object
def disasterClassificationThread(_data, _sessionId):
    # rtmp_url = f"rtmp://{os.environ['NET_IP']}/live/{_data['droneName']}"
    disasterClassification = Disaster_Classification.Predict_Images.DisasterClassification(
        droneId = _data['droneId'],
        operationId = _data['operationId'],
        userId = _data['userId'],
        sessionId = _sessionId,
        droneName = _data['droneName']
    )
    database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_CONNECTED", "DISASTER_CLASSIFICATION", "YOLO")
    runningDisasterClassifications[_data['droneName']] = disasterClassification
    disasterClassification.start()





# start the crowd detector
def startDroneCrowdDetector(_data):
    threadName = f"disasterThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNew(_data['droneId'], _data['operationId'], _data['userId'])  # deactivate drone's sessions and create a new one

    # start a thread that runs CV crowd detector on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=crowdDetectionThread, args=(_data, sessionId))
        thread.name = threadName
        thread.start()


# instantiate and run a new CrowdDetector object
def crowdDetectionThread(_data, _sessionId):
    # rtmp_url = f"rtmp://{os.environ['NET_IP']}/live/{_data['droneName']}"
    crowdDetector = crowd_loc.crowdloc_run.Crowd_local(
        droneId = _data['droneId'],
        operationId = _data['operationId'],
        userId = _data['userId'],
        sessionId = _sessionId,
        droneName = _data['droneName']
    )
    database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_CONNECTED", "CROWD_LOCALIZATION", "YOLO")
    runningCrowdDetectors[_data['droneName']] = crowdDetector
    crowdDetector.start_loop()






# stop the detection of a specific drone
def stopDetection(_data):
    try:
        tracker = runningTrackers[_data['droneName']]
        tracker.stop_tracker()
        del runningTrackers[_data['droneName']]
    except:
        # check if disaster classification detection is running and stop it
        try:
            disasterClassification = runningDisasterClassifications[_data['droneName']]
            disasterClassification.stopDetector()
            del runningDisasterClassifications[_data['droneName']]
        except:
            # check if disaster classification detection is running and stop it
            try:
                crowdDetector = runningCrowdDetectors[_data['droneName']]
                crowdDetector.stopDetector()
                del runningCrowdDetectors[_data['droneName']]
            except:
                pass
            
    database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_DISCONNECTED", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])
    database.queries.updateSessionEnd(session[0])