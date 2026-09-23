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
import Disaster_Classification.Predict_Images
import crowd_loc.crowdloc_run
import waldo.waldo_run

runningDisasterClassifications = {}
runningCrowdDetectors = {}
runningWaldoDetectors = {}




###########################
# DISASTER CLASSIFICATION
###########################

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


###########################
# CROWD LOCALIZATION
###########################

# start the crowd detector
def startDroneCrowdDetector(_data):
    threadName = f"crowdLocThread_{_data['droneName']}"
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


###########################
# WALDO
###########################

# start the waldo detector
def startDroneWaldoDetector(_data):
    threadName = f"waldoThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNew(_data['droneId'], _data['operationId'], _data['userId'])  # deactivate drone's sessions and create a new one

    # start a thread that runs CV crowd detector on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=waldoThread, args=(_data, sessionId))
        thread.name = threadName
        thread.start()


# instantiate and run a new ObjectDetector object
def waldoThread(_data, _sessionId):
    objectDetector = waldo.waldo_run.ObjectDetector(
        droneId = _data['droneId'],
        operationId = _data['operationId'],
        userId = _data['userId'],
        sessionId = _sessionId,
        droneName = _data['droneName']
    )
    database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_CONNECTED", "WALDO_DETECTOR", "YOLO")
    runningWaldoDetectors[_data['droneName']] = objectDetector
    objectDetector.start_loop()




###########################
# STOP
###########################


# stop the detection of a specific drone
def stopDetection(_data):
    try:
        waldoDetector = runningWaldoDetectors[_data['droneName']]
        waldoDetector.stopDetector()
        del runningWaldoDetectors[_data['droneName']]
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