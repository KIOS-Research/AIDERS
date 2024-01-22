import threading

# custom libs
import database.queries
import utils
from integration_safeml import SafeMLDetection
from integration_deepknowledge import DeepKnowledgeDetection

runningSafeML = {}
runningDeepKnowledge = {}

# start the safeml detection on a new thread
def startDroneSafeMLDetection(_data):
    threadName = f"safemlThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSession(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNew(_data['droneId'], _data['operationId'], _data['userId'])
    # start a thread that runs CV tracker on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=saveMlThread, args=(_data["droneId"], _data["operationId"], _data["userId"], sessionId, _data["droneName"]))
        thread.name = threadName
        thread.start()


# stop the detection of a specific drone
def stopDetection(_data ):
    print(_data, flush=True)
    if _data['detectionType'] == "SAFEML":
        deepKnowledgeDetection = runningSafeML[_data['droneName']]
        deepKnowledgeDetection.stop()
        del runningSafeML[_data['droneName']]
        database.queries.updateDroneDetectionEntry(_data['droneId'], "DETECTION_DISCONNECTED", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
        session = database.queries.getActiveDroneDetectionSession(_data['droneId'])
        database.queries.updateSessionEnd(session[0])
        return True
    elif _data['detectionType'] == "DEEPKNOWLEDGE":
        deepKnowledgeDetection = runningDeepKnowledge[_data['droneName']]
        deepKnowledgeDetection.stop()
        del runningDeepKnowledge[_data['droneName']]
        database.queries.updateDroneDetectionEntryDeepKnowledge(_data['droneId'], "DETECTION_DISCONNECTED", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
        session = database.queries.getActiveDroneDetectionSessionDeepKnowledge(_data['droneId'])
        database.queries.updateSessionEndDeepKnowledge(session[0])
        return True
    return False


def saveMlThread(_droneId, _operationId, _userId, _sessionId, _droneName):
    safeMlDetection=SafeMLDetection(_droneId, _operationId, _userId, _sessionId, _droneName)
    database.queries.updateDroneDetectionEntry(_droneId, "DETECTION_CONNECTED", "SAFEML", "YOLO")
    runningSafeML[_droneName] = safeMlDetection
    safeMlDetection.start()

def startDroneDeepKnowledgeDetection(_data):
    threadName = f"deepKnowledgeThread_{_data['droneName']}"
    session = database.queries.getActiveDroneDetectionSessionDeepKnowledge(_data['droneId'])  # retrieve last ACTIVE live session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateDetectionSessionsAndCreateNewDeepKnowledge(_data['droneId'], _data['operationId'], _data['userId'])
    # start a thread that runs CV tracker on the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=deepKnowledgeThread, args=(_data["droneId"], _data["operationId"], _data["userId"], sessionId, _data["droneName"]))
        thread.name = threadName
        thread.start()

def deepKnowledgeThread(_droneId, _operationId, _userId, _sessionId, _droneName):
    deepKnowledge=DeepKnowledgeDetection(_droneId, _operationId, _userId, _sessionId, _droneName)
    database.queries.updateDroneDetectionEntryDeepKnowledge(_droneId, "DETECTION_CONNECTED", "DEEPKNOWLEDGE", "YOLO")
    runningDeepKnowledge[_droneName] = deepKnowledge
    deepKnowledge.start()