import os
from datetime import datetime
import pytz

# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc  # timezone = pytz.timezone(os.environ.get("TZ"))


def getActiveDroneDetectionSession(_droneId):
    query = f"SELECT id FROM aiders_safemldetectionsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def deactivateDetectionSessionsAndCreateNew(_droneId, _operationId, _userId):
    updateQuery = f"UPDATE aiders_safemldetectionsession SET is_active = 0 WHERE drone_id = %s"
    updateParams = (_droneId,)

    insertQuery = (
        "INSERT INTO aiders_safemldetectionsession "
        "(operation_id, user_id, drone_id, start_time, is_active, latest_frame_url) "
        "VALUES (%s, %s, %s, %s, %s, %s)"
    )
    insertParams = (_operationId, _userId, _droneId, datetime.now(timezone), 1, "/static/aiders/imgs/drone_img.jpg")

    connector = MySQLConnector()
    connector.executeQuery(updateQuery, updateParams, False)
    sessionId = connector.executeQuery(insertQuery, insertParams, False)
    connector.close()
    return sessionId

def updateDroneDetectionEntry(_droneId, _status, _type, _model):
    query = (
        "UPDATE aiders_safemldetection "
        "SET detection_status = %s, detection_type_str = %s, detection_model = %s WHERE drone_id = %s"
    )
    params = (_status, _type, _model, _droneId)
    connector = MySQLConnector()
    detectionId = connector.executeQuery(query, params, False)
    connector.close()
    return detectionId


def saveFrame(_sessionId, _framePath):
    frameQuery = (
        "INSERT INTO aiders_safemldetectionframe "
        "(detection_session_id, frame, time) "
        "VALUES (%s, %s, %s)"
    )
    frameParams = (_sessionId, _framePath, datetime.now(timezone))

    sessionQuery = "UPDATE aiders_safemldetectionsession SET latest_frame_url = %s WHERE id = %s"
    sessionParams = (_framePath, _sessionId)

    connector = MySQLConnector()
    frameId = connector.executeQuery(frameQuery, frameParams, False)
    connector.executeQuery(sessionQuery, sessionParams, False)
    connector.close()
    return frameId


def getLatestLiveStreamFrame(_droneId):
    query = "SELECT frame FROM aiders_rawframe WHERE drone_id = %s ORDER BY id DESC LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def updateSessionEnd(_sessionId):
    query = "UPDATE aiders_safemldetectionsession SET is_active = 0, end_time = %s WHERE id = %s"
    params = (datetime.now(timezone), _sessionId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def saveSafeMLOutput(_sessionId, _frameId, _scueData):
    query = "INSERT INTO aiders_safemloutput (time, detection_session_id, scue, frame_id) VALUES (%s, %s, %s, %s)"
    params = (datetime.now(timezone), _sessionId, _scueData, _frameId)

    connector = MySQLConnector()
    safemlOutputId = connector.executeQuery(query, params, False)
    connector.close()
    return safemlOutputId

def getActiveDroneDetectionSessionDeepKnowledge(_droneId):
    query = f"SELECT id FROM aiders_deepknowledgedetectionsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def deactivateDetectionSessionsAndCreateNewDeepKnowledge(_droneId, _operationId, _userId):
    updateQuery = f"UPDATE aiders_deepknowledgedetectionsession SET is_active = 0 WHERE drone_id = %s"
    updateParams = (_droneId,)

    insertQuery = (
        "INSERT INTO aiders_deepknowledgedetectionsession "
        "(operation_id, user_id, drone_id, start_time, is_active, latest_frame_url) "
        "VALUES (%s, %s, %s, %s, %s, %s)"
    )
    insertParams = (_operationId, _userId, _droneId, datetime.now(timezone), 1, "/static/aiders/imgs/drone_img.jpg")

    connector = MySQLConnector()
    connector.executeQuery(updateQuery, updateParams, False)
    sessionId = connector.executeQuery(insertQuery, insertParams, False)
    connector.close()
    return sessionId

def updateDroneDetectionEntryDeepKnowledge(_droneId, _status, _type, _model):
    query = (
        "UPDATE aiders_deepknowledgedetection "
        "SET detection_status = %s, detection_type_str = %s, detection_model = %s WHERE drone_id = %s"
    )
    params = (_status, _type, _model, _droneId)
    connector = MySQLConnector()
    detectionId = connector.executeQuery(query, params, False)
    connector.close()
    return detectionId


def saveFrameDeepKnowledge(_sessionId, _framePath):
    frameQuery = (
        "INSERT INTO aiders_deepknowledgedetectionframe "
        "(detection_session_id, frame, time) "
        "VALUES (%s, %s, %s)"
    )
    frameParams = (_sessionId, _framePath, datetime.now(timezone))

    sessionQuery = "UPDATE aiders_deepknowledgedetectionsession SET latest_frame_url = %s WHERE id = %s"
    sessionParams = (_framePath, _sessionId)

    connector = MySQLConnector()
    frameId = connector.executeQuery(frameQuery, frameParams, False)
    connector.executeQuery(sessionQuery, sessionParams, False)
    connector.close()
    return frameId


def getLatestLiveStreamFrameDeepKnowledge(_droneId):
    query = "SELECT frame FROM aiders_rawframe WHERE drone_id = %s ORDER BY id DESC LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def updateSessionEndDeepKnowledge(_sessionId):
    query = "UPDATE aiders_deepknowledgedetectionsession SET is_active = 0, end_time = %s WHERE id = %s"
    params = (datetime.now(timezone), _sessionId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDeepKnowledgeOutput(_sessionId, _frameId, _Data):
    query = "INSERT INTO aiders_deepknowledgeoutput (time, detection_session_id, uncertainty, frame_id) VALUES (%s, %s, %s, %s)"
    params = (datetime.now(timezone), _sessionId, _Data, _frameId)

    connector = MySQLConnector()
    deepknowledgeOutputId = connector.executeQuery(query, params, False)
    connector.close()
    return deepknowledgeOutputId