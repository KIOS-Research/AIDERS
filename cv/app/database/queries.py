import os
from datetime import datetime
import pytz

# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))


def getActiveDroneDetectionSession(_droneId):
    query = f"SELECT id FROM aiders_detectionsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
    params = (_droneId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def deactivateDetectionSessionsAndCreateNew(_droneId, _operationId, _userId):
    updateQuery = f"UPDATE aiders_detectionsession SET is_active = 0 WHERE drone_id = %s"
    updateParams = (_droneId, )

    insertQuery = (
        "INSERT INTO aiders_detectionsession "
        "(operation_id, user_id, drone_id, start_time, is_active, latest_frame_url) "
        "VALUES (%s, %s, %s, %s, %s, %s)"
    )
    insertParams = (
        _operationId, _userId, _droneId, datetime.now(timezone), 1, "/static/aiders/imgs/drone_img.jpg"
    )

    connector = MySQLConnector()
    connector.executeQuery(updateQuery, updateParams, False)
    sessionId = connector.executeQuery(insertQuery, insertParams, False)
    connector.close()
    return sessionId


def updateDroneDetectionEntry(_droneId, _status, _type, _model):
    query = (
        "UPDATE aiders_detection "
        "SET detection_status = %s, detection_type_str = %s, detection_model = %s WHERE drone_id = %s"
    )
    params = (_status, _type, _model, _droneId)
    connector = MySQLConnector()
    detectionId = connector.executeQuery(query, params, False)
    connector.close()
    return detectionId


def saveFrame(_sessionId, _framePath):
    frameQuery = (
        "INSERT INTO aiders_detectionframe "
        "(detection_session_id, frame, time) "
        "VALUES (%s, %s, %s)"
    )
    frameParams = (_sessionId, _framePath, datetime.now(timezone))

    sessionQuery = "UPDATE aiders_detectionsession SET latest_frame_url = %s WHERE id = %s"
    sessionParams = (_framePath, _sessionId)

    connector = MySQLConnector()
    frameId = connector.executeQuery(frameQuery, frameParams, False)
    connector.executeQuery(sessionQuery, sessionParams, False)
    connector.close()
    return frameId


def getLatestLiveStreamFrame(_droneId):
    query = (
        "SELECT frame FROM aiders_rawframe WHERE drone_id = %s ORDER BY id DESC LIMIT 1"
    )
    params = (_droneId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def updateSessionEnd(_sessionId):
    query = "UPDATE aiders_detectionsession SET is_active = 0, end_time = %s WHERE id = %s"
    params = (datetime.now(timezone), _sessionId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def getDroneLatestTelemetry(_droneId):
    query = (
        "SELECT lat, lon, alt, heading, gimbal_angle FROM aiders_telemetry WHERE drone_id = %s ORDER BY id DESC LIMIT 1"
    )
    params = (_droneId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def saveDetectedObject(_lat, _lon, _label, _trackId, _distance, _sessionId, _frameId):
    query = (
        "INSERT INTO aiders_detectedobject "
        "(lat, lon, label, track_id, distance_from_drone, detection_session_id, frame_id, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (_lat, _lon, _label, _trackId, _distance, _sessionId, _frameId, datetime.now(timezone))

    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def saveDetectedDisaster(_lat, _lon, _earthquake, _fire, _flood, _sessionId, _frameId):
    query = (
        "INSERT INTO aiders_detecteddisaster "
        "(lat, lon, earthquake_probability, fire_probability, flood_probability, detection_session_id, frame_id, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (_lat, _lon, _earthquake, _fire, _flood, _sessionId, _frameId, datetime.now(timezone))

    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()
