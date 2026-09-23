import json
import os
from datetime import datetime
import pytz

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

# custom libs
from database.connection import MySQLConnector


def getDroneOperationIdByDroneName(_droneName):
    query = f"SELECT operation_id FROM aiders_drone WHERE drone_name = %s LIMIT 1"
    params = (_droneName, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result[0]

def getOperationNameById(_operationId):
    query = f"SELECT operation_name FROM aiders_operation WHERE id = %s LIMIT 1"
    params = (_operationId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result[0]


###########################
# OBJECT DETECTION
###########################


def saveObjectDetectionInfo( _msgIdentifier, _uavStatus, _droneID, _sentUTC, _district, _sessionID):

    query = (
        "INSERT INTO aiders.aiders_detectioninfo "
        "(msg_identifier, uav_status, drone_id, sent_utc, district, detection_session_id) "
        "VALUES (%s, %s, %s, %s, %s, %s)"
    )

    params = (_msgIdentifier, _uavStatus, _droneID, _sentUTC,_district, _sessionID)


    connector = MySQLConnector()
    connector.executeQuery(query, params)
    connector.close()



def saveDetectedObject(
    _lat, _lon, _label,
    _sessionId, _droneId, _frameId,
    _operationId, _bounding_boxes, 
    _confidence, _track_id
):
    query = (
        "INSERT INTO aiders_detectedobject "
        "(time, lat, lon, label, detection_session_id, drone_id, frame_id, "
        "operation_id, bounding_boxes, confidence, track_id) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )

    params = (
        datetime.now(timezone),
        _lat,
        _lon,
        _label,
        _sessionId,
        _droneId,
        _frameId,
        _operationId,
        json.dumps(_bounding_boxes),
        _confidence,
        _track_id,
    )

    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDetectedFrame(_sessionId, _framePath):
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


def deactivateDetectionSessionsAndCreateNew(_droneId, _operationId, _userId):
    updateQuery = f"UPDATE aiders_detectionsession SET is_active = 0, end_time = %s WHERE drone_id = %s AND is_active = 1"
    updateParams = (datetime.now(timezone), _droneId)

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


def getActiveDroneDetectionSession(_droneId):
    query = f"SELECT id FROM aiders_detectionsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
    params = (_droneId, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, False)
    connector.close()
    return result

def updateSessionEnd(_sessionId):
    query = "UPDATE aiders_detectionsession SET is_active = 0, end_time = %s WHERE id = %s"
    params = (datetime.now(timezone), _sessionId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def getDetectionSessionStatus(session_id):
    query  = "SELECT is_active FROM aiders_detectionsession WHERE id = %s"
    params    = (session_id,)
    connector = MySQLConnector()
    result      = connector.executeQuery(query, params, False)
    connector.close()
    return result[0][0]



# def getActiveDroneLiveSession(_droneId):
#     query = f"SELECT id FROM aiders_livestreamsession WHERE drone_id = %s AND is_active = 1 LIMIT 1"
#     params = (_droneId, )
#     connector = MySQLConnector()
#     result = connector.executeQuery(query, params, True)
#     connector.close()
#     return result


# def updateLiveStreamConnectionStatus(_droneId):
#     query = "UPDATE aiders_drone SET is_live_stream_connected = 1 WHERE id = %s"
#     params = (_droneId, )
#     connector = MySQLConnector()
#     connector.executeQuery(query, params, False)
#     connector.close()

# def deactivateSessionsAndCreateNew(_droneId):
#     updateQuery = f"UPDATE aiders_livestreamsession SET is_active = 0 WHERE drone_id = %s"
#     updateParams = (_droneId, )

#     insertQuery = (
#         "INSERT INTO aiders_livestreamsession "
#         "(drone_id, start_time, is_active, latest_frame_url) "
#         "VALUES (%s, %s, %s, %s)"
#     )
#     insertParams = (
#         _droneId, datetime.now(timezone), 1, "/static/aiders/imgs/drone_img.jpg"
#     )

#     connector = MySQLConnector()
#     connector.executeQuery(updateQuery, updateParams, False)
#     sessionId = connector.executeQuery(insertQuery, insertParams, False)
#     connector.close()
#     return sessionId

# def updateSessionEnd(_sessionId):
#     query = "UPDATE aiders_livestreamsession SET is_active = 0, end_time = %s WHERE id = %s"
#     params = (datetime.now(timezone), _sessionId)
#     connector = MySQLConnector()
#     connector.executeQuery(query, params, False)
#     connector.close()

# def saveLiveFrame(_droneId, _sessionId, _framePath):
#     frameQuery = (
#         "INSERT INTO aiders_rawframe "
#         "(drone_id, live_stream_session_id, frame, time) "
#         "VALUES (%s, %s, %s, %s)"
#     )
#     frameParams = (_droneId, _sessionId, _framePath, datetime.now(timezone))

#     telemetryLatestQuery = "UPDATE aiders_telemetrylatest SET live_stream_frame_url = %s WHERE drone_id = %s"
#     telemetryLatestParams = (_framePath, _droneId)

#     connector = MySQLConnector()
#     connector.executeQuery(frameQuery, frameParams, False)
#     connector.executeQuery(telemetryLatestQuery, telemetryLatestParams, False)
#     connector.close()

# def getDroneConnectionState(_droneId):
#     query = f"SELECT is_connected_with_platform FROM aiders_drone WHERE id = %s LIMIT 1"
#     params = (_droneId, )
#     connector = MySQLConnector()
#     result = connector.executeQuery(query, params, True)
#     connector.close()
#     return result


###########################
# CRISIS CLASSIFICATION
###########################


def getDescriptionOfCrisisClassificationWithIncidentId(_incident_id):
    query = "SELECT description FROM aiders_crisisclassification WHERE incident_id = %s ORDER BY id DESC LIMIT 1"
    params = (_incident_id, )
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

crisis_counter = 0
def saveCrisisClassificationData(_entry, _incident_description):
    try:
        header = _entry.get("header")
        body = _entry.get("body")

        # dt = datetime.strptime(body["timestamp"], "%Y-%m-%dT%H:%M:%S.%fZ")
        # formattedTimestamp = dt.strftime("%Y-%m-%d %H:%M:%S")

        if not header or not body:
            raise ValueError("Missing 'header' or 'body' fields in message")
        
        sentUTC = header["sentUTC"]
        sentUTC = datetime.fromisoformat(sentUTC)

        query = (
            "INSERT INTO aiders_crisisclassification "
            "(sent_utc, sender, incident_id, incident_type, severity_level, latitude, longitude, timestamp, area_type, time_of_day, detections_summary, description, resolved, false_alarm) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, 0, 0)"
        )

        params = (
            sentUTC,
            header["sender"],
            body["incidentID"],
            body["incidentType"],
            body["severityLevel"],
            body["geoLocation"]["latitude"],
            body["geoLocation"]["longitude"],
            body["timestamp"],
            body["areaType"],
            body["timeOfDay"],
            json.dumps(body["detectionsSummary"]),
            _incident_description
        )

        connector = MySQLConnector()
        connector.executeQuery(query, params, False)
        global crisis_counter
        crisis_counter += 1

        print(f"Number of saved Crisis Classifications: {crisis_counter}")
    except Exception as e:
        print(f"Error saving crisis classification: {e}")
    finally:
        connector.close()


###########################
# PATH PLANNING OUTPUT
###########################

def savePathPlanningDataRaw(entry):

    try:
        insert_query = (
            "INSERT INTO aiders_pathplanningoutput "
            "(message, time, processed) "
            "VALUES (%s, %s, 0)"
        )
        entry = json.dumps(entry)  # Convert entry to JSON string
        time = datetime.now(timezone)
        insert_params = (entry,time)
        connector = MySQLConnector()
        id = connector.executeQuery(insert_query, insert_params, False)        

        print("Path planning raw message saved successfully!")
        return id
    except Exception as e:
        print(f"Error saving raw path planning message: {e}")
    finally:
        connector.close()


# mark the raw path planning data as processed
def updatePathPlanningRawData(id, processed):
    try:
        update_query = (
            "UPDATE aiders_pathplanningoutput "
            "SET processed = %s "
            "WHERE id = %s"
        )
        connector = MySQLConnector()
        update_params = (processed, id)
        connector.executeQuery(update_query, update_params, False)
        print("Path planning raw message updated successfully!")
    except Exception as e:
        print(f"Error updating raw path planning message: {e}")
    finally:
        connector.close()


def savePathPlanningPathAsAlgorithm(mission_path, operation_id, drone_name, user_id):
    try:
        time = datetime.now(timezone)
        algorithm_name = "CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM"
        input = json.dumps({'source': 'PathPlanning_Output'})
        # print(mission_path)
        # print(json.dumps(mission_path))
        output = json.dumps([{"path":mission_path, "drone_id": drone_name}])
        canBeLoadedOnMap = 1
        title = f"Mission path for {drone_name}"
        operation_id = operation_id

        insert_query = (
            "INSERT INTO aiders_algorithm "
            "(time, algorithm_name, input, output, canBeLoadedOnMap, title, operation_id, user_id) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
        )

        connector = MySQLConnector()
        insert_params = (time, algorithm_name, input, output, canBeLoadedOnMap, title, operation_id, user_id,)
        connector.executeQuery(insert_query, insert_params, False)
        print("Path planning data saved in algorithms!")
    except Exception as e:
        print(f"Error saving path planning data: {e}")
    finally:
        connector.close()
