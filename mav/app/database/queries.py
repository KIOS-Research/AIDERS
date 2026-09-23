import json
import os
from datetime import datetime

import pytz

# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))


def getDroneByNameAndModel(_name, _model):
    query = f"SELECT id, is_connected_with_platform, live_stream_url FROM aiders_drone WHERE drone_name = %s AND model = %s LIMIT 1"
    params = (_name, _model)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result


def updateDroneConnectionStatus(_id, _status):
    if _status == 0:
        query = f"UPDATE aiders_drone SET is_connected_with_platform = {_status}, build_map_activated = 0 WHERE id = %s"
    else:
        query = f"UPDATE aiders_drone SET is_connected_with_platform = {_status} WHERE id = %s"
    params = (_id,)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def saveDrone(_drone):
    query = (
        "INSERT INTO aiders_drone "
        "(drone_name, ip, model, camera_model, time, is_connected_with_platform, "
        "ballistic_available, water_sampler_available, weather_station_available, "
        "multispectral_available, lidar_available, drone_movement_available, build_map_activated, operation_id, type) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _drone["name"], _drone["ip"], _drone["model"], _drone["cameraModel"], datetime.now(timezone), 1,
        _drone["ballisticAvailable"], _drone["waterSamplerAvailable"], _drone["weatherStationAvailable"],
        _drone["multispectralAvailable"], _drone["lidarAvailable"], 0, 0, _drone["operation_id"], _drone["type"]
    )
    connector = MySQLConnector()
    droneId = connector.executeQuery(query, params, False)
    connector.close()
    return droneId


def createDroneDetectionEntry(_droneId):
    query = (
        "INSERT INTO aiders_detection "
        "(drone_id, detection_status, detection_type_str, detection_model) "
        "VALUES (%s, %s, %s, %s)"
    )
    params = (_droneId, "DETECTION_INITIAL_STATUS", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
    connector = MySQLConnector()
    detectionId = connector.executeQuery(query, params, False)
    connector.close()
    return detectionId


def saveDroneTelemetry(_droneId, _secondsOn, _telemetryData, _missionLogId, _operationId, _fov_polygon):
    query = (
        "INSERT INTO aiders_telemetry "
        "(drone_id, lat, lon, alt, heading, velocity, "
        "gps_signal, satellites, homeLat, homeLon, drone_state, mission_log_id, "
        "gimbal_angle, water_sampler_in_water, battery_percentage, vtol_state, operation_id, secondsOn, fov_coordinates, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _droneId, _telemetryData["latitude"], _telemetryData["longitude"], _telemetryData["altitude"], _telemetryData["heading"], _telemetryData["velocity"],
        _telemetryData["gpsSignal"], _telemetryData["satelliteNumber"], _telemetryData["homeLatitude"], _telemetryData["homeLongitude"], _telemetryData["droneState"], _missionLogId,
        _telemetryData["gimbalAngle"], False, _telemetryData["batteryPercentage"], str(_telemetryData["vtolState"]), _operationId, _secondsOn, json.dumps(_fov_polygon), datetime.now(timezone)
    )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def createDroneTelemetryLatest(_droneId):
    connector = MySQLConnector()
    
    # Check if a record with the same drone_id already exists
    check_query = "SELECT * FROM aiders_telemetrylatest WHERE drone_id = %s"
    check_params = (_droneId,)
    existing_record = connector.executeQuery(check_query, check_params)
    
    # If no record exists, insert a new one
    if not existing_record:
        insert_query = (
            "INSERT INTO aiders_telemetrylatest "
            "(drone_id, time_updated) "
            "VALUES (%s, %s)"
        )
        insert_params = (_droneId, datetime.now(timezone))
        connector.executeQuery(insert_query, insert_params, False)
    
    connector.close()


def updateDroneTelemetryLatest(_droneId, _secondsOn, _rosMsg, _missionLogId, _operationId, _fov_polygon):
    query = (
        "UPDATE aiders_telemetrylatest "
        "SET lat = %s, lon = %s, alt = %s, heading = %s, velocity = %s, gps_signal = %s, satellites = %s, "
        "homeLat = %s, homeLon = %s, drone_state = %s, mission_log_id = %s, gimbal_angle = %s, "
        "water_sampler_in_water = %s, battery_percentage = %s, vtol_state = %s, operation_id = %s, secondsOn = %s, fov_coordinates = %s, time_updated = %s "
        "WHERE drone_id = %s "
    )
    params = (
        _rosMsg["latitude"], _rosMsg["longitude"], _rosMsg["altitude"], _rosMsg["heading"], _rosMsg["velocity"], _rosMsg["gpsSignal"], _rosMsg["satelliteNumber"], 
        _rosMsg["homeLatitude"], _rosMsg["homeLongitude"], _rosMsg["droneState"], _missionLogId, _rosMsg["gimbalAngle"], 
        False, _rosMsg["batteryPercentage"], str(_rosMsg["vtolState"]), _operationId, _secondsOn, json.dumps(_fov_polygon), datetime.now(timezone),
        _droneId,
        )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


def saveMavlinkLog(_droneId, _operationId, _msg):
    query = (
        "INSERT INTO aiders_mavlinklog "
        "(drone_id, operation_id, time, message) "
        "VALUES (%s, %s, %s, %s)"
    )
    params = (_droneId, _operationId, datetime.now(timezone), _msg)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()