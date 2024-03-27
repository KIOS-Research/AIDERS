import os
from datetime import datetime
import pytz
import json

# custom libs
from database.connection import MySQLConnector

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))


#####################################
############## DRONES ###############
#####################################


def getDroneByNameAndModel(_name, _model):
    query = f"SELECT id, is_connected_with_platform FROM aiders_drone WHERE drone_name = %s AND model = %s LIMIT 1"
    params = (_name, _model)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getDroneMissionLogId(_droneId):
    query = f"SELECT id FROM aiders_missionlog WHERE drone_id = %s AND action = 'START_MISSION' ORDER BY id DESC LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getDroneOperationId(_droneId):
    query = f"SELECT operation_id FROM aiders.aiders_drone WHERE id = %s"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getDroneState(_droneId):
    query = f"SELECT drone_state FROM aiders.aiders_telemetry WHERE drone_id = %s ORDER BY id DESC LIMIT 1"
    params = (_droneId,)
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
        "multispectral_available, lidar_available, drone_movement_available, build_map_activated, type) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _drone["name"], _drone["ip"], _drone["model"], _drone["cameraModel"], datetime.now(timezone), 1,
        _drone["ballisticAvailable"], _drone["waterSamplerAvailable"], _drone["weatherStationAvailable"],
        _drone["multispectralAvailable"], _drone["lidarAvailable"], 0, 0, _drone["type"]
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

def updateDroneDetectionStatus(_droneId, _status):
    query = "UPDATE aiders_detection SET detection_status = %s WHERE drone_id = %s "
    params = (_status, _droneId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def updateDroneDetectionSessionStatus(_droneId, _status):
    query = "UPDATE aiders_detectionsession SET is_active = %s WHERE drone_id = %s "
    params = (_status, _droneId)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDroneTelemetry(_droneId, _secondsOn, _rosMsg, _missionLogId, _operationId, _fov_polygon):
    query = (
        "INSERT INTO aiders_telemetry "
        "(drone_id, lat, lon, alt, heading, velocity, "
        "gps_signal, satellites, homeLat, homeLon, drone_state, mission_log_id, "
        "gimbal_angle, water_sampler_in_water, battery_percentage, operation_id, secondsOn, fov_coordinates, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _droneId, _rosMsg.latitude, _rosMsg.longitude, _rosMsg.altitude, _rosMsg.heading, _rosMsg.velocity,
        _rosMsg.gpsSignal, _rosMsg.satelliteNumber, _rosMsg.homeLatitude, _rosMsg.homeLongitude, _rosMsg.droneState, _missionLogId,
        _rosMsg.gimbalAngle, False, _rosMsg.batteryPercentage, _operationId, _secondsOn, json.dumps(_fov_polygon), datetime.now(timezone)
    )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def getDroneLatestTelemetryId(_droneId):
    query = f"SELECT id FROM aiders.aiders_telemetry WHERE drone_id = %s ORDER BY time DESC LIMIT 1"
    params = (_droneId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result[0]

def saveDroneError(_droneId, _errorMsg):
    query = f"INSERT INTO aiders_errormessage SET drone_id = %s, message = %s, time = %s"
    params = (_droneId, _errorMsg, datetime.now(timezone))
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDroneWeatherStationData(_data, _droneId):
    query = (
        "INSERT INTO aiders_weatherstation "
        "(drone_id, wind_speed, wind_direction, temperature, pressure, humidity, heading, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (_droneId, _data.speed, _data.direction, _data.temperature, _data.pressure, _data.humidity, _data.heading, datetime.now(timezone))
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDroneMonitoringData(_data, _droneId):
    query = (
        "INSERT INTO aiders_controldevice "
        "(drone_id, cpu_usage, cpu_core_usage, cpu_core_frequency, cpu_temp, cpu_fan_RPM, "
        "gpu_usage, gpu_frequency, gpu_temp, ram_usage, swap_usage, swap_cache, emc_usage, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (_droneId, _data["cpu_usage"], _data["cpu_core_usage"], _data["cpu_core_frequency"], _data["cpu_temp"], _data["cpu_fan_RPM"], 
        _data["gpu_usage"], _data["gpu_frequency"], _data["gpu_temp"], _data["ram_usage"], _data["swap_usage"], _data["swap_cache"], _data["emc_usage"], datetime.now(timezone)
        )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def updateDroneCollaborationRequestStatus(_id, _status):
    query = f"UPDATE aiders_drone SET requested_collaboration = {_status} WHERE id = %s"
    params = (_id,)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def updateDroneCollaborationRespondingStatus(_id, _status):
    query = f"UPDATE aiders_drone SET responding_to_collaboration = {_status} WHERE id = %s"
    params = (_id,)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveDroneLidarPointDataInBatches(_points, _telemetryId, _lidarSessionId):
    query = (
        "INSERT INTO aiders_lidarpoint"
        "(x, y, z, intensity, red, green, blue, telemetry_id, lidar_point_session_id, time)"
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = []
    for point in _points:
        params.append((point["x"], point["y"], point["z"], point["intensity"], point["red"], point["green"], point["blue"], _telemetryId, _lidarSessionId, datetime.now(timezone)))
    connector = MySQLConnector()
    connector.executeBatchQuery(query,params)  
    connector.close()

#####################################
############# DEVICES ###############
#####################################


def getDeviceByNameAndModel(_name, _model):
    query = f"SELECT id, is_connected_with_platform FROM aiders_device WHERE name = %s AND model = %s LIMIT 1"
    params = (_name, _model)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getDeviceOperationId(_deviceId):
    query = f"SELECT operation_id FROM aiders.aiders_device WHERE id = %s"
    params = (_deviceId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def updateDeviceConnectionStatus(_name, _model, _status):
    query = f"UPDATE aiders_device SET is_connected_with_platform = {_status} WHERE name = %s AND model = %s"
    params = (_name, _model)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def deactivateDeviceSessionAndCreateNew(_deviceId, _deviceName, _operationId):
    updateQuery = f"UPDATE aiders_devicesession SET is_active = 0 WHERE device_id = %s"
    updateParams = (_deviceId, )

    currentDatetime = datetime.now(timezone)
    formattedDatetime = currentDatetime.strftime("%Y-%m-%d_%H.%M.%S")
    folderPath = f"Device_Images_{_deviceName}_{formattedDatetime}"
    insertQuery = (
        "INSERT INTO aiders_devicesession "
        "(device_id, start_time, is_active, operation_id, user_id, folder_path) "
        "VALUES (%s, %s, %s, %s, %s, %s)"
    )
    insertParams = (
        _deviceId, currentDatetime, 1, _operationId, 1, folderPath
    )

    connector = MySQLConnector()
    connector.executeQuery(updateQuery, updateParams, False)
    sessionId = connector.executeQuery(insertQuery, insertParams, False)
    connector.close()
    return sessionId

def saveDevice(_name, _model, _ip):
    query = f"INSERT INTO aiders_device (name, operator, model, ip, is_connected_with_platform) VALUES (%s, %s, %s, %s, 1)"
    params = (_name, _name, _model, _ip)
    connector = MySQLConnector()
    deviceId = connector.executeQuery(query, params, False)
    connector.close()
    return deviceId

def saveDeviceTelemetry(_deviceId, _secondsOn, _rosMsg, _operationId):
    query = (
        "INSERT INTO aiders_devicetelemetry "
        "(device_id, latitude, longitude, altitude, heading, "
        "orientation_x, orientation_y, orientation_z, accelerometer_x, accelerometer_y, accelerometer_z, "
        "gyroscope_x, gyroscope_y, gyroscope_z, geomagnetic_x, geomagnetic_y, geomagnetic_z, "
        "light, step, pressure, proximity, battery_percentage, operation_id, secondsOn, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _deviceId, _rosMsg.latitude, _rosMsg.longitude, _rosMsg.altitude, _rosMsg.heading,
        _rosMsg.orientation_x, _rosMsg.orientation_y, _rosMsg.orientation_z, _rosMsg.accelerometer_x, _rosMsg.accelerometer_y, _rosMsg.accelerometer_z,
        _rosMsg.gyroscope_x, _rosMsg.gyroscope_y, _rosMsg.gyroscope_z, _rosMsg.geomagnetic_x, _rosMsg.geomagnetic_y, _rosMsg.geomagnetic_z,
        _rosMsg.light, _rosMsg.step, _rosMsg.pressure, _rosMsg.proximity, _rosMsg.battery_percentage, _operationId, _secondsOn, datetime.now(timezone)
    )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()


##################################
############# LORA ###############
##################################


def getLoraMasterByName(_name):
    query = f"SELECT id, is_connected_with_platform FROM aiders_baloramaster WHERE name = %s LIMIT 1"
    params = (_name,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def getLoraMasterOperationId(_loraId):
    query = f"SELECT operation_id FROM aiders.aiders_baloramaster WHERE id = %s"
    params = (_loraId,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def updateLoraMasterConnectionStatus(_name, _status):
    query = f"UPDATE aiders_baloramaster SET is_connected_with_platform = {_status} WHERE name = %s"
    params = (_name,)
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveLoraMaster(_name):
    query = f"INSERT INTO aiders_baloramaster (name, time, is_connected_with_platform) VALUES (%s, %s, 1)"
    params = (_name, datetime.now(timezone))
    connector = MySQLConnector()
    loraMasterId = connector.executeQuery(query, params, False)
    connector.close()
    return loraMasterId

def getLoraClientByName(_name):
    query = "SELECT id FROM aiders_balora WHERE name = %s LIMIT 1"
    params = (_name,)
    connector = MySQLConnector()
    result = connector.executeQuery(query, params, True)
    connector.close()
    return result

def saveLoraClient(_clientName, _masterId):
    query = "INSERT INTO aiders_balora (name, baloraMaster_id, time) VALUES (%s, %s, %s)"
    params = (_clientName, _masterId, datetime.now(timezone))
    connector = MySQLConnector()
    loraClientId = connector.executeQuery(query, params, False)
    connector.close()
    return loraClientId

def saveLoraTelemetry(_obj):
    query = (
        "INSERT INTO aiders_baloratelemetry "
        "(baloraMaster_id, balora_id, secondsOn, latitude, longitude, "
        "pm1, pm25, acc_x, acc_y, acc_z, received_signal_strength_indication, operation_id, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
    )
    params = (
        _obj["baloraMaster_id"], _obj["balora_id"], _obj["secondsOn"], _obj["latitude"], _obj["longitude"],
        _obj["pm1"], _obj["pm25"], _obj["acc_x"], _obj["acc_y"], _obj["acc_z"], _obj["received_signal_strength_indication"], 
        _obj["operation_id"], datetime.now(timezone)
    )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()

def saveLoraMonitor(_obj):
    query = (
            "INSERT INTO aiders_baloramonitor"
            "(baloraMaster_id, balora_id, secondsOn, battery_percentage, cpu_usage, "
            "heap_memory, received_signal_strength_indication, operation_id, time) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)"
        )
    params = (
            _obj["baloraMaster_id"], _obj["balora_id"], _obj["secondsOn"], _obj["battery_percentage"], _obj["cpu_usage"],
            _obj["heap_memory"], _obj["received_signal_strength_indication"], _obj["operation_id"], datetime.now(timezone)
        )
    connector = MySQLConnector()
    connector.executeQuery(query, params, False)
    connector.close()