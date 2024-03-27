import json
import re
import time
import math

# custom libs
import database.queries
import httpRequests
import ros.publishers
import ros.subscribers

#####################################
############## DRONES ###############
#####################################


# runs when a drone requests to connect or disconnect
def droneConnectedOrDisconnected(_rosMsg):
    jsonData = json.loads(str(_rosMsg.data))
    droneName = jsonData["DroneName"]
    droneModel = jsonData["Model"]
    droneIp = jsonData.get("DroneIp")
    cameraModel = jsonData.get("cameraName")
    ballisticAvailable = jsonData.get("Ballistic", False)
    lidarAvailable = jsonData.get("Lidar", False)
    multispectralAvailable = jsonData.get("Multispectral", False)
    waterSamplerAvailable = jsonData.get("WaterSampler", False)
    weatherStationAvailable = jsonData.get("WeatherStation", False)    
    droneIsRequestingConnection = jsonData["Connected"]
    drone = database.queries.getDroneByNameAndModel(droneName, droneModel)  # retrieve drone from database
    if droneIsRequestingConnection == "True":
        if drone is not None:
            droneId = drone[0]
            droneIsConnected = drone[1]
            if droneIsConnected == 1:
                print(f"\U0001F44D Drone '{droneName}' is already connected")
            else:
                database.queries.updateDroneConnectionStatus(droneId, 1)  # flag drone as connected in the database
                print(f"\U0001F504 Drone '{droneName}' has re-connected")
        else:
            droneObj = {
                "name": droneName,
                "model": droneModel,
                "ip": droneIp,
                "cameraModel": cameraModel,
                "ballisticAvailable": ballisticAvailable,
                "waterSamplerAvailable": waterSamplerAvailable,
                "weatherStationAvailable": weatherStationAvailable,
                "multispectralAvailable": multispectralAvailable,
                "lidarAvailable": lidarAvailable,
                "type": "ROS"
            }            
            droneId = database.queries.saveDrone(droneObj)      # create drone entry in the database
            database.queries.createDroneDetectionEntry(droneId) # create entry in aiders_detection
            print(f"\U0001F680 Drone '{droneName}' has connected")

        ros.publishers.performHandshake(droneName)                          # send connection acknowledgement
        ros.subscribers.createDroneSubscribers(droneId, droneName)          # create ROS subscribers for this drone
        try:
            httpRequests.startDroneLiveStreamCapture(droneId, droneName)    # request stream capture start
        except:
            print("\U0001F6AB Live stream capture service is unreachable")        
    else:
        if drone is not None:
            droneId = drone[0]        
            database.queries.updateDroneConnectionStatus(droneId, 0)                        # flag drone as disconnected
            # database.queries.updateDroneDetectionStatus(droneId, "DETECTION_DISCONNECTED")  # flag detection as disconnected
            # database.queries.updateDroneDetectionSessionStatus(droneId, 0)                  # flag detection status as inactive
            ros.subscribers.stopDroneSubscribers(droneName)                                 # stop subscribers
            httpRequests.stopDroneDetector(droneId, droneName)                              # request detection end
            print(f"\U0001F480 Drone '{droneName}' has disconnected")


# runs when a new drone telemetry message is received
def droneTelemetryReceived(_rosMsg, _args):
    startTime = _args[0]
    droneId = _args[1]
    connectionDuration = round((time.time() - startTime), 2)
    missionLogId = None
    if _rosMsg.droneState == "In_Mission":
        mission = database.queries.getDroneMissionLogId(droneId)
        missionLogId = mission[0]
    operation = database.queries.getDroneOperationId(droneId)
    operationId = operation[0]

    fov_polygon = []
    if _rosMsg.altitude > 1:
        from camera_footprint_calculator import CameraFootprintCalculator
        c = CameraFootprintCalculator()
        fov_polygon = c.getBoundingPolygon(
            _rosMsg.latitude, 
            _rosMsg.longitude,   
            math.radians(68),
            math.radians(40),
            _rosMsg.altitude, 
            math.radians(0),
            math.radians(_rosMsg.gimbalAngle+90),
            math.radians(_rosMsg.heading+180 % 360))    

    database.queries.saveDroneTelemetry(droneId, connectionDuration, _rosMsg, missionLogId, operationId, fov_polygon) # save telemetry data to the database    



# runs when a new drone error message is received
def droneErrorReceived(_rosMsg, _args):
    jsonData = json.loads(_rosMsg.data)
    errorMessage = str(jsonData)
    droneId = _args[0]
    database.queries.saveDroneError(droneId, errorMessage) # save error data to the database   


# runs when new weather data is received from drone
def droneWeatherDataReceived(_rosMsg, _args):
    droneId = _args[0]
    database.queries.saveDroneWeatherStationData(_rosMsg, droneId)  # save weather data to DB


# received monitoring data from the jetson
def droneMonitoringDataReceived(_rosMsg, _args):
    droneId = _args[0]

    cpu_usage = 0
    cpu_core_usage = ''
    for core_usage in list(_rosMsg.cpu_core_usage):
        cpu_usage = cpu_usage+float(core_usage)
        if cpu_core_usage != '':
            cpu_core_usage = cpu_core_usage+', '+str(core_usage)
        else:
            cpu_core_usage = cpu_core_usage+str(core_usage)
    cpu_usage = cpu_usage/len(_rosMsg.cpu_core_usage)
    ram_usage = float(_rosMsg.ram_use)*100/float(_rosMsg.ram_max)
    swap_usage = float(_rosMsg.swap_use)*100/float(_rosMsg.swap_max)

    data = {
        "cpu_usage": round(cpu_usage, 1),
        "cpu_core_usage": str(cpu_core_usage),
        "cpu_core_frequency": str(_rosMsg.cpu_core_freq),
        "cpu_temp": round(float(_rosMsg.cpuTemp)/1000, 1),
        "cpu_fan_RPM": round(float(_rosMsg.cpuFanRPM), 1),
        "gpu_usage": round(float(_rosMsg.gr3d_usage), 1),
        "gpu_frequency": round(float(_rosMsg.gr3d_freq), 1),
        'gpu_temp': round(float(_rosMsg.gpuTemp)/1000, 1),
        "ram_usage": round(ram_usage, 1),
        "swap_usage": round(swap_usage, 1),
        "swap_cache": round(float(_rosMsg.swap_cache), 1),
        "emc_usage": round(float(_rosMsg.emc_usage), 1),
    }
    database.queries.saveDroneMonitoringData(data, droneId)  # save weather data to DB
    pass

# runs when a new drone lidar message is received
def droneLidarReceived(_rosMsg, _args):
    droneId = _args[0]
    lidarSessionId = _args[1]
    try:
        # Works only with android application
        listOfPoints = json.loads(re.sub(r"(\w+):", r'"\1":', json.loads(_rosMsg.data)["PointCloud"]))
    except:
        # Works only with aiders simulator
        listOfPoints = json.loads(_rosMsg.data)["PointCloud"]
    telemetryId = database.queries.getDroneLatestTelemetryId(droneId)
    database.queries.saveDroneLidarPointDataInBatches(listOfPoints, telemetryId, lidarSessionId)


# runs when a drone requests collaboration
def droneRequestingCollaboration(_rosMsg, _args):

    droneId = _args[0]
    status = 0
    if(_rosMsg.data == True):
        status = 1
    database.queries.updateDroneCollaborationRequestStatus(droneId, status)

    print(f"\U0001F480 Drone ID '{droneId}' is requesting collaboration")
    pass


# runs when a drone is responding to a collaboration request
def droneRespondingToCollaboration(_rosMsg, _args):
    
    droneId = _args[0]
    status = 0
    if(_rosMsg.data == True):
        status = 1
    database.queries.updateDroneCollaborationRespondingStatus(droneId, status)

    print(f"\U0001F480 Drone ID '{droneId}' is responding to collaboration")
    pass


#####################################
############# DEVICES ###############
#####################################


# runs when a device requests to connect or disconnect
def deviceConnectedOrDisconnected(_rosMsg):
    jsonData = json.loads(str(_rosMsg.data))    # decode JSON data
    deviceName = jsonData["Name"]
    deviceModel = jsonData["Model"]
    deviceIp = jsonData["Ip"]
    deviceIsRequestingConnection = jsonData["Connected"]
    if deviceIsRequestingConnection == "True":
        device = database.queries.getDeviceByNameAndModel(deviceName, deviceModel)  # retrieve device from database
        if device is not None:
            deviceId = device[0]
            deviceIsConnected = device[1]
            if deviceIsConnected == 1:
                print(f"\U0001F44D Device '{deviceName}' is already connected")
            else:
                database.queries.updateDeviceConnectionStatus(deviceName, deviceModel, 1)  # flag device as connected in the database
                # if device belongs to an operation create a new device session
                operation = database.queries.getDeviceOperationId(deviceId)
                if operation is not None:
                    operationId = operation[0]
                    database.queries.deactivateDeviceSessionAndCreateNew(deviceId, deviceName, operationId)

                print(f"\U0001F504 Device '{deviceName}' has re-connected")
        else:
            deviceId = database.queries.saveDevice(deviceName, deviceModel, deviceIp)   # create device entry in the database
            print(f"\U0001F680 Device '{deviceName}' has connected")

        ros.publishers.performHandshake(deviceName)                         # send connection acknowledgement
        ros.subscribers.createDeviceSubscribers(deviceId, deviceName) # create ROS subscribers for this device
    else:
        database.queries.updateDeviceConnectionStatus(deviceName, deviceModel, 0)  # flag device as disconnected in the database
        ros.subscribers.stopDeviceSubscribers(deviceName)                              # stop threads
        print(f"\U0001F480 Device '{deviceName}' has disconnected")


# runs when a new device telemetry message is received
def deviceTelemetryReceived(_rosMsg, _args):
    startTime = _args[0]
    deviceId = _args[1]
    connectionDuration = round((time.time() - startTime), 2)
    operation = database.queries.getDeviceOperationId(deviceId)
    operationId = operation[0]
    database.queries.saveDeviceTelemetry(deviceId, connectionDuration, _rosMsg, operationId) # save telemetry data to the database    


##################################
############# LORA ###############
##################################


# runs when a lora requests to connect or disconnect
def loraMasterConnectedOrDisconnected(_rosMsg):
    data = _rosMsg.data.split(",")
    loraMasterName = data[0]
    loraIsRequestingConnection = data[1]
    if loraIsRequestingConnection == "True":
        lora = database.queries.getLoraMasterByName(loraMasterName)  # retrieve lora from database
        if lora is not None:
            loraMasterId = lora[0]
            loraIsConnected = lora[1]
            if loraIsConnected == 1:
                print(f"\U0001F44D Lora '{loraMasterName}' is already connected")
            else:
                database.queries.updateLoraMasterConnectionStatus(loraMasterName, 1)  # flag lora as connected in the database
                print(f"\U0001F504 Lora '{loraMasterName}' has re-connected")
        else:
            loraMasterId = database.queries.saveLoraMaster(loraMasterName)   # create lora entry in the database
            print(f"\U0001F680 Lora '{loraMasterName}' has connected")

        ros.publishers.performHandshake(loraMasterName)                             # send connection acknowledgement
        ros.subscribers.createLoraMasterSubscribers(loraMasterId, loraMasterName)   # create ROS subscribers for this lora
    else:
        database.queries.updateLoraMasterConnectionStatus(loraMasterName, 0)  # flag lora as disconnected in the database
        ros.subscribers.stopLoraMasterSubscribers(loraMasterName)             # stop threads
        print(f"\U0001F480 Lora '{loraMasterName}' has disconnected")


# runs when a new lora telemetry message is received
def loraTelemetryReceived(_rosMsg, _args):
    data = _rosMsg.data.split(",")
    startTime = _args[0]
    loraMasterId = _args[1]
    connectionDuration = round((time.time() - startTime), 2)
    operation = database.queries.getLoraMasterOperationId(loraMasterId)
    operationId = operation[0]
    loraClientName = data[0]
    loraClient = database.queries.getLoraClientByName(loraClientName) # retrieve lora client
    if loraClient is not None:
        loraClientId = loraClient[0]
    else:
        loraClientId = database.queries.saveLoraClient(loraClientName, loraMasterId) # create lora client

    try:
        telemetryObject = {
            "baloraMaster_id": loraMasterId,
            "balora_id": loraClientId,
            "secondsOn": connectionDuration,
            "latitude": float(data[2]),
            "longitude": float(data[3]),
            "pm1": float(data[4]),
            "pm25": float(data[5]),
            "acc_x": float(data[6]),
            "acc_y": float(data[7]),
            "acc_z": float(data[8]),
            "received_signal_strength_indication": float(data[9]),
            "operation_id": operationId
        }
        database.queries.saveLoraTelemetry(telemetryObject) # save telemetry data to the database
    except:
        print("Invalid Lora telemetry data.")


# runs when a new lora monitor message is received
def loraMonitorDataReceived(_rosMsg, _args):
    data = _rosMsg.data.split(",")
    startTime = _args[0]
    loraMasterId = _args[1]
    connectionDuration = round((time.time() - startTime), 2)
    operation = database.queries.getLoraMasterOperationId(loraMasterId)
    operationId = operation[0]
    loraClientName = data[0]
    loraClient = database.queries.getLoraClientByName(loraClientName) # retrieve lora client
    if loraClient is not None:
        loraClientId = loraClient[0]
    else:
        loraClientId = database.queries.saveLoraClient(loraClientName, loraMasterId) # create lora client
    try:
        monitorObject = {
            "baloraMaster_id": loraMasterId,
            "balora_id": loraClientId,
            "secondsOn": connectionDuration,
            "battery_percentage": float(data[1]),
            "cpu_usage": float(data[2]),
            "heap_memory": float(data[3]),
            "received_signal_strength_indication": float(data[4]),
            "operation_id": operationId
        }
        database.queries.saveLoraMonitor(monitorObject) # save monitor data to the database
    except:
        print("Invalid Lora monitor data.")