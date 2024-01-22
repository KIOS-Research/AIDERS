import os
import json
import requests

#####################################
############### CV ##################
#####################################

cvBaseUrl = f"http://localhost:{os.environ['CV_API_PORT']}"

def postDetectionStartToCv(_userId, _operationId, _droneId, _droneName, _detectionType):
    url = f"{cvBaseUrl}/startDetection"
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    payload = {
        "userId": _userId,
        "operationId": _operationId,
        "droneId": _droneId,
        "droneName": _droneName,
        "detectionType": _detectionType,
    }
    response = requests.post(url, headers=headers, data=json.dumps(payload))
    return response.json()


def postDetectionStopToCv(_operationId, _droneId, _droneName):
    url = f"{cvBaseUrl}/stopDetection"
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    payload = {
        "operationId": _operationId,
        "droneId": _droneId,
        "droneName": _droneName,
    }
    response = requests.post(url, headers=headers, data=json.dumps(payload))
    return response.json()

#####################################
############## ROS ##################
#####################################

rosBaseUrl = f"http://localhost:{os.environ['ROS_API_PORT']}"

def postRequestForLidarStartOrStop(_droneId, _droneName, _lidarSessionId, _command,):
    url = f"{rosBaseUrl}/droneStartOrStopLidar"
    payload = {
        "droneId": _droneId,
        "droneName": _droneName,
        "lidarSessionId": _lidarSessionId,
        "command": _command,
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers) # send the POST request
    return response.json()

def postRequestForOpenWaterSamplingValve(_droneName):
    url = f"{rosBaseUrl}/droneOpenWaterSamplingValve"
    payload = {
        "droneName": _droneName,
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()

# Safedrone Post data of GPS reliability
def postRequestForReliableNavigationValue(_droneId, _droneName, _value):
    url = f"{rosBaseUrl}/droneReliableNavigationValue"
    payload = {
        "droneId": _droneId,
        "droneName": _droneName,
        "reliableNavigation": _value
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()


#####################################
############## MAVLINK ##############
#####################################


mavlinkBaseUrl = f"http://localhost:{os.environ['MAV_API_PORT']}"

def postRequestForMavlink(_urlSlug, _payload):
    url = f"{mavlinkBaseUrl}/{_urlSlug}"
    payload = _payload
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()


#####################################
############## SESAME ###############
#####################################

ssmBaseUrl = f"http://localhost:{os.environ['SSM_API_PORT']}"

def postRequestForSinadraStartOrStop(_operationId, _command):
    url = f"{ssmBaseUrl}/sinadraStartOrStop"
    payload = {
        "operationId": _operationId,
        "command": _command,
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()

#####################################
############## SAFEML ###############
#####################################

safemlBaseUrl = f"http://localhost:{os.environ['SAFEML_API_PORT']}"

def postDetectionStartToSafeML(_userId, _operationId, _droneId, _droneName, _detectionType):
    url = f"{safemlBaseUrl}/startDetection"
    payload = {
        "userId": _userId,
        "operationId": _operationId,
        "droneId": _droneId,
        "droneName": _droneName,
        "detectionType": _detectionType
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()

def postDetectionStopToSafeML(_operationId, _droneId, _droneName, _detectionType):
    url = f"{safemlBaseUrl}/stopDetection"
    payload = {
        "operationId": _operationId,
        "droneId": _droneId,
        "droneName": _droneName,
        "detectionType": _detectionType
    }
    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()