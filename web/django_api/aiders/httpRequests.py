import json
import os

import requests

#####################################
############### LSC #################
#####################################

if os.environ.get('VIDEO_AND_CV_REMOTE', 0) == "1":
    streamCaptureBaseUrl = f"http://{os.environ['NGINX_HOST']}:{os.environ['NGINX_PORT']}/remote/lsc"
else:
    streamCaptureBaseUrl = f"http://{os.environ['NGINX_HOST']}:{os.environ['NGINX_PORT']}/lsc"


def startDroneLiveStreamCapture(_droneId, _droneName):
    url = f"{streamCaptureBaseUrl}/startDroneStreamCapture"
    headers = {
        'Content-Type': 'application/json'
    }
    payload = {
        "droneId": _droneId,
        "droneName": _droneName
    }
    response = requests.post(url, headers=headers, data=json.dumps(payload))
    return response.json()




#####################################
############### CV ##################
#####################################

# Legacy CVN base URL (kept for reference / other CV endpoints)
# cvBaseUrl = f"http://{os.environ['CV_HOST']}:{os.environ['CV_API_PORT']}/cv"
if os.environ.get('VIDEO_AND_CV_REMOTE', 0) == "1":
    cvBaseUrl = f"http://{os.environ['NGINX_HOST']}:{os.environ['NGINX_PORT']}/remote/cvn"
else:
    cvBaseUrl = f"http://{os.environ['NGINX_HOST']}:{os.environ['NGINX_PORT']}/cvn"

# StreamCV microservice (replaces CVN for detection start/stop)
streamcvBaseUrl = os.environ.get(
    'STREAMCV_URL',
    f"http://{os.environ.get('STREAMCV_HOST', 'streamcv')}:{os.environ.get('STREAMCV_PORT', '8000')}",
)


def postDetectionStartToCv(_userId, _operationId, _droneId, _droneName, _detectionType):
    url = f"{streamcvBaseUrl}/api/streamcv/start/{_droneId}"
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    rtmp_url = f"rtmp://{os.environ.get('NET_IP')}/live/{_droneName}"
    payload = {
        "rtmp_url": rtmp_url,
        "operation_id": _operationId,
        "user_id": _userId,
        "detection_type": _detectionType,
    }
    response = requests.post(url, headers=headers, data=json.dumps(payload))
    return response.text


def postDetectionStopToCv(_operationId, _droneId, _droneName):
    url = f"{streamcvBaseUrl}/api/streamcv/stop/{_droneId}"
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    response = requests.post(url, headers=headers)
    return response.text

#############################################
############## ROS and WSI ##################
#############################################

rosBaseUrl = f"http://{os.environ['ROS_HOST']}:{os.environ['ROS_API_PORT']}/ros"
wsBaseUrl = f"http://{os.environ['WSI_HOST']}:{os.environ['WSI_PORT']}/wsi/sendMessageToClient"


def postRequestForLidarStartOrStop(_droneId, _droneName, _lidarSessionId, _droneType, _command):

    from .models import Drone
    connection_type = Drone.objects.get(drone_name=_droneName).connection_type # get drone's connection_type from the database

    if(connection_type == "ROS"):
        url = f"{rosBaseUrl}/droneStartOrStopLidar"
        payload = {
            "droneId": _droneId,
            "droneName": _droneName,
            "lidarSessionId": _lidarSessionId,
            "command": _command,
        }

    elif(connection_type == "WEBSOCKETS"):
        url = wsBaseUrl
        _numericCommand = 1 if _command == "START" else 0
        payload = {
            "name": _droneName,
            "type": "lidar",
            "msg": {
                "sender": _droneName,
                "session_id": int(_lidarSessionId),
                "command": int(_numericCommand)
             }
        }
        
    else:
        return {"error": "Invalid connection type. Only 'ROS' or 'WEBSOCKETS' are supported."}
    
    headers = {
            'Content-Type': 'application/json'
    }    

    json_payload = json.dumps(payload)
    response = requests.post(url, data=json_payload, headers=headers)  # send the POST request
    
    if response.status_code == 200:
        print("REQUEST TO LIDAR SUCCESSFUL")
    else:
        print("REQUEST TO LIDAR FAILED")


def postRequestForPilotNotification(_droneName, _message, _sender):
    requestUrl = f"http://{os.environ['WSI_HOST']}:{os.environ['WSI_PORT']}/wsi/sendMessageToClient"

    payload = {
        "name": _droneName,
        "type": "notification",
        "msg": {
            "message": _message,
            "sender": _sender,
        }
    }

    print(payload, flush=True)

    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(requestUrl, data=json_payload, headers=headers) # send the POST request
    # TODO: check the response status code
    if response.status_code == 200:
        print("REQUEST TO WSI API SUCCESSFUL")
        return True
    else:
        print("REQUEST TO WSI API FAILED")
        return False



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



def postRequestForDeviceNotification(_deviceName, _message, _sender):
    requestUrl = f"http://{os.environ['WSI_HOST']}:{os.environ['WSI_PORT']}/wsi/sendMessageToDevice"

    payload = {
        "name": _deviceName,
        "type": "notification",
        "msg": {
            "message": _message,
            "sender": _sender,
        }
    }

    print(payload, flush=True)

    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(requestUrl, data=json_payload, headers=headers) # send the POST request
    # TODO: check the response status code
    if response.status_code == 200:
        print("REQUEST TO WSI API SUCCESSFUL")
        return True
    else:
        print("REQUEST TO WSI API FAILED")
        return False


#####################################
############## MAVLINK ##############
#####################################


mavlinkBaseUrl = f"http://{os.environ['MAV_IP']}:{os.environ['MAV_API_PORT']}/mav"
mavlinkWSBaseUrl = f"http://{os.environ['WSM_IP']}:{os.environ['WSM_PORT']}/wsm"

def postRequestForMavlink(_urlSlug, _payload):
    
    from .models import Drone
    connection_type = Drone.objects.get(drone_name=_payload["name"]).connection_type # get drone's connection_type from the database
    baseUrl = mavlinkWSBaseUrl if connection_type == "WEBSOCKETS" else mavlinkBaseUrl # set the base url based on the connection type

    url = f"{baseUrl}/{_urlSlug}"

    print(f"*** CALLING URL: {url}", flush=True)
    payload = _payload
    headers = {
        'Content-Type': 'application/json'
    }
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)

    try:
        return response.json()
    except json.JSONDecodeError as e:
        print(f"JSONDecodeError: {e}", flush=True)
        print(f"Response content: {response.text}", flush=True)
        return {"error": "Invalid JSON response", "details": response.text}



#####################################
############# ALGORITHM #############
#####################################

algorithmBaseUrl = f"http://{os.environ['ALG_HOST']}:{os.environ['ALG_API_PORT']}/alg"

def postRequestForLidarProcess(_lidarSessionId):
    url = f"{algorithmBaseUrl}/processPointCloud"
    payload = {
        "sessionId": _lidarSessionId,
    }
    headers = {
        'Content-Type': 'application/json'
    }
    json_payload = json.dumps(payload)

    response = requests.post(url, data=json_payload, headers=headers)
    return response.json()
