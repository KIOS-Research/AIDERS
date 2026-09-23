#!/usr/bin/env python
import json
import os

import requests

# TODO: Change to from ROS to API ROS
# def buildMapPublisherSingleMessageMultispectral(drone_name, data, overlap):
#     publisher = rospy.Publisher("/" + drone_name + "/MultispectralBuildMapRequest", BuildMap, queue_size=10)
#     t = BuildMap()
#     t.buildmap = data
#     t.overlap = int(overlap)
#     publisher.publish(t)

def PostRequestForBuildMapStartOrStop(_droneName, _command, _interval, _connectionType):


    if _connectionType == "ROS":
        requestUrl = f"http://{os.environ['ROS_HOST']}:{os.environ['ROS_API_PORT']}/ros/droneStartOrStopBuildMap"

        payload = {
            "droneName": _droneName,
            "command": _command,
            "interval": _interval,
        }
    else:
        requestUrl = f"http://{os.environ['WSI_HOST']}:{os.environ['WSI_PORT']}/wsi/sendMessageToClient"

        payload = {
            "name": _droneName,
            "type": "buildmap",
            "msg": {
                "command": _command,
                "interval": int(_interval),
            }
        }



    headers = {
        'Content-Type': 'application/json'
    }    
    json_payload = json.dumps(payload)

    response = requests.post(requestUrl, data=json_payload, headers=headers) # send the POST request
    # TODO: check the response status code
    if response.status_code == 200:
        print("REQUEST TO ROS API SUCCESSFUL")
    else:
        print("REQUEST TO ROS API FAILED")
