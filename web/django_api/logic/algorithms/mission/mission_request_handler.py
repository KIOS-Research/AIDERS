#!/usr/bin/env python
import json
import os
import sys

import requests
from aiders import models, views

# import rospy
# from kios.msg import GpsInput, InputDJI, MissionCommandDJI, MissionDji


def publishMissionToRos(
    operationPK,
    missionType,
    drone_name,
    grid,
    captureAndStoreImages,
    missionPath,
    missionSpeed,
    missionGimbal,
    missionRepeat,
    action,
    userPK,
    dronePK,
):
    if captureAndStoreImages == "false":
        captureAndStoreImages = False
    elif captureAndStoreImages == "true":
        captureAndStoreImages = True
    if isinstance(missionGimbal, list):
        for i in range(0, len(missionGimbal)):
            if missionGimbal[i] == "":
                missionGimbal[i] = "None"
    else:
        if missionGimbal == "N":
            missionGimbal = "None"
    print(missionGimbal)


    drone = models.Drone.objects.get(drone_name=drone_name)

    if(drone.type == "MAVLINK"):
        # missionPath = [
        #     [
        #         23.771642126874877,
        #         37.96927936774028,
        #         100
        #     ],
        #     [
        #         23.779130735706563,
        #         37.96881901732853,
        #         100
        #     ],

        #     [
        #         23.768646657122062,
        #         37.96946337441829,
        #         100
        #     ],
        #     [
        #         23.771642126874877,
        #         37.96927936774028,
        #         100
        #     ]
        # ]
        # send the mission to the MAV container through an http request
        mavUrl = f"http://{os.environ['MAV_IP']}:{os.environ['MAV_API_PORT']}/mav/mission"
        print(missionPath, flush=True)
        payload = {
            "name": drone_name,
            "missionPath": missionPath,
            "missionSpeed": missionSpeed,
            "action": action,
        }
        headers = {
            'Content-Type': 'application/json'
        }    
        json_payload = json.dumps(payload)

        response = requests.post(mavUrl, data=json_payload, headers=headers) # send the POST request
        if response.status_code == 200:
            if(action == "START_MISSION"):
                # save the mission to the database
                savedSuccessfully = _saveMissionToDatabase(
                    operationPK, missionType, grid, captureAndStoreImages, missionPath, missionSpeed, missionGimbal, missionRepeat, action, userPK, dronePK
                )
            else:
                mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
                views.MissionLoggerListCreateAPIView.mission_logger_save_to_db(action, mission_logger.mission, userPK, operationPK, dronePK)        
        else:
            # TODO: handle error
            print("REQUEST TO MAV API FAILED")

    elif(drone.connection_type == "WEBSOCKETS"):
        print("SENDING MISSION TO WS DRONE", flush=True)

        # send the mission to the WSI container through an http request
        wsUrl = f"http://{os.environ['WSI_HOST']}:{os.environ['WSI_PORT']}/wsi/sendMessageToClient"
        payload = {
            "name": drone_name,
            "type": "mission",
            "msg": {
                "action": action,
                "grid": grid,
                "missionSpeed": int(missionSpeed),
                "missionGimbal": missionGimbal,
                "missionRepeat": missionRepeat,
                "captureAndStoreImages": captureAndStoreImages,
                "missionPath": missionPath,
            }
        }
        headers = {
            'Content-Type': 'application/json'
        }    
        json_payload = json.dumps(payload)

        response = requests.post(wsUrl, data=json_payload, headers=headers)
        print("RESPONSE FROM WS API", response, flush=True)

        if response.status_code == 200:
            if(action == "START_MISSION"):
                print("SAVING MISSION TO DB", flush=True)
                # save the mission to the database
                savedSuccessfully = _saveMissionToDatabase(
                    operationPK, missionType, grid, captureAndStoreImages, missionPath, missionSpeed, missionGimbal, missionRepeat, action, userPK, dronePK
                )
            else:
                mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
                views.MissionLoggerListCreateAPIView.mission_logger_save_to_db(action, mission_logger.mission, userPK, operationPK, dronePK)        
        else:
            # TODO: handle error
            print("REQUEST TO WSI API FAILED")
    else:

        # send the mission to the ROS container through an http request
        rosUrl = f"http://{os.environ['ROS_HOST']}:{os.environ['ROS_API_PORT']}/ros/droneMission"
        payload = {
            "droneId": dronePK,
            "droneName": drone_name,
            "action": action,
            "grid": grid,
            "missionSpeed": missionSpeed,
            "missionGimbal": missionGimbal,
            "missionRepeat": missionRepeat,
            "captureAndStoreImages": captureAndStoreImages,
            "missionPath": missionPath,
        }
        headers = {
            'Content-Type': 'application/json'
        }    
        json_payload = json.dumps(payload)

        response = requests.post(rosUrl, data=json_payload, headers=headers) # send the POST request
        if response.status_code == 200:
            if(action == "START_MISSION"):
                # save the mission to the database
                savedSuccessfully = _saveMissionToDatabase(
                    operationPK, missionType, grid, captureAndStoreImages, missionPath, missionSpeed, missionGimbal, missionRepeat, action, userPK, dronePK
                )
            else:
                mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
                views.MissionLoggerListCreateAPIView.mission_logger_save_to_db(action, mission_logger.mission, userPK, operationPK, dronePK)        
        else:
            # TODO: handle error
            print("REQUEST TO ROS API FAILED")




def _saveMissionToDatabase(
    operationPK, missionType, grid, captureAndStoreImages, missionPath, missionSpeed, missionGimbal, missionRepeat, action, userPK, dronePK
):
    # num_results = models.Mission.objects.filter().count()
    if missionType == "NORMAL_MISSION" or missionType == "SEARCH_AND_RESCUE_MISSION" or missionType == "GRID_MISSION":
        missionPathCorrectFormat = [{"point": {"latitude": point[1], "longitude": point[0]}} for point in missionPath]
        if isinstance(missionGimbal, list):
            missionGimbal = ",".join(str(x) for x in missionGimbal)
        if isinstance(missionSpeed, list):
            missionSpeed = ",".join(str(x) for x in missionSpeed)
        missionObj = {
            # 'action': action,
            "mission_type": missionType,
            "operation": operationPK,
            "grid": grid,
            "captureAndStoreImages": captureAndStoreImages,
            "user": userPK,
            "mission_points": missionPathCorrectFormat,
            "mission_speeds": missionSpeed,
            "mission_gimbal": missionGimbal,
            "repeat": missionRepeat,
        }
        views.MissionListCreateAPIView.mission_save_to_db(missionObj, dronePK, userPK, operationPK)


def main():
    global BASE_URL_DRONE_API, FROM_DOCKER
    if len(sys.argv) < 2:
        # print("Please provide the drone's name!")
        print("\nPlease make sure you provide the following arguments:" "\n1)Drone Name (e.g kios_mavic2h)")
        exit()
    else:
        dji_name = sys.argv[1]

    publishMissionToRos(dji_name)
    # try:
    #     if sys.argv[0] != "":  # This is the case where script is started as separate terminal and thus, rospy.init_noe should be claled
    #         rospy.init_node("dji_input", anonymous=True)
    #     sys.argv = []
    #     print("** Mission request handler script for drone : " + dji_name + " has started**")

    # except rospy.ROSInterruptException:
    #     pass

    # finally:
    #     pass


if __name__ == "__main__":
    main()
