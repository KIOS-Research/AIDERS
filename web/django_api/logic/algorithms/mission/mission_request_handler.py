#!/usr/bin/env python
import os
import sys
import requests
import json
from aiders import views
from aiders import models

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
        # send the mission to the MAV container through an http request
        mavUrl = f"http://localhost:{os.environ['MAV_API_PORT']}/mission"
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

    else:

        # send the mission to the ROS container through an http request
        rosUrl = f"http://localhost:{os.environ['ROS_API_PORT']}/droneMission"
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



    # publisher = rospy.Publisher("/" + drone_name + "/Mission", MissionDji, queue_size=10)

    # t = MissionDji()
    # t.name = drone_name
    # # t.header.stamp = rospy.get_rostime()

    # if not rospy.core.is_initialized():
    #     print("WILL NOW INITIALIZE ROSPY")
    #     rospy.init_node("dji_input", anonymous=True)

    # t.header.frame_id = drone_name
    # if action == models.MissionLog.START_MISSION:
    #     s = MissionCommandDJI()
    #     s.missionCommand = s.start
    #     t.missionCommand = s
    #     t.grid = grid
    #     t.captureAndStoreImages = captureAndStoreImages
    #     t.repeat = missionRepeat
    #     tempSpeed = ""
    #     tempGimbal = ""
    #     for i in range(0, len(missionPath)):
    #         if isinstance(missionSpeed, str):
    #             tempSpeed = float(missionSpeed)
    #         elif isinstance(missionSpeed, list):
    #             tempSpeed = float(missionSpeed[i])
    #         if isinstance(missionGimbal, str):
    #             tempGimbal = missionGimbal
    #         elif isinstance(missionGimbal, list):
    #             tempGimbal = missionGimbal[i]
    #         k = GpsInput()
    #         k.latitude = missionPath[i][1]
    #         k.longitude = missionPath[i][0]
    #         k.altitude = float(missionPath[i][2])
    #         k.speed = tempSpeed
    #         k.gimbalAngle = tempGimbal
    #         k.stayTime = 0
    #         k.photo = False
    #         t.gpsInput.append(k)
    #     publisher.publish(t)
    #     print("MISSION PUBLISHED TO ROS: ", t)
    # elif action == models.MissionLog.PAUSE_MISSION:
    #     s = MissionCommandDJI()
    #     s.missionCommand = s.pause
    #     t.missionCommand = s
    #     publisher.publish(t)
    #     mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
    #     views.MissionLoggerListCreateAPIView.mission_logger_save_to_db("PAUSE_MISSION", mission_logger.mission, userPK, operationPK, dronePK)
    #     print("PAUSE_MISSION")
    # elif action == models.MissionLog.RESUME_MISSION:
    #     s = MissionCommandDJI()
    #     s.missionCommand = s.resume
    #     t.missionCommand = s
    #     publisher.publish(t)
    #     mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
    #     views.MissionLoggerListCreateAPIView.mission_logger_save_to_db("RESUME_MISSION", mission_logger.mission, userPK, operationPK, dronePK)
    #     print("RESUME_MISSION")
    # elif action == models.MissionLog.CANCEL_MISSION:
    #     s = MissionCommandDJI()
    #     s.missionCommand = s.stop
    #     t.missionCommand = s
    #     publisher.publish(t)
    #     mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
    #     views.MissionLoggerListCreateAPIView.mission_logger_save_to_db("CANCEL_MISSION", mission_logger.mission, userPK, operationPK, dronePK)
    #     print("CANCEL_MISSION")





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
