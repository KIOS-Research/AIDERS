#!/usr/bin/env python
import json
import pprint
import sys
from aiders import views
import requests
import rospy
from aiders import models, serializers
from kios.msg import GpsInput, InputDJI, MissionCommandDJI, MissionDji


def publishMissionToRos(operationPK, missionType, drone_name, grid, captureAndStoreImages, missionPath, action, userPK, dronePK):
    savedSuccessfully = _saveMissionToDatabase(operationPK, missionType, grid, captureAndStoreImages, missionPath, action, userPK, dronePK)

    publisher = rospy.Publisher("/" + drone_name + "/Mission", MissionDji, queue_size=10)

    t = MissionDji()
    t.name = drone_name
    # t.header.stamp = rospy.get_rostime()

    if not rospy.core.is_initialized():
        print("WILL NOW INITIALIZE ROSPY")
        rospy.init_node('dji_input', anonymous=True)

    t.header.frame_id = drone_name
    if (action == models.MissionLog.START_MISSION):
        s = MissionCommandDJI()
        s.missionCommand = s.start
        t.missionCommand = s
        t.grid = grid
        t.captureAndStoreImages = captureAndStoreImages
        for i in range(0, len(missionPath)):
            k = GpsInput()
            k.latitude = missionPath[i][1]
            k.longitude = missionPath[i][0]
            k.altitude = float(missionPath[i][2])
            k.speed = 4.0
            k.stayTime = 0
            k.photo = False
            t.gpsInput.append(k)
        publisher.publish(t)
        print("MISSION PUBLISHED TO ROS: ", t)
    elif (action == models.MissionLog.PAUSE_MISSION):
        s = MissionCommandDJI()
        s.missionCommand = s.pause
        t.missionCommand = s
        publisher.publish(t)
        mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
        views.MissionLoggerListCreateAPIView.mission_logger_save_to_db('PAUSE_MISSION', mission_logger.mission, userPK, operationPK, dronePK)
        print("PAUSE_MISSION")
    elif (action == models.MissionLog.RESUME_MISSION):
        s = MissionCommandDJI()
        s.missionCommand = s.resume
        t.missionCommand = s
        publisher.publish(t)
        mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
        views.MissionLoggerListCreateAPIView.mission_logger_save_to_db('RESUME_MISSION', mission_logger.mission, userPK, operationPK, dronePK)
        print("RESUME_MISSION")
    elif (action == models.MissionLog.CANCEL_MISSION):
        s = MissionCommandDJI()
        s.missionCommand = s.stop
        t.missionCommand = s
        publisher.publish(t)
        mission_logger = models.MissionLog.objects.filter(user=userPK, operation=operationPK, action="START_MISSION").last()
        views.MissionLoggerListCreateAPIView.mission_logger_save_to_db('CANCEL_MISSION', mission_logger.mission, userPK, operationPK, dronePK)
        print("CANCEL_MISSION")




def _saveMissionToDatabase(operationPK, missionType, grid, captureAndStoreImages, missionPath, action, userPK, dronePK):
    
    # num_results = models.Mission.objects.filter().count()
    if missionType == "NORMAL_MISSION" or missionType == "SEARCH_AND_RESCUE_MISSION" or missionType == "GRID_MISSION":
        missionPathCorrectFormat = [{"point": {"latitude": point[1],"longitude": point[0]}} for point in
                      missionPath]
        missionObj = {
            # 'action': action,
            'mission_type': missionType,
            'operation': operationPK,
            'grid': grid,
            'captureAndStoreImages': captureAndStoreImages,
            'user': userPK,
            'mission_points': missionPathCorrectFormat
        }
        views.MissionListCreateAPIView.mission_save_to_db(missionObj, dronePK, userPK, operationPK)
    # else:
    #     mission = models.Mission.objects.filter()

# def saveMissionLoggerData(action, mission, userPK, operationPK, dronePK):
#     if models.Mission.objects.get(pk=mission.pk).mission_type == 'SEARCH_AND_RESCUE_MISSION':
#         algorithm=models.Algorithm.objects.filter(algorithm_name='CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM',user=userPK, operation=operationPK).last()
#         algorithmPK=algorithm.pk
#     else:
#         algorithmPK=None
#     missionLoggerData={
#         'action': action,
#         'mission': models.Mission.objects.get(pk=mission.pk).pk,
#         'user': userPK,
#         'operation': operationPK,
#         'drone': dronePK,
#         'algorithm': algorithmPK
#     }
    
#     serializerMissionLogger = serializers.MissionLoggerSerializer(data=missionLoggerData)
#     if serializerMissionLogger.is_valid():
#         createdMissionLogger = serializerMissionLogger.save()
#         print("Logger saved")
#     else:
#         print('Logger Not saved')
#         print("ERRORS: ", serializerMissionLogger.errors)

def main():
    global BASE_URL_DRONE_API, FROM_DOCKER
    if (len(sys.argv) < 2):
        # print("Please provide the drone's name!")
        print("\nPlease make sure you provide the following arguments:"
              "\n1)Drone Name (e.g kios_mavic2h)")
        exit()
    else:
        dji_name = sys.argv[1]

    try:
        if sys.argv[
            0] != "":  # This is the case where script is started as separate terminal and thus, rospy.init_noe should be claled
            rospy.init_node('dji_input', anonymous=True)
        sys.argv = []
        print("** Mission request handler script for drone : " + dji_name + " has started**")
        publishMissionToRos(dji_name)

    except rospy.ROSInterruptException:
        pass

    finally:
        pass


if __name__ == '__main__':
    main()
